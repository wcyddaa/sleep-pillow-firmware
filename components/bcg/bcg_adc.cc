// bcg_adc.cc - 压电陶瓷(BCG) 采集与三通道派生（REAL/SIM 双路径）
#include "bcg_adc.h"
#include "motion_infer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"     // DetectSensor() 里用 vTaskDelay 让出 CPU
#include "freertos/task.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sleep_pillow {

// ESP32-S3 ADC1 的 GPIO→通道映射（GPIO1..GPIO10 = CH0..CH9）
static adc_channel_t AdcChannelOf(int gpio) {
    if (gpio >= 1 && gpio <= 10) {
        return (adc_channel_t)(gpio - 1);
    }
    return ADC_CHANNEL_0;   // 非法值回落到默认
}

BcgAdc* BcgAdc::GetInstance() {
    static BcgAdc inst;
    return &inst;
}

void BcgAdc::AllocateRingIfNeeded() {
    if (ring_) return;
    ring_ = (float*)heap_caps_malloc(kRingCap * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ring_) {
        ring_ = (float*)heap_caps_malloc(kRingCap * sizeof(float), MALLOC_CAP_8BIT);
    }
    if (!ring_) {
        ESP_LOGE(TAG, "ring buffer alloc failed (%d B)", (int)(kRingCap * sizeof(float)));
    } else {
        memset(ring_, 0, kRingCap * sizeof(float));
        // 注：IDF 5.5 已移除 esp_ptr_internal()；这里只打印容量,不再区分落点
        ESP_LOGI(TAG, "ring buffer allocated, %d samples (%.1fs @ %d Hz)",
                 kRingCap, (float)kRingCap / kBcgSampleRateHz, (int)kBcgSampleRateHz);
    }
}

esp_err_t BcgAdc::Init(EdgeMode mode, int gpio_num) {
    mode_ = mode;
    gpio_ = gpio_num;
    AllocateRingIfNeeded();

    if (mode_ == EdgeMode::kReal) {
        esp_err_t err = InitAdc();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "REAL 模式：ADC 初始化失败=%s，BCG 任务不启动",
                     esp_err_to_name(err));
            adc_inited_ = false;
            // ★★★ REAL 模式绝不回退到 SIM：麦克风/压电陶瓷故障不偷偷换合成数据
        } else {
            adc_inited_ = true;
            ESP_LOGI(TAG, "REAL 模式：压电陶瓷 GPIO%02d → ADC1_CH%d，140Hz 采集就绪",
                     gpio_, (int)chan_);
            // ★ 立刻做一次接入检测，把「硬件到底接没接」一次说清楚
            const BcgHealth h = DetectSensor();
            ESP_LOGW(TAG, "压电陶瓷接入检测: %s", BcgHealthText(h));
        }
    } else if (mode_ == EdgeMode::kSim) {
        ESP_LOGI(TAG, "SIM 模式：合成 140Hz BCG 信号（心率 60 / 呼吸 15 / 体动 0.2Hz）");
        adc_inited_ = false;
    } else {
        ESP_LOGI(TAG, "OFF 模式：BCG 关闭");
        adc_inited_ = false;
    }
    return ESP_OK;
}

const char* BcgHealthText(BcgHealth h) {
    switch (h) {
        case BcgHealth::kOk:
            return "正常（检测到明显波动，压电陶瓷受载）";
        case BcgHealth::kIdle:
            return "已接入，当前无受力（床上没人时属正常待机）";
        case BcgHealth::kAdcFail:
            return "ADC 失效（初始化失败或读不到样本）";
        case BcgHealth::kFlat:
            return "读数恒定死住 → 压电陶瓷未接入 / 对地短路";
        case BcgHealth::kNoBias:
            return "平均电平贴 0 或满幅 → 缺 1.65V 偏置电路，或接错引脚";
    }
    return "未知";
}

// ★ 压电陶瓷接入检测：实测 0.5 秒 ADC 波形，按「平均电平 + 峰峰值」分类。
//   这一步不做任何滤波，纯粹判断硬件状态。
//
//   判定顺序很关键：先排「电气上不可能」的情况（没偏置、死值），
//   再看波动幅度。没人躺时幅度小是正常的 → kIdle，不报故障。
BcgHealth BcgAdc::DetectSensor() {
    if (mode_ == EdgeMode::kSim) return BcgHealth::kOk;   // 模拟模式不做硬件检测
    if (mode_ == EdgeMode::kOff) return BcgHealth::kAdcFail;
    if (!adc_inited_ || !adc_handle_) return BcgHealth::kAdcFail;

    const int n = kBcgSampleRateHz / 2;   // 0.5s
    int mn = 4095, mx = 0;
    int64_t sum = 0;
    int ok_cnt = 0;
    for (int i = 0; i < n; ++i) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle_, chan_, &raw) == ESP_OK) {
            if (raw < mn) mn = raw;
            if (raw > mx) mx = raw;
            sum += raw;
            ++ok_cnt;
        }
        vTaskDelay(pdMS_TO_TICKS(2));   // 快速采完，不占满 140Hz 的 7ms 周期
    }
    if (ok_cnt < n / 2) return BcgHealth::kAdcFail;

    const int mean = (int)(sum / ok_cnt);
    const int p2p  = mx - mn;
    ESP_LOGI(TAG, "传感器检测: %d 样本 min=%d max=%d mean=%d p2p=%d",
             ok_cnt, mn, mx, mean, p2p);

    // 1) 没有偏置：真实压电陶瓷信号应该以 1.65V(≈2048) 为中心摆动
    if (mean < 200 || mean > 3895) {
        return BcgHealth::kNoBias;
    }
    // 2) 读数完全死住：≤2 LSB 属于 ADC 自身噪声，说明输入端没有任何电位变化
    if (p2p <= 2) {
        return BcgHealth::kFlat;
    }
    // 3) 有微小波动（>2 LSB）：说明电气上确实接着东西，只是当前没受力。
    //    床上没人时就是这个状态，**不是故障**。
    if (p2p < 10) {
        return BcgHealth::kIdle;
    }
    return BcgHealth::kOk;
}

BcgAdc::~BcgAdc() {
    DoneAdc();
}

esp_err_t BcgAdc::InitAdc() {
    // -------------------------------------------------
    // ADC1 通道随 GPIO 自动映射（ESP32-S3：GPIO1→CH0, GPIO2→CH1, GPIO3→CH2 …）
    // 默认 GPIO1 = ADC1_CHANNEL_0
    // -------------------------------------------------
    chan_ = AdcChannelOf(gpio_);

    // 1) 单元初始化
    //    显式写全所有字段：只写 unit_id 时编译器会报
    //    -Wmissing-field-initializers（clk_src / ulp_mode 未初始化）
    init_cfg_ = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg_, &adc_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    // 2) 通道配置
    //    attenuation=12dB（原 11dB 已废弃，两者行为一致）→ 量程≈0~3.1V，
    //    覆盖压电陶瓷经 1.65V 偏置后的输出
    //    bit_width=12 → raw ∈ [0, 4095]，中点 2048 对应 1.65V
    chan_cfg_ = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(adc_handle_, chan_, &chan_cfg_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
        return err;
    }

    // 3) 校准（可选，提升精度；校准失败不影响采集，只是精度略低）
    //    ESP32-S3 提供 eFuse 校准值，adc_cali_efuse_calibration() 会自动读取。
    ESP_LOGI(TAG, "ADC1_CH%d 初始化完成（atten=12dB, 12bit, GPIO%02d）",
             (int)chan_, gpio_);
    return ESP_OK;
}

void BcgAdc::DoneAdc() {
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
        adc_handle_ = nullptr;
    }
}

bool BcgAdc::ReadSample(float* out) {
    if (mode_ == EdgeMode::kOff) return false;

    if (mode_ == EdgeMode::kSim) {
        // ★ SIM：合成 140Hz BCG 信号
        //   1) 心跳 60bpm：每 1 秒一个明显三角脉冲（峰值 1.5，0.12s 衰减）
        //   2) 呼吸 15bpm：0.25Hz × 振幅 0.5 的慢调制
        //   3) 体动脉冲：每 5s 一次 0.5s 大脉冲（峰值 1.0）
        const float t = (float)sim_phase_ / (float)kBcgSampleRateHz;
        const float beat_phase = fmodf(t, 1.0f);
        const float hr_pulse = (beat_phase < 0.12f)
            ? 1.5f * (1.0f - beat_phase / 0.12f)
            : 0.0f;
        const float br_env = 0.5f * sinf(2.0f * (float)M_PI * 0.25f * t);
        const float mt_burst = (fmodf(t, 5.0f) < 0.5f) ? 1.0f : 0.0f;
        *out = hr_pulse + br_env + mt_burst;
        sim_phase_++;
        return true;
    }

    // REAL：走真实 ADC（InitAdc() 已成功创建 adc_handle_）
    if (mode_ == EdgeMode::kReal) {
        if (!adc_inited_ || !adc_handle_) {
            // ★ REAL 模式绝不回退 SIM。这里每 5 秒才提示一次：
            //   本函数被 140Hz 调用，逐次打印会直接刷爆串口。
            if ((read_err_cnt_++ % (kBcgSampleRateHz * 5)) == 0) {
                ESP_LOGE(TAG, "REAL 模式但 ADC 未就绪（检查 GPIO%d 接线与 Init 日志）", gpio_);
            }
            return false;
        }
        int raw;
        esp_err_t err = adc_oneshot_read(adc_handle_, chan_, &raw);
        if (err != ESP_OK) {
            if ((read_err_cnt_++ % (kBcgSampleRateHz * 5)) == 0) {
                ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(err));
            }
            return false;
        }
        read_err_cnt_ = 0;
        // 12bit ADC：中点 2048 对应 1.65V（VCC/2，偏置电压）；
        // 压电陶瓷信号叠加在偏置上，±1.0 表示满幅 ±1.65V。
        *out = ((float)raw - 2048.0f) / 2048.0f;
        return true;
    }

    return false;
}

void BcgAdc::FeedSample(float s) {
    if (!ring_) return;
    ring_[head_] = s;
    head_ = (head_ + 1) % kRingCap;
    if (filled_ < kRingCap) filled_++;
}

void BcgAdc::TickAggregate() {
    // ★ 诊断日志：每 30s 打一次实际信号幅度，用来区分三种情况
    //   1) filled 一直很小  → ReadSample 失败（ADC 没初始化 / 读失败）
    //   2) p2p 极小、raw 恒定 → 传感器没接 / 没受力 / 偏置跑飞
    //   3) p2p 有值但被判无效 → 阈值偏高，需要下调
    if (mode_ == EdgeMode::kReal) {
        float p2p = 0.0f, std = 0.0f, mean_norm = 0.0f;
        // 不接返回值：下面的 verdict 会逐条判断原因，比一个 bool 更能说明问题
        SignalValid(&p2p, &std, &mean_norm);
        // 平均电平（raw 量纲）非常关键：
        //   · 接近 2048 → 偏置正常（1.65V 中点），信号在中间摆动
        //   · 接近 0    → 没有偏置电路，压电负半周被 ADC 削掉，几乎采集不到
        //   · 接近 4095 → 直接顶到 3.3V，接线/偏置有问题
        const int raw_p2p  = (int)(p2p * 2048.0f);
        const int raw_mean = (int)(mean_norm * 2048.0f) + 2048;

        // 明确说出「为什么无效」，避免只看 p2p 大就以为信号好
        const char* verdict;
        int vid;
        const float mf = (float)raw_mean;
        if (filled_ < kDeriveWinSamples) {
            verdict = "窗口未填满，等数据";
            vid = 0;
        } else if (mf < kMinBiasRaw || mf > kMaxBiasRaw) {
            verdict = "偏置异常 → 缺 1.65V 偏置电路或接错引脚，心率呼吸不输出";
            vid = 1;
        } else if (p2p < kMinPeakToPeak || std < kMinStd) {
            verdict = "无受力/信号弱（床上没人时属正常待机）";
            vid = 2;
        } else {
            verdict = "信号有效";
            vid = 3;
        }

        // ★ 只在「状态变化」或「满 5 分钟」时打印：
        //   之前每 10 秒打一条完全相同的内容，40 多行全是重复，反而看不到关键日志。
        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        const bool changed = (vid != last_diag_verdict_);
        const bool periodic = (now_ms - last_diag_ms_) >= 300000u;   // 5 分钟
        if (changed || periodic) {
            last_diag_verdict_ = vid;
            last_diag_ms_ = now_ms;
            ESP_LOGI(TAG, "BCG 诊断: p2p=%.5f(raw %d) mean_raw=%d std=%.5f → %s",
                     (double)p2p, raw_p2p, raw_mean, (double)std, verdict);
        }
    }
    ComputeHeartRate();
    ComputeRespRate();
    UpdateMotion();
}

// ★ 信号有效性检查：压电陶瓷没接/没受力时，ADC 只是悬空读噪声，
//   这时如果直接跑过零点/方差算法，会"数"出假心率、假呼吸、假体动。
//   判定标准（任一不满足 → 认为没有有效信号，各派生量输出 -1）：
//     1) 窗口内峰峰值 >= kMinPeakToPeak
//     2) 窗口内标准差 >= kMinStd
//   注：压电陶瓷直接接 ADC（无放大）时信号很小，阈值必须放低；
//       真正的「没接」表现为 p2p 接近 0 且 raw 恒定不动。
bool BcgAdc::SignalValid(float* out_p2p, float* out_std, float* out_mean) const {
    if (!ring_ || filled_ < kDeriveWinSamples) {
        if (out_p2p) *out_p2p = 0.0f;
        if (out_std) *out_std = 0.0f;
        if (out_mean) *out_mean = 0.0f;
        return false;
    }
    float mn = 1e9f, mx = -1e9f, sum = 0.0f, sum2 = 0.0f;
    for (int i = 0; i < kDeriveWinSamples; ++i) {
        const int idx = (head_ - kDeriveWinSamples + i + kRingCap) % kRingCap;
        const float v = ring_[idx];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum  += v;
        sum2 += v * v;
    }
    const float n    = (float)kDeriveWinSamples;
    const float mean = sum / n;
    float var = sum2 / n - mean * mean;
    if (var < 0.0f) var = 0.0f;
    const float std  = sqrtf(var);
    const float p2p  = mx - mn;

    if (out_p2p) *out_p2p = p2p;
    if (out_std) *out_std = std;
    if (out_mean) *out_mean = mean;

    // ★ 偏置检查必须在幅度检查之前：
    //   压电陶瓷是交流信号，必须叠在 1.65V（raw≈2048）上摆动。
    //   实测 mean_raw=102（贴地）说明没有偏置电路，负半周被 ADC 削掉，
    //   这种波形看着 p2p 很大，但完全不能用来算心率/呼吸。
    const float mean_raw = mean * 2048.0f + 2048.0f;
    if (mean_raw < kMinBiasRaw || mean_raw > kMaxBiasRaw) {
        return false;
    }

    return (p2p >= kMinPeakToPeak) && (std >= kMinStd);
}

void BcgAdc::ComputeHeartRate() {
    if (filled_ < kDeriveWinSamples) { derived_.heart_rate = -1; return; }

    // ★ 先验信号有效性：没接压电陶瓷 / 没受力 → 直接判无心率
    float p2p = 0.0f, std = 0.0f;
    if (!SignalValid(&p2p, &std)) {
        derived_.heart_rate = -1;
        if (mode_ == EdgeMode::kReal && !warned_no_signal_) {
            warned_no_signal_ = true;
            ESP_LOGW(TAG, "REAL 模式无有效 BCG 信号 (p2p=%.3f std=%.3f)，"
                          "心率/呼吸输出 -1。检查压电陶瓷接线与受力。",
                     (double)p2p, (double)std);
        }
        return;
    }
    warned_no_signal_ = false;

    // 简化：自相关求最大峰（仅取最近的 60s 窗）
    // 这里用近似算法：j_peak 数 / 60 = bpm
    // TODO: 与 BCG-classify 对齐的自相关（FFT）实现
    int peaks = 0;
    int last_peak = -100;
    const int min_distance = (int)(kBcgSampleRateHz * 0.4f);  // ≤ 150 bpm 不应期
    for (int i = 1; i < kDeriveWinSamples; ++i) {
        int idx = (head_ - kDeriveWinSamples + i + kRingCap) % kRingCap;
        int idx_prev = (head_ - kDeriveWinSamples + i - 1 + kRingCap) % kRingCap;
        if (ring_[idx] > 0.3f && ring_[idx_prev] <= 0.3f) {
            if (i - last_peak >= min_distance) {
                peaks++;
                last_peak = i;
            }
        }
    }
    int bpm = (int)((float)peaks * 60.0f / (float)kDeriveWinSec);
    if (bpm < 30 || bpm > 200) bpm = -1;
    derived_.heart_rate = (int8_t)bpm;
}

void BcgAdc::ComputeRespRate() {
    if (filled_ < kDeriveWinSamples) { derived_.resp_rate = -1; return; }

    // ★ 先验信号有效性：无有效信号时绝不输出占位值
    float p2p = 0.0f, std = 0.0f, mean = 0.0f;
    if (!SignalValid(&p2p, &std, &mean)) {
        derived_.resp_rate = -1;
        return;
    }

    // ★ 真实呼吸率估计（不再返回占位 15）：
    //   1) 30s 窗按 0.5s 分块取均值 → 消掉心跳(≈1Hz)与高频噪声，
    //      保留呼吸(0.1~0.5Hz)慢波，得 60 个点
    //   2) 数慢波上穿自身均值的次数 → 30s 内的呼吸周期数
    //   3) ×2 得每分钟次数
    constexpr int kBlocks = 60;                       // 30s / 0.5s
    const int block_len = kDeriveWinSamples / kBlocks; // =70 样本
    if (block_len <= 0) { derived_.resp_rate = -1; return; }

    float slow[kBlocks];
    for (int b = 0; b < kBlocks; ++b) {
        float acc = 0.0f;
        const int base = b * block_len;
        for (int k = 0; k < block_len; ++k) {
            const int idx = (head_ - kDeriveWinSamples + base + k + kRingCap) % kRingCap;
            acc += ring_[idx];
        }
        slow[b] = acc / (float)block_len;
    }

    float smean = 0.0f;
    for (int b = 0; b < kBlocks; ++b) smean += slow[b];
    smean /= (float)kBlocks;

    int crossings = 0;
    int prev = -1;
    for (int b = 0; b < kBlocks; ++b) {
        const int cur = (slow[b] > smean) ? 1 : 0;
        if (prev == 0 && cur == 1) ++crossings;
        prev = cur;
    }

    int rpm = crossings * 2;   // 30s → 每分钟
    // 成人静息呼吸 8~24 次/分；放宽到 4~40 容忍异常，超出视为噪声
    if (rpm < 4 || rpm > 40) rpm = -1;
    derived_.resp_rate = (int8_t)rpm;
}

void BcgAdc::UpdateMotion() {
    if (!ring_ || filled_ < kBcgSampleRateHz) { derived_.motion = false; return; }

    // ★ 无有效信号时不出体动（否则 ADC 噪声会被当成翻身）
    float p2p = 0.0f, std = 0.0f;
    if (!SignalValid(&p2p, &std)) { derived_.motion = false; return; }

    float sum = 0, sum2 = 0;
    for (int i = 0; i < kBcgSampleRateHz; ++i) {
        int idx = (head_ - 1 - i + kRingCap) % kRingCap;
        float v = ring_[idx];
        sum += v; sum2 += v * v;
    }
    float mean = sum / kBcgSampleRateHz;
    float var  = sum2 / kBcgSampleRateHz - mean * mean;
    float sd   = (var > 0) ? sqrtf(var) : 0;
    derived_.motion = (sd > 0.4f);
}

float BcgAdc::SelfTest() {
    if (mode_ == EdgeMode::kOff) return -1.0f;
    Init(mode_, gpio_);
    sim_phase_ = 0;
    for (int i = 0; i < kDeriveWinSamples; ++i) {
        float s;
        ReadSample(&s);
        FeedSample(s);
    }
    TickAggregate();
    ESP_LOGI(TAG, "SelfTest: hr=%d rr=%d motion=%d",
             (int)derived_.heart_rate, (int)derived_.resp_rate, (int)derived_.motion);
    return (derived_.heart_rate > 0) ? 0.0f : -1.0f;
}

}  // namespace sleep_pillow
