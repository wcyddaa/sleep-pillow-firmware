// inmp441_mic.cc - 双 INMP441 立体声 I2S 采集 → 单声道近场输出
#include "inmp441_mic.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "board.h"

namespace sleep_pillow {

Inmp441Mic* Inmp441Mic::GetInstance() {
    static Inmp441Mic inst;
    return &inst;
}

esp_err_t Inmp441Mic::Init(const Inmp441MicConfig& cfg) {
    if (ready_) return ESP_OK;
    cfg_ = cfg;
    if (cfg_.sample_rate <= 0)       cfg_.sample_rate = 16000;
    if (cfg_.max_mono_samples <= 0)  cfg_.max_mono_samples = 16000;

    // ---- 交错立体声 scratch（PSRAM 优先，回退内部 SRAM）----
    // ★ 用 int32_t：INMP441 是 24bit 器件，32bit 槽才能装下完整数据。
    //   16bit 槽（ws_width=16）装不下 1BCLK 延迟 + 24bit 数据，
    //   会导致左右声道混叠、位对齐错乱 → 读出来是乱跳的垃圾值。
    scratch_cap_ = (size_t)cfg_.max_mono_samples;
    const size_t scratch_bytes = scratch_cap_ * 2 * sizeof(int32_t);
    stereo_scratch_ = (int32_t*)heap_caps_malloc(
        scratch_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (stereo_scratch_ == nullptr) {
        stereo_scratch_ = (int32_t*)heap_caps_malloc(scratch_bytes, MALLOC_CAP_8BIT);
    }
    if (stereo_scratch_ == nullptr) {
        ESP_LOGE(TAG, "立体声 scratch 分配失败 (%u B)", (unsigned)scratch_bytes);
        return ESP_ERR_NO_MEM;
    }

    // ---- 向板型索取 I2S 采集实现并打开 ----
    // 通道建立、DMA、引脚、槽宽与对齐修正全部封装在板级抽象层，
    // 本组件只面对 BoardMicInput 统一接口。
    auto* board = Board::GetInstance();
    if (board == nullptr) {
        ESP_LOGE(TAG, "Board 未注册，无法获取麦克风采集接口");
        heap_caps_free(stereo_scratch_);
        stereo_scratch_ = nullptr;
        return ESP_ERR_INVALID_STATE;
    }
    input_ = board->GetMicInput();
    if (input_ == nullptr) {
        ESP_LOGE(TAG, "板型 %s 未提供麦克风采集接口", board->GetBoardName());
        heap_caps_free(stereo_scratch_);
        stereo_scratch_ = nullptr;
        return ESP_ERR_NOT_SUPPORTED;
    }
    esp_err_t err = input_->Open(cfg_.sample_rate);
    if (err != ESP_OK) {
        input_ = nullptr;
        heap_caps_free(stereo_scratch_);
        stereo_scratch_ = nullptr;
        return err;
    }

    ready_ = true;
    cur_pick_ = 0;
    short_frames_ = 0;
    ESP_LOGI(TAG, "初始化完成: %dHz 单声道契约, scratch=%uB（引脚/时序见板型日志）",
             cfg_.sample_rate, (unsigned)scratch_bytes);
    return ESP_OK;
}

esp_err_t Inmp441Mic::Deinit() {
    if (input_ != nullptr) {
        input_->Close();
        input_ = nullptr;
    }
    if (stereo_scratch_ != nullptr) {
        heap_caps_free(stereo_scratch_);
        stereo_scratch_ = nullptr;
    }
    scratch_cap_ = 0;
    ready_ = false;
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// 近场选麦 + 接近时平均
// ---------------------------------------------------------------------------
// ★ 32bit 槽 → 16bit 有效数据
//   INMP441 的 24bit 数据在 32bit 帧里左对齐（占 bit[31:8]），
//   右移 16 位即取高 16 位（保留 16bit 精度，等价于原 16bit 槽的价值）。
static inline int16_t RawToInt16(int32_t raw) {
    int32_t v = raw >> 16;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

void Inmp441Mic::MixToMono(const int32_t* stereo, size_t frames, int16_t* mono) {
    // ★ 抓 8 个原始采样存档，供日志判断波形质量（位错位 vs 真实音频）
    for (size_t k = 0; k < 8 && k < frames; ++k) {
        last_raw_head_[k] = RawToInt16(stereo[2 * k]);
    }

    // 1) 两路统计量：均值(DC) / RMS / 峰值
    double sum_l = 0.0, sum_r = 0.0;
    double acc_l = 0.0, acc_r = 0.0;
    int32_t pk_l = 0, pk_r = 0;
    for (size_t i = 0; i < frames; ++i) {
        const int32_t l = RawToInt16(stereo[2 * i]);
        const int32_t r = RawToInt16(stereo[2 * i + 1]);
        sum_l += l;
        sum_r += r;
        acc_l += (double)l * (double)l;
        acc_r += (double)r * (double)r;
        const int32_t al = (l < 0) ? -l : l;
        const int32_t ar = (r < 0) ? -r : r;
        if (al > pk_l) pk_l = al;
        if (ar > pk_r) pk_r = ar;
    }
    const double inv = (frames > 0) ? (1.0 / (double)frames) : 0.0;
    last_dc_l_ = (float)(sum_l * inv);
    last_dc_r_ = (float)(sum_r * inv);
    const float rms_l = (frames > 0) ? (float)sqrt(acc_l * inv) : 0.0f;
    const float rms_r = (frames > 0) ? (float)sqrt(acc_r * inv) : 0.0f;
    last_rms_l_ = rms_l;
    last_rms_r_ = rms_r;
    last_peak_l_ = (int)pk_l;
    last_peak_r_ = (int)pk_r;

    const float hi = (rms_l >= rms_r) ? rms_l : rms_r;
    const float lo = (rms_l >= rms_r) ? rms_r : rms_l;

    // ★★ 逐声道质量评分（最重要的修复）：
    //   双麦并联在一条 I2S 总线上，只要有一路麦克风**没接 VDD/GND**，
    //   它的 SD 输出就是浮空干扰或串过来的另一路数据。
    //   原逻辑默认「两路平均」，等于把一半废数据混进模型输入 → 特征全废。
    //
    //   评分：2=干净可用 / 1=能用但削顶 / 0=不可用（没数据）
    //   注：pk_l / pk_r 是本函数开头统计出来的本帧峰值，直接复用
    const bool l_dead = (rms_l < kDeadRms);
    const bool r_dead = (rms_r < kDeadRms);
    const bool l_clip = (pk_l >= kClipPeak);
    const bool r_clip = (pk_r >= kClipPeak);
    const int score_l = l_dead ? 0 : (l_clip ? 1 : 2);
    const int score_r = r_dead ? 0 : (r_clip ? 1 : 2);
    last_score_l_ = score_l;
    last_score_r_ = score_r;

    int pick;
    if (score_l == 0 && score_r == 0) {
        // 两路都没数据 → 输出静音，绝不用垃圾数据喂模型
        pick = 3;
    } else if (score_l > score_r) {
        pick = 0;               // 左路更好
    } else if (score_r > score_l) {
        pick = 1;               // 右路更好
    } else if (!cfg_.near_field_select) {
        // 两路同等质量：默认「双麦降噪」= 两路平均（与训练端 mean(axis=1) 一致）
        pick = 2;
    } else if (hi < kSilenceRms) {
        // 近乎静音：没有"近场"可言，直接平均（不改变滞回状态）
        pick = 2;
    } else if (lo > 0.0f && (hi / lo) <= kAvgRatio) {
        // 两路接近：平均降噪
        pick = 2;
    } else {
        // 一路明显更响 → 判为更靠近声源；带滞回避免边界帧来回翻转
        if (cur_pick_ == 0) {
            pick = (rms_r > rms_l * kSwitchRatio) ? 1 : 0;
        } else {
            pick = (rms_l > rms_r * kSwitchRatio) ? 0 : 1;
        }
        cur_pick_ = pick;
    }
    last_pick_ = pick;

    // 2) 合成单声道
    // ★ 近场降噪增益（只保留 1.5~2 米内的声音），整帧统一算一次增益：
    //   - RMS < kNoiseFloorRms：背景噪声 / 远场 → 抑制为 0
    //   - RMS 在 kNoiseFloorRms~kRefRms 之间：线性过渡（越远衰减越多）
    //   - RMS >= kRefRms：全增益（近场）
    // 阈值标定：kNoiseFloorRms(250) ≈ 安静室内底噪上限；
    //           kRefRms(1500) ≈ 正常说话 30~50cm 距离。
    const float rms_total = (float)sqrt((acc_l + acc_r) * inv);
    float near_gain = 1.0f;
    if (pick == 3) {
        near_gain = 0.0f;                       // 两路都没数据 → 静音
    } else if (!kEnableNearFieldGain) {
        // ★ 默认关闭：保持原始幅度，与训练端分布一致
        near_gain = 1.0f;
    } else if (rms_total <= 0.0f) {
        near_gain = 0.0f;
    } else if (rms_total < kNoiseFloorRms) {
        near_gain = 0.0f;                       // 远场/底噪：完全抑制
    } else if (rms_total < kRefRms) {
        near_gain = (rms_total - kNoiseFloorRms) / (kRefRms - kNoiseFloorRms);
        if (near_gain < 0.15f) near_gain = 0.0f; // 极微弱也清零
    }
    last_near_gain_ = near_gain;

    for (size_t i = 0; i < frames; ++i) {
        const int32_t l = RawToInt16(stereo[2 * i]);
        const int32_t r = RawToInt16(stereo[2 * i + 1]);
        int32_t v;
        if (pick == 0)      v = l;
        else if (pick == 1) v = r;
        else if (pick == 3) v = 0;      // 两路都不可用 → 静音
        else                v = (l + r) / 2;

        // ★ 输入衰减（压制削顶）：kInputAtten=1.0 时等价于不变
        if (kInputAtten < 1.0f) {
            v = (int32_t)((float)v * kInputAtten);
        }

        if (near_gain < 1.0f) {
            v = (int32_t)((float)v * near_gain);
        }

        if (v > 32767)  v = 32767;
        if (v < -32768) v = -32768;
        mono[i] = (int16_t)v;
    }

    // ★ 去直流（DC 偏置）：
    //   INMP441 输出常带明显 DC 偏置（实测 L 声道可达 5000+，占满幅 17%）。
    //   不去掉的话会污染 FFT 的直流分量 → log-mel / MFCC 整体偏移 → 模型误判。
    //   训练端音频来自 WAV（录音链路本身已去 DC），所以这里去 DC 反而更贴近
    //   「训练时的输入分布」，不是额外引入的偏差。
    if (frames > 0) {
        int64_t sum = 0;
        for (size_t i = 0; i < frames; ++i) sum += mono[i];
        const int32_t dc = (int32_t)(sum / (int64_t)frames);
        if (dc != 0) {
            for (size_t i = 0; i < frames; ++i) {
                int32_t v = (int32_t)mono[i] - dc;
                if (v > 32767)  v = 32767;
                if (v < -32768) v = -32768;
                mono[i] = (int16_t)v;
            }
        }
        last_dc_removed_ = dc;
    }
}

// ---------------------------------------------------------------------------
// 读单声道帧
// ---------------------------------------------------------------------------
size_t Inmp441Mic::ReadMono(int16_t* dst, size_t mono_samples, uint32_t timeout_ms) {
    if (!ready_ || dst == nullptr || mono_samples == 0) return 0;
    if (mono_samples > scratch_cap_) {
        ESP_LOGW(TAG, "请求 %u 帧 > 预留容量 %u 帧",
                 (unsigned)mono_samples, (unsigned)scratch_cap_);
        return 0;
    }

    // 产出 N 个单声道样本 == 读 N 个立体声帧（每帧 L+R 两个 int32）
    size_t got_bytes = 0;
    const esp_err_t err = input_->ReadStereo(stereo_scratch_, mono_samples,
                                             &got_bytes, timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ReadStereo: %s (%u/%u B)",
                 esp_err_to_name(err), (unsigned)got_bytes,
                 (unsigned)(mono_samples * 2 * sizeof(int32_t)));
    }

    const size_t got_frames = got_bytes / (2 * sizeof(int32_t));
    if (got_frames < mono_samples) short_frames_ += (mono_samples - got_frames);
    if (got_frames == 0) return 0;

    MixToMono(stereo_scratch_, got_frames, dst);
    return got_frames;
}

// ---------------------------------------------------------------------------
// 自检：判断麦克风是否真的在工作
// ---------------------------------------------------------------------------
const char* MicHealthText(MicHealth h) {
    switch (h) {
        case MicHealth::kOk:       return "正常";
        case MicHealth::kNoData:   return "无数据(检查 SD/DIN、3V3 供电、GND 共地)";
        case MicHealth::kTooQuiet: return "几乎静音(麦克风未焊好 / 增益过低 / 没对着声源)";
        case MicHealth::kDcOffset: return "DC 偏置过大(建议固件去直流或硬件加耦合电容)";
        case MicHealth::kClipping: return "削顶失真(声压过大 / 增益过高)";
    }
    return "未知";
}

MicHealth Inmp441Mic::SelfTest(float* rms_out, float* dc_out, int* peak_out) {
    if (!ready_) return MicHealth::kNoData;

    const size_t nsamp = (size_t)(cfg_.sample_rate / 2);   // 0.5 秒
    if (nsamp == 0 || nsamp > scratch_cap_) return MicHealth::kNoData;

    int16_t* buf = (int16_t*)heap_caps_malloc(
        nsamp * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == nullptr) {
        buf = (int16_t*)heap_caps_malloc(nsamp * sizeof(int16_t), MALLOC_CAP_8BIT);
    }
    if (buf == nullptr) return MicHealth::kNoData;

    const size_t got = ReadMono(buf, nsamp, 2000);
    heap_caps_free(buf);

    if (got < nsamp) {
        ESP_LOGW(TAG, "自检: %s (只读到 %u/%u 帧)",
                 MicHealthText(MicHealth::kNoData),
                 (unsigned)got, (unsigned)nsamp);
        return MicHealth::kNoData;
    }

    const float rms = (last_rms_l_ > last_rms_r_) ? last_rms_l_ : last_rms_r_;
    const int   pk  = (last_peak_l_ > last_peak_r_) ? last_peak_l_ : last_peak_r_;
    const float dc  = (fabsf(last_dc_l_) > fabsf(last_dc_r_)) ? last_dc_l_ : last_dc_r_;
    if (rms_out)  *rms_out = rms;
    if (dc_out)   *dc_out = dc;
    if (peak_out) *peak_out = pk;

    MicHealth verdict = MicHealth::kOk;
    if (pk == 0) {
        verdict = MicHealth::kNoData;          // 采样全 0：没数据
    } else if (rms < 1.0f) {
        verdict = MicHealth::kTooQuiet;
    } else if (pk >= 32700) {
        verdict = MicHealth::kClipping;
    } else if (fabsf(dc) > 2000.0f) {
        verdict = MicHealth::kDcOffset;
    }

    ESP_LOGI(TAG, "自检: %s  (rms=%.0f peak=%d dc=%.0f, 合成=%s)",
             MicHealthText(verdict), (double)rms, pk, (double)dc,
             cfg_.near_field_select ? "近场选麦" : "双麦平均降噪");
    return verdict;
}

}  // namespace sleep_pillow
