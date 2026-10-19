// app_main.cpp - v1.0+: 三端侧模型 + 三开关 + 30s 结构化数据
// ============================================================
// 数据流:
//   三个 task(鼾声 / 分期 / BCG 体动)各自按运行时 cfg 决定数据源
//     OFF : 不创建 task
//     SIM : 假数据走真实模型推理(★ "模拟思想" = 假数据进模型)
//     REAL: 真实传感器(BCG 压电陶瓷 / 麦克风)走真实模型
//   EdgeAggregator 30s 累积一帧 → CloudClient::PublishFrame
//   主题 /device/{id}/frame；cfg 变更走 BLE 0x05 SET_CFG + MQTT /cfg
//   删除 v1.0 旧 SimulateXxx / FlushWindow / s_sampling 路径
// ============================================================
#include "app_main.h"

#include <sys/time.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <string>

#include "app_config.h"
#include "bcg_adc.h"
#include "ble_provision.h"
#include "board.h"
#include "cloud_client.h"
#include "edge_aggregator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "model_nets.h"
#include "nvs_flash.h"
#include "motion_infer.h"
#include "inmp441_mic.h"
#include "snore_infer.h"
#include "stage_infer.h"
#include "wifi.h"

static constexpr const char* TAG = "AppMain";

// Wi-Fi 管理器（BLE 配网回调里会用它重连路由器）
static WifiManager s_wifi;

// ★ 端侧三开关 NVS 配置
static sleep_pillow::AppConfigStore* g_cfg = nullptr;

// ★ 三个 task 句柄（按 cfg 启停）
static TaskHandle_t s_snore_task  = nullptr;
static TaskHandle_t s_stage_task  = nullptr;
static TaskHandle_t s_motion_task = nullptr;

// ★ BCG 实例。
//   注意：BCG 的 SIM 模式**只**在用户通过 BLE 三开关主动选 SIM 时生效；
//   REAL 模式下 ADC 失败只报错并输出 -1，绝不偷偷换成合成数据。
static sleep_pillow::BcgAdc* g_bcg = nullptr;

// ★ 3 个模型的 cfg 状态：task 内部循环检查
static inline int CurrentSnoreMode() { return g_cfg ? (int)g_cfg->Get().snore_mode : 0; }
static inline int CurrentStageMode() { return g_cfg ? (int)g_cfg->Get().stage_mode : 0; }
static inline int CurrentBcgMode()   { return g_cfg ? (int)g_cfg->Get().bcg_mode   : 0; }

// ============================================================
// 音频调试开关
//   true  → REAL 模式第 10 秒把 1 秒 PCM 以 base64 打到串口
//           （16000 样本 ≈ 561 行日志，会刷满串口并占用 CPU）
//   false → 完全不输出（默认）
// 只有需要用 tools/decode_audio_dump.py 还原波形时才临时改 true。
// ============================================================
static constexpr bool kEnableAudioDump = false;

// ============================================================
// 音频调试：把 1 秒单声道 PCM 以 base64 打到串口，
// 配合 tools/decode_audio_dump.py 还原成 .wav 直接试听。
// （自包含 base64，避免给 main 增加 mbedtls 依赖）
//
// 输出格式：
//   ---AUDIO-DUMP-BEGIN--- <tag> <samples> samples 16000Hz 16bit mono
//   <每行 76 字符的 base64 块>
//   ---AUDIO-DUMP-END---
// ============================================================
[[maybe_unused]] static void DumpPcmBase64(const char* tag, const int16_t* pcm, size_t samples) {
    static const char kB64[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const uint8_t* b = reinterpret_cast<const uint8_t*>(pcm);
    const size_t n = samples * sizeof(int16_t);
    char line[80];
    size_t li = 0;
    size_t lines = 0;   // ★ 已打出的行数，用于周期性让出 CPU
    ESP_LOGI(TAG, "---AUDIO-DUMP-BEGIN--- %s %u samples %dHz 16bit mono",
             tag, (unsigned)samples, (int)sleep_pillow::kSnoreSampleRate);
    for (size_t i = 0; i < n; i += 3) {
        const size_t rem = n - i;
        uint32_t v = (uint32_t)b[i] << 16;
        if (rem > 1) v |= (uint32_t)b[i + 1] << 8;
        if (rem > 2) v |= (uint32_t)b[i + 2];
        line[li++] = kB64[(v >> 18) & 0x3F];
        line[li++] = kB64[(v >> 12) & 0x3F];
        line[li++] = (rem > 1) ? kB64[(v >> 6) & 0x3F] : '=';
        line[li++] = (rem > 2) ? kB64[v & 0x3F] : '=';
        if (li >= 76) {
            line[li] = '\0';
            ESP_LOGI(TAG, "%s", line);
            li = 0;
            // ★★ 关键：16000 样本 ≈ 561 行日志，115200 波特率下串口是阻塞写，
            //    全程约 3~4 秒。这期间本任务一直不阻塞、不让出 CPU，
            //    IDLE0 得不到调度 → task_wdt 报 "CPU 0: audio_snore"。
            //    每 8 行让出一次（~55ms），保证看门狗被喂。
            if ((++lines & 0x07) == 0) {
                vTaskDelay(1);
            }
        }
    }
    if (li > 0) {
        line[li] = '\0';
        ESP_LOGI(TAG, "%s", line);
    }
    ESP_LOGI(TAG, "---AUDIO-DUMP-END---");
    // dump 结束后再让一次，把这段占用的时间彻底还回去
    vTaskDelay(1);
}

// ============================================================
// 鼾声任务：每秒取 1 秒音频 → snore_infer 推理
// ============================================================
static void AudioSnoreTaskEntry(void* arg) {
    (void)arg;
    using sleep_pillow::SnoreInfer;
    auto* infer = SnoreInfer::GetInstance();
    if (infer->Init() != ESP_OK) {
        ESP_LOGE(TAG, "[snore] 推理初始化失败，任务退出");
        vTaskDelete(nullptr);
        return;
    }

    const size_t frame_samples = sleep_pillow::kSnoreFrameSamples;   // 16000 = 1s @16k
    // 32KB 帧缓冲放 PSRAM，别占内部 SRAM
    int16_t* frame = static_cast<int16_t*>(
        heap_caps_malloc(frame_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (frame == nullptr) {
        frame = static_cast<int16_t*>(
            heap_caps_malloc(frame_samples * sizeof(int16_t), MALLOC_CAP_8BIT));
    }
    if (frame == nullptr) {
        ESP_LOGE(TAG, "[snore] 帧缓冲分配失败");
        infer->Deinit();
        vTaskDelete(nullptr);
        return;
    }

    auto* mic = sleep_pillow::Inmp441Mic::GetInstance();
    bool mic_ready = false;
    bool self_tested = false;   // 开机第 3 秒做一次健康判定
    uint64_t sim_phase = 0;
    // REAL 音频调试用
    uint32_t mic_sec     = 0;      // REAL 模式已采集的秒数
    bool     dumped_once = false;  // 是否已 dump 过一段音频
    int      last_hit    = -1;     // 上一次鼾声判定（-1=未初始化）

    while (true) {
        const int mode = CurrentSnoreMode();
        if (mode == 0) {
            ESP_LOGW(TAG, "[snore] cfg 切回 OFF，任务退出");
            if (mic_ready) { mic->Deinit(); mic_ready = false; }
            infer->Deinit();
            s_snore_task = nullptr;
            heap_caps_free(frame);
            vTaskDelete(nullptr);
            return;
        }

        bool have_audio = false;

        if (mode == 2) {
            // ---------- REAL：双 INMP441 I2S 真实音频 → 近场单声道 ----------
            if (!mic_ready) {
                if (mic->Init() != ESP_OK) {
                    ESP_LOGE(TAG, "[snore] INMP441 初始化失败，2s 后重试");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                mic_ready = true;
                ESP_LOGI(TAG, "[snore] REAL 模式已接入 INMP441 (BCLK=GPIO5 WS=GPIO4 DIN=GPIO6)");
                // ★ 自检延后到开机第 3 秒（见下方 mic_sec == 3）：
                //   刚 Init 时抓到的是上电瞬态，会误报「削顶失真」。
            }
            // 1s 音频正常远快于 1.6s 超时；超时说明 I2S 没有数据
            const size_t got = mic->ReadMono(frame, frame_samples, 1600);
            if (got == frame_samples) {
                have_audio = true;
                ++mic_sec;
                const int pk = mic->LastPick();

                // ★ 开机第 3 秒做一次麦克风健康判定（避开上电瞬态）。
                //   直接用本帧已统计好的 rms/pk，不额外读音频、不丢数据。
                if (!self_tested && mic_sec == 3) {
                    self_tested = true;
                    const float rms_l = mic->LastRmsLeft();
                    const float rms_r = mic->LastRmsRight();
                    const float rms_max = (rms_l > rms_r) ? rms_l : rms_r;
                    const int pkl = mic->LastPeakLeft();
                    const int pkr = mic->LastPeakRight();
                    const int pk_max = (pkl > pkr) ? pkl : pkr;
                    const int pct = (int)(rms_max / 32768.0f * 100.0f);
                    if (pk_max == 0) {
                        ESP_LOGE(TAG, "[snore] 麦克风自检: 无数据（检查 DIN/3V3/GND）");
                    } else if (rms_max < 1.0f) {
                        ESP_LOGW(TAG, "[snore] 麦克风自检: 几乎静音（rms=%.0f）", (double)rms_max);
                    } else if (pk_max >= 32700) {
                        ESP_LOGE(TAG, "[snore] 麦克风自检: 削顶失真 (rms=%.0f 占满幅 %d%%, pk=%d)",
                                 (double)rms_max, pct, pk_max);
                        ESP_LOGE(TAG, "  → ① 麦克风挪到离人 30~50cm；"
                                      "② 或把 inmp441_mic.h 的 kInputAtten 调到 0.5/0.25");
                    } else {
                        ESP_LOGI(TAG, "[snore] 麦克风自检: 正常 (rms=%.0f 占满幅 %d%%, pk=%d)",
                                 (double)rms_max, pct, pk_max);
                    }
                }
                // ★ 每 5 秒打一次音频电平统计：
                //   dc  = 原始均值（INMP441 常有 DC 偏置；偏大会污染 FFT 直流分量 → 特征偏移 → 误判）
                //   rms = 有效值（判断到底有没有收到声音）
                //   pk  = 峰值（判断是否削顶 / 是否完全静音）
                if ((mic_sec % 5) == 0) {
                    const char* pick_txt =
                        (pk == 0) ? "左" : (pk == 1) ? "右" : (pk == 2) ? "平均" : "静音(两路都不可用)";
                    // 占满幅百分比：>50% 就已经很响，接近 100% 必然削顶
                    const int lp = (int)(mic->LastRmsLeft()  / 32768.0f * 100.0f);
                    const int rp = (int)(mic->LastRmsRight() / 32768.0f * 100.0f);
                    ESP_LOGI(TAG,
                             "[snore] L dc=%.0f rms=%.0f(%d%%) pk=%d(q%d) | R dc=%.0f rms=%.0f(%d%%) pk=%d(q%d) | 选麦=%s | 近场增益=%.2f | 去DC=%d",
                             (double)mic->LastDcLeft(), (double)mic->LastRmsLeft(), lp, mic->LastPeakLeft(),
                             mic->LastScoreLeft(),
                             (double)mic->LastDcRight(), (double)mic->LastRmsRight(), rp, mic->LastPeakRight(),
                             mic->LastScoreRight(),
                             pick_txt,
                             (double)mic->LastNearGain(), mic->LastDcRemoved());

                    // ★ 原始波形抽样：判断"位错位"还是"真实音频"
                    //   连续平滑（如 120 190 260 310）→ 真实音频
                    //   大幅跳变（如 0 32767 -32768 5）→ 错位或纯噪声
                    const int16_t* h = mic->LastRawHead();
                    ESP_LOGI(TAG, "[snore] 原始采样(L): %d %d %d %d %d %d %d %d",
                             h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
                }
                // ★ REAL 模式第 10 秒 dump 一段原始音频（默认关闭，见 kEnableAudioDump）
                if (kEnableAudioDump && !dumped_once && mic_sec == 10) {
                    dumped_once = true;
                    DumpPcmBase64("REAL10s", frame, frame_samples);
                }
            } else {
                ESP_LOGW(TAG, "[snore] I2S 读帧不足 (%u/%u)，检查接线/3V3 供电/GPIO 配置",
                         (unsigned)got, (unsigned)frame_samples);
            }
        } else {
            // ---------- SIM：合成 1s 16kHz PCM（呼吸包络 × 180Hz 载波）----------
            for (size_t i = 0; i < frame_samples; ++i) {
                const float t = (float)(sim_phase + i) / (float)sleep_pillow::kSnoreSampleRate;
                const float env = 0.5f + 0.5f * sinf(2.0f * (float)M_PI * 1.0f * t);
                frame[i] = (int16_t)(env * sinf(2.0f * (float)M_PI * 180.0f * t) * 8000.0f);
            }
            sim_phase += frame_samples;
            have_audio = true;
        }

        if (have_audio) {
            float prob = 0.0f;
            if (infer->PredictFrame(frame, &prob) == ESP_OK) {
                sleep_pillow::EdgeAggregator::GetInstance()->OnSnoreProb(prob);
                // 判定切换时打一行，便于和上面的电平行对齐看"为什么误判"
                const int hit = (prob >= 0.5f) ? 1 : 0;
                if (hit != last_hit) {
                    last_hit = hit;
                    // ★ 只在概率 >= 0.3 时才打印，极低概率静音不再刷屏
                    if (prob >= 0.3f) {
                        ESP_LOGI(TAG, "[snore] 判定 -> %s (prob=%.2f)",
                                 hit ? "鼾声" : "静音", (double)prob);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t StartAudioSnoreTask() {
    if (s_snore_task) return ESP_OK;
    return xTaskCreate(AudioSnoreTaskEntry, "audio_snore", 12288, nullptr, 3, &s_snore_task) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

// ============================================================
// 睡眠分期任务：4Hz 维护 IHR/Resp/Motion 三路 → 30s 推理一次
// ============================================================
static void StageTaskEntry(void* arg) {
    (void)arg;
    using sleep_pillow::StageInfer;
    auto* infer = StageInfer::GetInstance();
    if (infer->Init() != ESP_OK) {
        ESP_LOGE(TAG, "[stage] 初始化失败，任务退出");
        vTaskDelete(nullptr);
        return;
    }
    static sleep_pillow::StageFeatureBuilder builder;
    uint64_t sim_tick = 0;
    TickType_t last_infer = xTaskGetTickCount();
    while (true) {
        if (CurrentStageMode() == 0) {
            ESP_LOGW(TAG, "[stage] cfg 切回 OFF，任务退出");
            infer->Deinit();
            s_stage_task = nullptr;
            vTaskDelete(nullptr);
            return;
        }
        float ihr, resp, motion;
        bool  bcg_valid = true;   // ★ REAL 模式下 BCG 是否给出有效数据
        if (CurrentStageMode() == 2 && g_bcg) {
            // REAL：取 BCG 派生。
            // ★ 绝不拿 60/15 兜底：压电陶瓷没接或没受力时，
            //   塞一个"看起来正常"的 60bpm/15rpm 会让模型输出假分期，
            //   用户会看到明明没躺下却有睡眠阶段。这里改为整轮跳过推理。
            const auto d = g_bcg->GetDerived();
            bcg_valid = (d.heart_rate > 0) && (d.resp_rate > 0);
            if (!bcg_valid) {
                // 无有效 BCG → 本次不 Push 特征、不推理，30s 后重试
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }
            ihr    = (float)d.heart_rate;
            resp   = (float)d.resp_rate;
            motion = d.motion ? 1.0f : 0.0f;
        } else {
            // SIM：合成缓慢漂移的 IHR/Resp/Motion
            const float t = (float)sim_tick / (float)sleep_pillow::kStageFs;
            ihr    = 60.0f + 3.0f * sinf(2.0f * (float)M_PI * 0.02f * t);
            resp   = 15.0f + 2.0f * sinf(2.0f * (float)M_PI * 0.01f * t);
            motion = 0.05f * sinf(2.0f * (float)M_PI * 0.10f * t);
            sim_tick++;
        }
        builder.Push(ihr, resp, motion);
        if ((xTaskGetTickCount() - last_infer) >= pdMS_TO_TICKS(30000)) {
            last_infer = xTaskGetTickCount();
            if (builder.Ready()) {
                sleep_pillow::StageResult res;
                if (infer->PredictFrom(builder, &res) == ESP_OK && res.valid) {
                    sleep_pillow::EdgeAggregator::GetInstance()->OnStageResult((int)res.stage);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static esp_err_t StartStageTask() {
    if (s_stage_task) return ESP_OK;
    return xTaskCreate(StageTaskEntry, "stage_infer", 16384, nullptr, 2, &s_stage_task) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

// ============================================================
// BCG 体动任务：每秒 Feed 140 样本 → 多尺度 std 判定 + 心率/呼吸派生
// ============================================================
static void MotionTaskEntry(void* arg) {
    (void)arg;
    using sleep_pillow::MotionInfer;
    auto* infer = MotionInfer::GetInstance();
    if (infer->Init(140) != ESP_OK) {
        ESP_LOGE(TAG, "[motion] 初始化失败，任务退出");
        vTaskDelete(nullptr);
        return;
    }
    if (!g_bcg) {
        g_bcg = sleep_pillow::BcgAdc::GetInstance();
        const auto bcg_mode = (sleep_pillow::EdgeMode)CurrentBcgMode();
        g_bcg->Init(bcg_mode);
        if (bcg_mode == sleep_pillow::EdgeMode::kSim) {
            ESP_LOGW(TAG, "[motion] ★ BCG=SIM（合成模式，无真实压电陶瓷数据）");
        }
    }
    uint64_t sim_phase = 0;
    while (true) {
        if (CurrentBcgMode() == 0) {
            ESP_LOGW(TAG, "[motion] cfg 切回 OFF，任务退出");
            s_motion_task = nullptr;
            vTaskDelete(nullptr);
            return;
        }
        for (int i = 0; i < 140; ++i) {
            float s = 0.0f;
            if (g_bcg && g_bcg->ReadSample(&s)) {
                infer->Feed(s);
                g_bcg->FeedSample(s);
            } else if (CurrentBcgMode() == 1) {
                // SIM：合成 2s 周期 + 0.5s 体动脉冲
                const float t = (float)(sim_phase + i) / 140.0f;
                const float base  = 0.02f * sinf(2.0f * (float)M_PI * 0.5f * t);
                const float burst = (fmodf(t, 2.0f) < 0.5f)
                                        ? 0.5f * sinf(2.0f * (float)M_PI * 3.0f * t)
                                        : 0.0f;
                s = base + burst;
                infer->Feed(s);
                g_bcg->FeedSample(s);
            }
        }
        sim_phase += 140;
        if (g_bcg) g_bcg->TickAggregate();
        sleep_pillow::MotionResult mr;
        if (infer->Detect(&mr) == ESP_OK) {
            ESP_LOGD(TAG, "[motion] conf=%.2f motion=%d",
                     mr.confidence, (int)mr.is_motion);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t StartMotionTask() {
    if (s_motion_task) return ESP_OK;
    return xTaskCreate(MotionTaskEntry, "motion_infer", 12288, nullptr, 2, &s_motion_task) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

// ============================================================
// BLE 实时推送：每秒把当前窗口的实时值推给手机
// ============================================================
// 契约（与小程序 utils/bluetooth/BLEProvisioner.js 的 onRealtime 对齐）：
//   {"evt":"rt","hr":65,"rr":15,"motion":0,"snore":3,"stage":1}
//   任一字段 -1 = 该通道未启用（对应开关 OFF）
// 手机未连接时 BleProvisioner::Notify 会静默忽略，所以可以无脑每秒发。
static void BleRealtimeTaskEntry(void* arg) {
    (void)arg;
    auto* agg = sleep_pillow::EdgeAggregator::GetInstance();
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int snoring = -1, in_bed = -1;
        int hr = -1, rr = -1, motion = -1, snore = -1, stage = -1;
        agg->LiveSnapshot(&snoring, &in_bed, &hr, &rr, &motion, &snore, &stage);
        // 实时区只关心：snoring(本秒是否打鼾) / in_bed(是否在床)。
        // hr/rr/stage/snore_sec 也一并带着，供将来扩展；前端的实时区不展示它们
        // （心率/呼吸按产品定义放在"最近一帧"，从后端读）。
        char json[160] = {};
        snprintf(json, sizeof(json),
                 "{\"evt\":\"rt\",\"snoring\":%d,\"in_bed\":%d,"
                 "\"hr\":%d,\"rr\":%d,\"motion\":%d,\"snore\":%d,\"stage\":%d}",
                 snoring, in_bed, hr, rr, motion, snore, stage);
        BleProvisioner::GetInstance().Notify(json);
    }
}

static esp_err_t StartBleRealtimeTask() {
    static bool started = false;
    if (started) return ESP_OK;
    // 4096：Notify 内部会构造 std::string + NimBLE mbuf，留够余量
    started = (xTaskCreate(BleRealtimeTaskEntry, "ble_rt", 4096, nullptr, 1, nullptr) == pdPASS);
    return started ? ESP_OK : ESP_ERR_NO_MEM;
}

// ============================================================
// 三开关启动/停止统一处理
// ============================================================
static void ApplyEdgeCfg() {
    using sleep_pillow::EdgeMode;
    if (!g_cfg) return;
    const auto& c = g_cfg->Get();

    if (c.snore_mode != EdgeMode::kOff && !s_snore_task) StartAudioSnoreTask();
    if (c.stage_mode != EdgeMode::kOff && !s_stage_task) StartStageTask();
    if (c.bcg_mode   != EdgeMode::kOff && !s_motion_task) StartMotionTask();
    // OFF 时各 task 内部循环检查后自动退出

    auto* agg = sleep_pillow::EdgeAggregator::GetInstance();
    agg->SetSources(
        sleep_pillow::SnoreInfer::GetInstance(),
        sleep_pillow::StageInfer::GetInstance(),
        sleep_pillow::BcgAdc::GetInstance());

    // ★ 聚合器必须在这里跟着 cfg 启停。
    //   之前只在开机时 Start 过一次，导致"开机全 OFF、之后用蓝牙打开开关"
    //   时聚合器永远不启动 —— 一个 30s 帧都不会发。
    //   Start() 内部：已在运行就直接返回；全 OFF 则不启动。
    if (c.snore_mode == EdgeMode::kOff &&
        c.stage_mode == EdgeMode::kOff &&
        c.bcg_mode   == EdgeMode::kOff) {
        agg->Stop();
    } else {
        agg->Start();   // 首次启动会走 5 秒宽限期再开始 30s 封装
    }

    // 注：三开关不再上行到云端（设备本地状态，手机经 BLE 读写）。
    //     当前 cfg 会作为字段随每一帧上行，后端无需单独存。
    ESP_LOGW(TAG, "ApplyEdgeCfg: snore=%d stage=%d bcg=%d",
             (int)c.snore_mode, (int)c.stage_mode, (int)c.bcg_mode);
}

// BLE 0x05 SET_CFG 触发
static void OnCfgChanged(int snore, int stage, int bcg) {
    (void)snore; (void)stage; (void)bcg;
    ESP_LOGW(TAG, "=== EDGE_CFG_CHANGED ===");
    ApplyEdgeCfg();
}

// ============================================================
// 云端接入
// ============================================================
static void StartCloud() {
    CloudConfig cfg;
    cfg.host      = CLOUD_MQTT_HOST;
    cfg.port      = CLOUD_MQTT_PORT;
    cfg.device_id = CLOUD_DEVICE_ID;
    cfg.secret    = CLOUD_DEVICE_SECRET;
    cfg.sign_type = CLOUD_SIGN_TYPE;
    cfg.report_device_id = CLOUD_DEVICE_ID;
    if (CloudClient::GetInstance().Start(cfg)) {
        ESP_LOGI(TAG, "云端接入已启动 (device_id=%s)", cfg.device_id.c_str());
        // 上线报到由 CloudClient 在 MQTT_EVENT_CONNECTED 里自动发，这里不用管
    } else {
        ESP_LOGE(TAG, "云端接入启动失败");
    }
}

// BLE 回调：收到手机下发的加密 Wi-Fi 凭据
static void OnWifiCred(const std::string& ssid, const std::string& password) {
    ESP_LOGI(TAG, "BLE 下发 Wi-Fi 凭据: ssid=%s", ssid.c_str());
    const bool ok = s_wifi.Connect(ssid.c_str(), password.c_str());
    if (ok) {
        StartCloud();
        // 配网成功：按当前 cfg 启停三 task + 启聚合器
        ApplyEdgeCfg();
        ESP_LOGI(TAG, "配网成功，端侧模型按 cfg 启停");
    }
    char json[160] = {};
    snprintf(json, sizeof(json), "{\"evt\":\"wifi\",\"ok\":%s,\"ssid\":\"%s\"}",
             ok ? "true" : "false", ssid.c_str());
    BleProvisioner::GetInstance().Notify(json);
}

// BLE 0x02 旧采集开关：保留兼容（旧手机/调试器还在用），但不驱动 s_sampling
// ============================================================
// 开机自动重连 Wi-Fi + 上云
//
// ★ 为什么需要：以前只有 BLE 配网回调里才调 WifiManager::Connect()，
//   开机不自动连。于是**任何一次重启（含崩溃重启）设备都会离线**，
//   手机端看起来就是"手机一离开小程序 / 蓝牙一断，Wi-Fi 也断了" ——
//   其实设备是重启了，重启后没去重连。
//
// 放在独立任务里跑：ConnectStored 要等关联 + DHCP（最长 35s），
// SyncTime 还要再等最多 30s，不能阻塞 Run() 里的 BLE 启动。
// ============================================================
static void AutoConnectTaskEntry(void* /*arg*/) {
    if (s_wifi.ConnectStored()) {
        StartCloud();
        ApplyEdgeCfg();
        ESP_LOGI(TAG, "开机自动上云完成");
    } else {
        ESP_LOGI(TAG, "未自动上云（无凭据或连不上），等待 BLE 配网");
    }
    vTaskDelete(nullptr);
}

static void OnSampling(bool enable) {
    ESP_LOGI(TAG, "BLE 0x02 legacy sampling=%s (已由三态 cfg 接管，忽略)",
             enable ? "ON" : "OFF");
    BleProvisioner::GetInstance().Notify(
        enable ? "{\"evt\":\"ctrl\",\"sampling\":true}" : "{\"evt\":\"ctrl\",\"sampling\":false}");
}

void AppMain::Run() {
    // 1. 初始化日志
    esp_log_level_set("*", APP_LOG_LEVEL);
    ESP_LOGI(TAG, "=== SleepPillow Starting (v1.0+: 端侧三模型 + 三开关) ===");

    // 1.5 ★ 初始化 NVS，再读三开关配置
    //   必须在这里先 init：AppConfigStore 靠 NVS 存三开关，
    //   之前 NVS 是在 WifiManager::EnsureInited() 里才初始化（配网时），
    //   导致这里 nvs_open 失败 → 打 "NVS open failed ... use defaults (全 OFF)"
    //   → 三开关既不落盘也读不回来，重启永远回到全 OFF。
    {
        esp_err_t nvs_ret = nvs_flash_init();
        if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
            nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(TAG, "NVS 分区需要擦除后重建 (err=%s)", esp_err_to_name(nvs_ret));
            nvs_flash_erase();
            nvs_ret = nvs_flash_init();
        }
        if (nvs_ret != ESP_OK) {
            ESP_LOGE(TAG, "NVS 初始化失败: %s（三开关将无法持久化）",
                     esp_err_to_name(nvs_ret));
        }
    }
    g_cfg = sleep_pillow::AppConfigStore::GetInstance();
    g_cfg->Init();

    // 2. 初始化 Board
    Board* board = Board::GetInstance();
    board->Init();
    ESP_LOGI(TAG, "Board: %s", board->GetBoardName());

    // 2.5 触发三个 Net 单例构造（保证 snore_infer / stage_infer 调 GetInstance() 拿得到）
    (void)sleep_pillow::SnoreNet::GetInstance();
    (void)sleep_pillow::StageNet::GetInstance();
    (void)sleep_pillow::MotionNet::GetInstance();

    // 2.6 按**已保存的** cfg 统一启动：三个模型任务 + EdgeAggregator
    //     ★ 必须调 ApplyEdgeCfg()，不能只 agg->Start()：
    //       否则开机后三个模型任务都不会启动，必须等手机通过 BLE 重发 SET_CFG，
    //       或等 Wi-Fi 通了走云端路径 —— Wi-Fi 不通时永远等不到，
    //       表现为"开关明明是开的，却没有真实数据"。
    ApplyEdgeCfg();

    // 3. 启动 BLE 配网
    BleProvisioner& ble = BleProvisioner::GetInstance();
    ble.SetHandlers(OnWifiCred, OnSampling, OnCfgChanged);
    ble.SetDeviceId(CLOUD_DEVICE_ID);
    if (!ble.Start(BLE_DEVICE_NAME)) {
        ESP_LOGE(TAG, "BLE 配网启动失败");
    }

    // 3.5 BLE 实时推送（每秒推当前窗口实时值；手机未连接时静默丢弃）
    StartBleRealtimeTask();

    // 3.6 ★ 开机自动重连 Wi-Fi 并上云（独立任务，不阻塞 BLE 启动）
    xTaskCreate(AutoConnectTaskEntry, "auto_net", 6144, nullptr, 4, nullptr);

    // 4. 等 BLE 配网后 StartCloud 会自动触发 ApplyEdgeCfg
    ESP_LOGI(TAG, "等待 BLE 配网（cfg=%d/%d/%d）",
             CurrentSnoreMode(), CurrentStageMode(), CurrentBcgMode());

    // 5. 主循环：idle 让 FreeRTOS 调度
    //    三个 task 在 cfg 非 OFF 时运行，EdgeAggregator 30s 上 frame；
    //    注册/上下线由 IoTDA「设备状态变更」规则处理，固件不参与。
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
