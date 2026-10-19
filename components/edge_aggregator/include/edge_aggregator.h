// edge_aggregator.h - 30s 结构化数据聚合器（降级 + 上报）
// ============================================================
// 输入（来自三个独立模型/采集器，按运行时 cfg 决定是否有效）：
//   snore : 鼾声（每秒回调 OnSnoreProb）
//   stage : 睡眠分期（每 30s 回调 OnStageResult；OFF 时不调）
//   bcg   : 心率/呼吸/体动（每 1s 回调 OnBcgDerived；OFF 时不调）
// 输出（每 30s 窗口到期）：
//   SleepFrame → MQTT /device/{id}/frame (JSON)
//   缺哪个 → 对应字段写 -1（前端降级）
//   flags   : bit0=snore_off, bit1=stage_off, bit2=bcg_off
// ============================================================
#pragma once

#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "snore_infer.h"
#include "stage_infer.h"
#include "bcg_adc.h"

namespace sleep_pillow {

struct SleepFrame {
    uint32_t window_start_ts;   // 窗口开始时间戳(秒)
    uint32_t window_end_ts;     // 窗口结束时间戳
    int8_t   stage;             // -1=关闭, 0=Wake, 1=Light, 2=Deep
    int8_t   heart_rate;        // -1=关闭, 0-200
    int8_t   resp_rate;         // -1=关闭, 0-50
    int8_t   motion;            // -1=关闭, 0=静, 1=体动
    int8_t   snore_seconds;     // -1=关闭, 0-30
    uint8_t  flags;             // bit0=snore_off, bit1=stage_off, bit2=bcg_off
    char     json[288];         // 含 device_id + cfg 子对象
    size_t   json_len;
};

class EdgeAggregator {
public:
    static EdgeAggregator* GetInstance();

    // 关联三个数据源（snore/stage 来自对应模型，bcg 来自 BcgAdc）
    void SetSources(class SnoreInfer* snore,
                    class StageInfer* stage,
                    BcgAdc* bcg);

    // 启动 30s 周期任务
    esp_err_t Start();
    void       Stop();

    // 三个回调（由 snore/stage 任务在产出新结果时调用）
    void OnSnoreProb(float prob);          // 每秒调用
    void OnStageResult(int stage_code);    // 每 30s 调用（0=Wake 1=Light 2=Deep）
    void OnBcgDerived();                   // 每秒调用（bcg 任务里 TickAggregate）

    // 拉最近一帧（前端详情页/AI 分析用）
    const SleepFrame& Latest() const { return latest_; }

    // 当前窗口的实时值（供 BLE 每秒推送；按 cfg 判定，未启用通道给 -1）
    //   snoring  = 上一秒是否检出鼾声（0/1，-1=通道 OFF）—— 这是"实时"真正要的值
    //   in_bed   = 是否在床（0/1，-1=通道 OFF / 数据源未接入）
    //   hr/rr/motion 取最近一次 BCG 派生，snore_sec 为本窗口累计，stage 为最近一次分期
    void LiveSnapshot(int* snoring, int* in_bed,
                      int* hr, int* rr, int* motion, int* snore_sec, int* stage) const;

private:
    EdgeAggregator() = default;

    void ResetWindow();
    void PublishCurrentFrame();

    SnoreInfer* snore_ = nullptr;
    StageInfer* stage_ = nullptr;
    BcgAdc*     bcg_   = nullptr;

    // 当前 30s 窗口累计
    time_t   win_start_ts_   = 0;       // 墙上时钟 epoch 秒（NTP 未同步前是 boot 秒）
    int      snore_sec_cnt_  = 0;       // 鼾声累计秒数
    int      snore_valid_cnt_ = 0;      // 本窗口收到**有效音频帧**的秒数
                                        // 0 → 整个窗口没拿到数据（麦克风异常/未接）
                                        // 上报时 snore_seconds 置 -1，避免把"没数据"当成"没鼾声"
    int      last_snore_hit_ = 0;       // 上一秒是否检出鼾声（实时区用）
    int      last_stage_     = -1;
    int      last_hr_        = -1;
    int      last_rr_        = -1;
    int      last_motion_    = -1;
    uint8_t  flags_          = 0;       // 哪些通道已关闭

    TaskHandle_t task_       = nullptr;
    bool         running_    = false;
    SleepFrame   latest_{};

    // 宽限期：开关刚打开时先等这么久再开始 30s 封装，
    // 让这期间打开的其它开关并入同一个窗口；0 表示不在宽限期。
    time_t        grace_until_ts_ = 0;

    static constexpr const char* TAG = "EdgeAggregator";
    static constexpr int kWindowSec = 30;
    static constexpr int kGraceSec  = 5;
};

}  // namespace sleep_pillow
