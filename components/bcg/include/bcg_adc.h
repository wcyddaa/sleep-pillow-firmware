// bcg_adc.h - 压电陶瓷(BCG) 采集与三通道派生
// ============================================================
// 数据源：
//   REAL: 板型 ADC 输入（GPIO/通道由板型定义），140Hz 采样压电陶瓷
//         偏置电路 VCC/2=1.65V，ADC raw→减中点 2048→归一化 ±1
//         ADC 失败绝不回退 SIM，任务等待重试
//   SIM : 固件内置合成 140Hz 信号（心跳 60bpm + 呼吸 15bpm + 体动 0.2Hz）
// 派生（每 30s 聚合一次，输出给 stage 通道和 EdgeAggregator）：
//   - heart_rate  : 过零率峰检测 + 不应期 + 30s 窗
//   - resp_rate   : 占位（需 0.1-0.5Hz 带通 + FFT）
//   - motion      : 1s std 阈值（喂给 motion_infer）
// ============================================================
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "motion_infer.h"
#include "app_config.h"

// 板级抽象层统一硬件接口（板型提供实现，本组件只面向接口）
class BoardAdcInput;

namespace sleep_pillow {

// 采样率（业务契约，140Hz）。ADC 的 GPIO/通道/量程配置属于板级抽象层，
// 由板型 BoardAdcInput 封装；原始计数中点 = 板型偏置电压（1.65V → 12bit 2048），
// 本组件读 raw 后减中点归一化到 ±1。
inline constexpr int kBcgSampleRateHz   = 140;

// 派生输出（每 60s 滚动一次）
struct BcgDerived {
    int8_t  heart_rate = -1;   // bpm, -1=数据不足
    int8_t  resp_rate  = -1;   // 次/分
    bool    motion     = false;
    uint32_t ts_sec    = 0;    // 最近一次更新时间
};

// ★ 压电陶瓷接入检测结论（Init 时实测 ADC 波形得出）
//
//   注意区分两件事：
//     · 「硬件有没有接上」—— 看平均电平是否落在偏置中点、读数是否完全死住
//     · 「床上有没有人」  —— 看波动幅度够不够大
//   没人躺时 p2p 很小是**正常待机**，不是故障，所以单列 kIdle 一档。
enum class BcgHealth : int {
    kOk = 0,      // 正常，检测到明显波动（有人躺着）
    kIdle,        // ★ 已接入但当前无受力（床上没人，待机状态，不是故障）
    kAdcFail,     // ADC 初始化失败 / 读不到样本
    kFlat,        // 读数恒定死住（≤2 LSB）→ 压电陶瓷未接入 / 对地短路
    kNoBias,      // 平均电平贴 0 或满幅 → 缺 1.65V 偏置电路，或接错引脚
};
const char* BcgHealthText(BcgHealth h);

class BcgAdc {
public:
    static BcgAdc* GetInstance();

    // 初始化；按 mode 决定 REAL/SIM（REAL 时向板型索取 ADC 输入）
    esp_err_t Init(EdgeMode mode);

    // 每次读一个 140Hz 样本（REAL 走 ADC，SIM 走合成）
    // 返回 false = 当前模式不工作
    bool ReadSample(float* out);

    // 推送样本到内部缓冲；调用方每 1/140 秒调用一次
    // 内部会判断是否到达 60s 窗口，触发一次派生
    void FeedSample(float s);

    // 强制触发一次派生（用于聚合器在 30s 窗口结束时拉取）
    void TickAggregate();

    // 取最近一次派生结果
    BcgDerived GetDerived() const { return derived_; }

    // 上电自检（合成信号走一遍派生）
    float SelfTest();

    // ★ 压电陶瓷接入检测：Init 后调一次，实测 0.5s 波形判断硬件是否真的接上。
    //   只做诊断，不影响采集；SIM 模式下直接返回 kOk（不做硬件检测）。
    BcgHealth DetectSensor();

private:
    BcgAdc() = default;
    ~BcgAdc();

    EdgeMode mode_ = EdgeMode::kOff;
    bool     adc_inited_ = false;

    // 板型提供的 ADC 输入实现（底层封装：单元/通道/量程/引脚）
    BoardAdcInput* adc_ = nullptr;

    // 合成信号相位
    uint64_t sim_phase_ = 0;

    // 多尺度 std 计算用环形缓冲（240s 容量）
    static constexpr int kRingSec = 240;
    static constexpr int kRingCap = kRingSec * kBcgSampleRateHz;
    float*  ring_ = nullptr;       // PSRAM
    int     head_ = 0;
    int     filled_ = 0;

    // 自相关心率 / FFT 呼吸窗口：30s 与 EdgeAggregator 的窗口对齐，
    // 这样 30s 帧封装时就能拿到最近一次派生，不需要再等 60s。
    // 真实硬件就位后,如果发现 30s 数据不足,可调回 60s。
    static constexpr int kDeriveWinSec = 30;
    static constexpr int kDeriveWinSamples = kDeriveWinSec * kBcgSampleRateHz;
    int      last_derive_pos_ = 0;

    BcgDerived derived_{};

    // ★ 「无有效信号」告警只打一次，避免每个窗口刷屏
    mutable bool warned_no_signal_ = false;
    // ★ ADC 读失败计数：140Hz 调用下逐次打印会刷爆串口，每 5 秒才提示一次
    uint32_t read_err_cnt_ = 0;
    // ★ 诊断日志去重：状态没变就不重复打印（否则每 10 秒一条同样内容）
    int      last_diag_verdict_ = -1;
    uint32_t last_diag_ms_ = 0;

    static constexpr const char* TAG = "BcgAdc";

    // ★ 信号有效性门限：压电陶瓷没接 / 没受力时，ADC 只是悬空噪声，
    //   直接用会数出假心率/假呼吸/假体动。三个条件全满足才算有效：
    //     1) 偏置在 1.65V 附近（raw 在 kMinBiasRaw~kMaxBiasRaw）
    //     2) 窗口内峰峰值 >= kMinPeakToPeak
    //     3) 窗口内标准差 >= kMinStd
    //
    //   ⚠ 阈值标定说明（之前设太高会把真实信号误判成没接）：
    //     · ReadSample 归一化后范围 [-1, 1]，1.0 = 满幅 1.65V
    //     · 12bit ADC 的 LSB = 1/2048 ≈ 0.00049，悬空噪声 p2p 约 0.003~0.005
    //     · 压电陶瓷**直接接 ADC、无放大**时，人体 BCG 信号通常只有 10~100mV
    //       → 归一化 p2p 约 0.006~0.06
    static constexpr float kMinPeakToPeak = 0.005f;   // raw ≈ 10 LSB ≈ 8mV
    static constexpr float kMinStd        = 0.0006f;

    // ★ 偏置窗口（raw 量纲，0~4095）：压电信号必须叠在 1.65V 中点附近摆动。
    //   实测「没接偏置电路」时 mean_raw 会贴在 0~200 或 3900~4095，
    //   那种波形负半周被削掉，算出来的心率/呼吸全是假的。
    static constexpr float kMinBiasRaw = 400.0f;
    static constexpr float kMaxBiasRaw = 3700.0f;

    void AllocateRingIfNeeded();
    // ★ 先验有效性：出参可选。out_mean 是归一化均值（0 = 1.65V 中点）
    bool SignalValid(float* out_p2p, float* out_std, float* out_mean = nullptr) const;
    void ComputeHeartRate();     // 60s 窗
    void ComputeRespRate();      // 60s 窗
    void UpdateMotion();
};

}  // namespace sleep_pillow
