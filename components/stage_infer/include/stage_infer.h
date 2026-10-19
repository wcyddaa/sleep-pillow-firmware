// stage_infer.h - 睡眠分期（EdgeBiGatedNet int8）+ 4 通道 4Hz 特征构造
// ============================================================
// 模型：PSG-classify/outputs/model_stage_distill.onnx（双向堆叠蒸馏学生，EdgeBiGatedNet，(4,360) -> Wake/Light/Deep）
// 输入通道契约（与 BCG-classify/report_edge_bcg.json 一致）：
//   ch0 IHR      : 心率序列（整段 z-score）
//   ch1 Resp     : 呼吸序列（整段 z-score）
//   ch2 Motion   : 体动序列（逐 30s epoch z-score）
//   ch3 RespMask : 常数 1（BCG 恒可派生呼吸）
// 输入长度 360 = 3 个 30s epoch @4Hz（即 90 秒上下文）
// ============================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

namespace sleep_pillow {

inline constexpr int kStageFs        = 4;     // 目标采样率 Hz
inline constexpr int kStageEpochSec  = 30;
inline constexpr int kStagePerEpoch  = kStageFs * kStageEpochSec;   // 120
inline constexpr int kStageCtxEpochs = 3;
inline constexpr int kStageSeqLen    = kStagePerEpoch * kStageCtxEpochs;  // 360

enum class SleepStage : int { kWake = 0, kLight = 1, kDeep = 2 };

struct StageResult {
    SleepStage stage = SleepStage::kLight;
    float probs[3] = {0, 0, 0};     // Wake/Light/Deep
    bool  valid = false;
};

// 4 通道 4Hz 环形缓冲 + 归一化（整段 z-score / 逐 epoch z-score）
class StageFeatureBuilder {
public:
    // 每 250ms（4Hz）调用一次，推入三路信号（RespMask 恒为 1）
    void Push(float ihr, float resp, float motion);
    // 是否已有 >= 3 个 epoch（360 点）的历史
    bool Ready() const { return filled_ >= kStageSeqLen; }
    int  Filled() const { return filled_; }
    // 输出 [4][360]（ch-major），按通道契约做归一化
    void Build(float* out_4x360) const;
    void Reset();

private:
    float buf_[4][kStageSeqLen] = {};   // 左侧最旧、右侧最新
    int   filled_ = 0;                  // 已填充样本数（<= kStageSeqLen）
};

class StageInfer {
public:
    static StageInfer* GetInstance();

    esp_err_t Init();
    esp_err_t Deinit();
    bool IsReady() const { return initialized_; }

    // 直接推理 [4][360]
    esp_err_t Predict(const float* ch4x360, StageResult* out);

    // 从特征缓冲构造输入并推理
    esp_err_t PredictFrom(const StageFeatureBuilder& builder, StageResult* out);

    // 上电自检（模型测试向量）
    float ModelSelfTest();

    // 暴露 PSRAM 复用缓冲（StageFeatureBuilder::Build 借用 build_pad_ 避免 tmp[] 落任务栈）
    // 公开 getter 而非公开指针：避免外部误改地址，保持单例所有权在 StageInfer。
    float* GetInferBuf() { return infer_buf_; }
    float* GetBuildPad() { return build_pad_; }

private:
    StageInfer() = default;
    bool initialized_ = false;
    // PSRAM 复用缓冲（Init 一次性分配，PredictFrom/Build 复用）
    // 解决：x[4*360]=5760B + tmp[360]=1440B 原本落在任务栈上导致 stage_infer 任务栈溢出
    float* infer_buf_  = nullptr;   // kCh*kStageSeqLen floats  -> PredictFrom 内的 x[]
    float* build_pad_  = nullptr;   // kStageSeqLen floats      -> Build 内的 tmp[]
    static constexpr const char* TAG = "StageInfer";
};

}  // namespace sleep_pillow
