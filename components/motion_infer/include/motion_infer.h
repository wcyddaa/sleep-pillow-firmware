// motion_infer.h - BCG 体动检测
// ============================================================
// 两条路径（可用宏 MOTION_USE_NN 切换，默认走"多尺度 std 阈值"更省）：
//  A) 轻量阈值：对原始体动信号做多尺度 std（1/30/60/120/240s），
//     以"短窗 std / 长窗 std"比值 + 绝对门限判定 rest / motion（省算力、无模型）
//  B) 神经网络：BCG-classify/outputs/motion_bigru_fcn.onnx（BiGRU-FCN, T x 14 -> 3 类），
//     逐 10s 窗输出 rest / motion / bedempty，权重由 components/models 内嵌
// 说明：NN 路径要求调用方提供与 BCG-classify 对齐的 14 维多尺度特征；
//       本组件提供 std 序列，其余特征（kurt/mad/ptp/zcr/ratio_*）口径待人工确认。
// ============================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

namespace sleep_pillow {

inline constexpr int kMotionFeatureDim = 14;   // 与 motion_bigru_fcn 输入一致
inline constexpr int kMotionWinSec     = 10;   // 每个窗 10s
inline constexpr int kMotionMaxT       = 128;  // 一次前向最多 128 个窗

struct MotionResult {
    bool  is_motion = false;
    float confidence = 0.0f;    // [0,1]
    int   cls = 0;              // NN: 0=rest 1=motion 2=bedempty；阈值路径: 0/1
    // 多尺度 std（秒 1/30/60/120/240）
    float std_1s = 0, std_30s = 0, std_60s = 0, std_120s = 0, std_240s = 0;
};

class MotionInfer {
public:
    static MotionInfer* GetInstance();

    // fs_hz：原始体动信号采样率（BCG 默认 140Hz）
    esp_err_t Init(int fs_hz = 140);
    void Deinit();
    bool IsReady() const { return initialized_; }

    // 追加一个原始体动样本（如 BCG motion 通道 / 雷达体动幅度）
    void Feed(float sample);

    // 计算多尺度 std（不足时按已有长度算）
    void MultiScaleStd(float* std_1s, float* std_30s, float* std_60s,
                       float* std_120s, float* std_240s) const;

    // 阈值路径：判定当前是否存在体动
    esp_err_t Detect(MotionResult* out) const;

    // NN 路径：feat 布局 [T][14]；probs_out 可选 [T][3]
    esp_err_t InferNN(const float* feat, int T, MotionResult* per_window, float* probs_out);

    // 上电自检（NN 模型测试向量）
    float ModelSelfTest();

private:
    MotionInfer() = default;

    bool   initialized_ = false;
    int    fs_ = 140;
    float* ring_ = nullptr;      // 原始信号环形缓冲（PSRAM）
    size_t ring_cap_ = 0;        // 样本数（240s）
    size_t head_ = 0;
    size_t filled_ = 0;
    // PSRAM 复用：InferNN 内的 logits[kMaxT*kCls]=128*3 floats 原压在任务栈
    // 解决：motion_infer 任务跑 NN 路径时栈溢出
    float* nn_logits_ = nullptr;

    static constexpr const char* TAG = "MotionInfer";
};

}  // namespace sleep_pillow
