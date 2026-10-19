// model_nets.h - 三个端侧模型的端侧前向实现（单例 + 复用 scratch 缓冲）
// ============================================================================
// 全部使用 int8 权重（见 sleep_models.h）+ float 激活，按需反量化；
// scratch 缓冲在 Init() 时一次性分配（优先 PSRAM），避免常驻 .bss。
// ============================================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

namespace sleep_pillow {

// 鼾声：输入 64 维标准化特征 -> 2 类 logits
class SnoreNet {
public:
    static SnoreNet* GetInstance();
    static constexpr int kIn = 64;
    static constexpr int kOut = 2;
    bool IsReady() const { return ready_; }
    esp_err_t Init();
    void Forward(const float* features, float* logits_out) const;
    // 上电自检：与 PC 端 onnxruntime 期望输出比对，返回最大绝对误差
    float SelfTest() const;
private:
    SnoreNet() = default;
    bool ready_ = false;
};

// 睡眠分期：输入 [4][360] -> 3 类 logits/probs
class StageNet {
public:
    static StageNet* GetInstance();
    static constexpr int kCh = 4;
    static constexpr int kLen = 360;
    static constexpr int kCls = 3;
    bool IsReady() const { return ready_; }
    esp_err_t Init();
    void Deinit();
    // x 布局 [kCh][kLen]（IHR/Resp/Motion/RespMask）；probs 长度 kCls
    esp_err_t Forward(const float* x, float* logits_out, float* probs_out) const;
    float SelfTest() const;
private:
    StageNet() = default;
    bool ready_ = false;
    float* buf_ = nullptr;
    size_t buf_bytes_ = 0;
};

// BCG 体动：输入 [T][14] 多尺度特征 -> 逐窗 3 类 logits/probs
class MotionNet {
public:
    static MotionNet* GetInstance();
    static constexpr int kIn = 14;
    static constexpr int kMaxT = 128;
    static constexpr int kCls = 3;
    bool IsReady() const { return ready_; }
    esp_err_t Init();
    void Deinit();
    // feat 布局 [T][kIn]；probs 输出 [T][kCls]
    esp_err_t Forward(const float* feat, int T, float* logits_out, float* probs_out) const;
    float SelfTest() const;
private:
    MotionNet() = default;
    bool ready_ = false;
    float* buf_ = nullptr;
    size_t buf_bytes_ = 0;
};

}  // namespace sleep_pillow
