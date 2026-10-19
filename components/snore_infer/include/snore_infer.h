// snore_infer.h - 鼾声检测推理（64 维手工特征前端 + int8 MLP + 事件聚合）
// ============================================================
// 输入：1 秒音频帧（16kHz / 16bit / 单声道 PCM，长度 kSnoreFrameSamples）
// 链路：
//   512 点 FFT(hop160/hann/101 帧) → 26 通道 mel 功率 → log10 → 13 阶 MFCC(Δ)
//   + 谱质心/带宽/滚降/平坦度 + ZCR/RMS（各取 mean/std）→ 64 维
//   → int8 MLP(64->48->24->2，权重来自 snoring-classify/edge_snore_mlp_int8.onnx)
//   → 鼾声概率 → 事件聚合（平滑 3 窗 + 滞回 0.6/0.4 + 去毛刺 + 合并）
// 输出：鼾声概率 [0,1]、事件级统计（鼾声时长/占比/鼾声指数）
// ============================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

namespace sleep_pillow {

// 模型输入帧：1 秒 @16kHz 单声道
inline constexpr int      kSnoreSampleRate  = 16000;
inline constexpr size_t   kSnoreFrameSamples = 16000;
// 64 维手工特征维度（MFCC13+Δ13 各 mean/std + 6 类谱/时域统计各 mean/std）
inline constexpr size_t   kSnoreFeatureDim   = 64;

struct SnoreResult {
    float probability = 0.0f;   // 鼾声概率 [0,1]
    bool  is_snore    = false;  // probability >= 阈值
};

// 事件聚合统计（对应 snoring-classify 报告的 event_aggregation 口径）
struct SnoreStats {
    uint32_t windows = 0;              // 累计窗口(秒)数
    uint32_t snore_windows = 0;        // 判定为鼾声的窗口数
    uint32_t events = 0;               // 鼾声事件数
    float    snore_duration_sec = 0;   // 鼾声总时长（秒）
    float    snore_ratio = 0;          // 鼾声时长 / 总时长
    float    snore_index_per_hour = 0; // 鼾声指数（次/小时）
};

class SnoreInfer {
public:
    static SnoreInfer* GetInstance();

    esp_err_t Init();
    esp_err_t Deinit();
    bool IsReady() const { return initialized_; }

    void SetThreshold(float threshold);

    // 主推理接口：输入任意长度 PCM，输出鼾声概率
    esp_err_t Predict(const int16_t* pcm, size_t samples, float* out_prob);
    esp_err_t PredictFrame(const int16_t* pcm, float* out_prob);
    esp_err_t PredictFull(const int16_t* pcm, size_t samples, SnoreResult* out);

    // 64 维手工特征前端（float 输出，未标准化；模型内部再做 mean/std 标准化）
    esp_err_t ExtractFeatures(const int16_t* pcm, size_t samples,
                              float* out_features, size_t max_dims,
                              size_t* out_dims);

    // 逐窗喂入概率，更新事件聚合状态（线程内串行调用）
    void UpdateAggregation(float prob);
    SnoreStats GetStats() const { return stats_; }
    // 触发一次事件收尾（例如停止采集时），把未闭合的激活段计入统计
    void FlushAggregation();

    // 上电自检：把模型测试向量跑一遍，返回最大绝对误差（<0 表示失败）
    float ModelSelfTest();

private:
    SnoreInfer() = default;
    ~SnoreInfer() = default;
    SnoreInfer(const SnoreInfer&) = delete;
    SnoreInfer& operator=(const SnoreInfer&) = delete;

    void CloseSegment(uint32_t start, uint32_t end);
    void AccumulateStats();

    bool  initialized_ = false;
    float threshold_   = 0.5f;

    // 前端常量表（Init 时初始化）
    float* tw_re_ = nullptr;   // FFT 旋转因子 cos，512
    float* tw_im_ = nullptr;   // FFT 旋转因子 -sin，512
    float* fft_re_ = nullptr;  // FFT 实部工作区，512
    float* fft_im_ = nullptr;  // FFT 虚部工作区，512
    // PSRAM 复用缓冲（Init 一次性分配，ExtractFeatures/Predict 复用）
    // 解决：power[257]+mag[257]+feat[64] 原压在任务栈上触发 snore_infer 栈溢出
    float* power_ = nullptr;   // 257 floats = 1028B，每帧功率谱
    float* mag_   = nullptr;   // 257 floats = 1028B，每帧幅度谱
    float* feat_  = nullptr;   // 64 floats = 256B，Predict 输入

    // 事件聚合状态
    SnoreStats stats_;
    float   p_ring_[3] = {0, 0, 0};   // 最近 3 窗概率（用于平滑）
    uint32_t ring_n_ = 0;
    bool    active_state_ = false;    // 滞回状态
    bool    in_run_ = false;
    uint32_t run_start_ = 0;
    uint32_t last_seg_start_ = 0;
    uint32_t last_seg_end_ = 0;
    bool    has_last_seg_ = false;

    static constexpr const char* TAG = "SnoreInfer";
};

}  // namespace sleep_pillow
