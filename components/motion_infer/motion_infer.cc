// motion_infer.cc - BCG 体动检测（多尺度 std 阈值 + 可选 BiGRU-FCN int8）
#include "motion_infer.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "model_nets.h"

namespace sleep_pillow {

MotionInfer* MotionInfer::GetInstance() {
    static MotionInfer inst;
    return &inst;
}

esp_err_t MotionInfer::Init(int fs_hz) {
    if (initialized_) return ESP_OK;
    fs_ = (fs_hz > 0) ? fs_hz : 140;
    ring_cap_ = (size_t)fs_ * 240;   // 240s 用于最长尺度
    ring_ = (float*)heap_caps_malloc(ring_cap_ * sizeof(float),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ring_ == nullptr) {
        ring_ = (float*)heap_caps_malloc(ring_cap_ * sizeof(float), MALLOC_CAP_8BIT);
    }
    if (ring_ == nullptr) {
        ESP_LOGE(TAG, "环形缓冲分配失败 (%u B)", (unsigned)(ring_cap_ * sizeof(float)));
        return ESP_ERR_NO_MEM;
    }
    memset(ring_, 0, ring_cap_ * sizeof(float));
    // NN 推理输出 logits 缓冲（kMaxT*kCls = 128*3 floats）挪到 PSRAM
    nn_logits_ = (float*)heap_caps_malloc(
        (size_t)kMotionMaxT * (size_t)MotionNet::kCls * sizeof(float),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (nn_logits_ == nullptr) {
        nn_logits_ = (float*)heap_caps_malloc(
            (size_t)kMotionMaxT * (size_t)MotionNet::kCls * sizeof(float),
            MALLOC_CAP_8BIT);
    }
    if (nn_logits_ == nullptr) {
        ESP_LOGE(TAG, "NN logits 缓冲分配失败");
        heap_caps_free(ring_);
        ring_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    initialized_ = true;
    ESP_LOGI(TAG, "初始化完成: fs=%dHz, 缓冲=%us, NN logits=%uB",
             fs_, (int)(ring_cap_ / fs_),
             (unsigned)((size_t)kMotionMaxT * (size_t)MotionNet::kCls * sizeof(float)));
    return ESP_OK;
}

void MotionInfer::Deinit() {
    if (ring_) { heap_caps_free(ring_); ring_ = nullptr; }
    if (nn_logits_) { heap_caps_free(nn_logits_); nn_logits_ = nullptr; }
    initialized_ = false;
}

void MotionInfer::Feed(float sample) {
    if (!ring_) return;
    ring_[head_] = sample;
    head_ = (head_ + 1) % ring_cap_;
    if (filled_ < ring_cap_) ++filled_;
}

// 取最近 n 个样本的 std（不足 n 时用已有）
static float StdOfLast(const float* ring, size_t cap, size_t head, size_t filled, size_t n) {
    if (filled == 0) return 0.0f;
    if (n > filled) n = filled;
    float mean = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const size_t idx = (head + cap - n + i) % cap;
        mean += ring[idx];
    }
    mean /= (float)n;
    float var = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const size_t idx = (head + cap - n + i) % cap;
        const float d = ring[idx] - mean;
        var += d * d;
    }
    return sqrtf(var / (float)n);
}

void MotionInfer::MultiScaleStd(float* s1, float* s30, float* s60,
                                float* s120, float* s240) const {
    if (!ring_) {
        if (s1) *s1 = 0;
        if (s30) *s30 = 0;
        if (s60) *s60 = 0;
        if (s120) *s120 = 0;
        if (s240) *s240 = 0;
        return;
    }
    const size_t f = (size_t)fs_;
    if (s1)   *s1   = StdOfLast(ring_, ring_cap_, head_, filled_, f * 1);
    if (s30)  *s30  = StdOfLast(ring_, ring_cap_, head_, filled_, f * 30);
    if (s60)  *s60  = StdOfLast(ring_, ring_cap_, head_, filled_, f * 60);
    if (s120) *s120 = StdOfLast(ring_, ring_cap_, head_, filled_, f * 120);
    if (s240) *s240 = StdOfLast(ring_, ring_cap_, head_, filled_, f * 240);
}

esp_err_t MotionInfer::Detect(MotionResult* out) const {
    if (!initialized_ || out == nullptr) return ESP_ERR_INVALID_STATE;
    float s1, s30, s60, s120, s240;
    MultiScaleStd(&s1, &s30, &s60, &s120, &s240);
    out->std_1s = s1; out->std_30s = s30; out->std_60s = s60;
    out->std_120s = s120; out->std_240s = s240;

    // 相对门限：短窗能量显著高于长窗基线 → 体动；兼顾绝对门限抗噪声
    const float baseline = (s60 > 1e-6f) ? s60 : 1e-6f;
    const float ratio = s1 / baseline;
    const bool motion = (ratio > 2.5f) && (s1 > 1e-3f);
    out->is_motion = motion;
    out->cls = motion ? 1 : 0;
    float conf = (ratio - 1.0f) / (4.0f - 1.0f);   // ratio 1~4 线性映射到 0~1
    if (conf < 0.0f) conf = 0.0f;
    if (conf > 1.0f) conf = 1.0f;
    out->confidence = conf;
    return ESP_OK;
}

esp_err_t MotionInfer::InferNN(const float* feat, int T, MotionResult* per_window,
                               float* probs_out) {
    if (!initialized_ || feat == nullptr || T <= 0 || T > kMotionMaxT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (MotionNet::GetInstance()->Init() != ESP_OK) return ESP_ERR_NO_MEM;
    // logits 缓冲已在 Init 时一次性分配到 PSRAM，避免 1536B 落在任务栈上
    float* logits = nn_logits_;
    esp_err_t err = MotionNet::GetInstance()->Forward(feat, T, logits, probs_out);
    if (err != ESP_OK) return err;
    if (per_window) {
        for (int t = 0; t < T; ++t) {
            const float* p = probs_out ? (probs_out + t * MotionNet::kCls)
                                       : (logits + t * MotionNet::kCls);
            int best = 0;
            for (int i = 1; i < MotionNet::kCls; ++i) if (p[i] > p[best]) best = i;
            per_window[t].cls = best;
            per_window[t].is_motion = (best == 1);
            per_window[t].confidence = p[best];
        }
    }
    return ESP_OK;
}

float MotionInfer::ModelSelfTest() {
    if (MotionNet::GetInstance()->Init() != ESP_OK) return -1.0f;
    return MotionNet::GetInstance()->SelfTest();
}

}  // namespace sleep_pillow
