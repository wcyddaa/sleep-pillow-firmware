// model_nets.cc - 三个端侧模型的端侧前向实现
#include "model_nets.h"

#include <math.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "sleep_models.h"
#include "nn_ops.h"

namespace sleep_pillow {
namespace m = sleep_pillow::models;
namespace nn = sleep_pillow::nn;

using m::kSnoreIn;
using m::kSnoreH0;
using m::kSnoreH1;
using m::kSnoreOut;
using m::kMotionHid;

// 优先 PSRAM，其次内部 RAM
static float* AllocF32(size_t count) {
    float* p = (float*)heap_caps_malloc(count * sizeof(float),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == nullptr) {
        p = (float*)heap_caps_malloc(count * sizeof(float), MALLOC_CAP_8BIT);
    }
    return p;
}

// ===========================================================================
// SnoreNet
// ===========================================================================
SnoreNet* SnoreNet::GetInstance() {
    static SnoreNet inst;
    return &inst;
}

esp_err_t SnoreNet::Init() {
    ready_ = true;
    return ESP_OK;
}

void SnoreNet::Forward(const float* features, float* logits_out) const {
    float h0[kSnoreH0];
    float h1[kSnoreH1];
    float xn[kSnoreIn];
    for (int i = 0; i < kSnoreIn; ++i) {
        xn[i] = (features[i] - m::g_snore_mean[i]) / (m::g_snore_std[i] + 1e-6f);
    }
    nn::Fc(xn, kSnoreIn, kSnoreH0, m::g_snore_fc0_q, m::g_snore_fc0_scale,
           m::g_snore_fc0_zp, m::g_snore_fc0_b, h0);
    nn::Relu(h0, kSnoreH0);
    nn::Fc(h0, kSnoreH0, kSnoreH1, m::g_snore_fc1_q, m::g_snore_fc1_scale,
           m::g_snore_fc1_zp, m::g_snore_fc1_b, h1);
    nn::Relu(h1, kSnoreH1);
    nn::Fc(h1, kSnoreH1, kSnoreOut, m::g_snore_fc2_q, m::g_snore_fc2_scale,
           m::g_snore_fc2_zp, m::g_snore_fc2_b, logits_out);
}

float SnoreNet::SelfTest() const {
    float out[kSnoreOut];
    Forward(m::g_snore_test_in, out);
    float maxe = 0.0f;
    for (int i = 0; i < kSnoreOut; ++i) {
        maxe = fmaxf(maxe, fabsf(out[i] - m::g_snore_test_out[i]));
    }
    return maxe;
}

// ===========================================================================
// StageNet：可分离卷积前端(3 块) + 双向 GRU + 门控融合 + 全局平均 + LN + MLP 头
// ===========================================================================
StageNet* StageNet::GetInstance() {
    static StageNet inst;
    return &inst;
}

esp_err_t StageNet::Init() {
    if (ready_) return ESP_OK;
    // 前端各段输出：(4,180) (16,180) (16,60) (24,60) (24,30) (24,30)
    // + GRU 输入/输出 (30,24)/(30,48) + 门控/融合 (30,24)x2 + 头 48
    const size_t n = 4 * 180 + 16 * 180 + 16 * 60 + 24 * 60 + 24 * 30 + 24 * 30
                   + 30 * 24 + 30 * 48 + 30 * 24 + 30 * 24 + 48;
    buf_ = AllocF32(n);
    if (buf_ == nullptr) return ESP_ERR_NO_MEM;
    buf_bytes_ = n * sizeof(float);
    ready_ = true;
    ESP_LOGI("StageNet", "init ok, scratch=%u B", (unsigned)buf_bytes_);
    return ESP_OK;
}

void StageNet::Deinit() {
    if (buf_) { heap_caps_free(buf_); buf_ = nullptr; }
    ready_ = false;
}

esp_err_t StageNet::Forward(const float* x, float* logits_out, float* probs_out) const {
    if (!ready_) return ESP_ERR_INVALID_STATE;
    float* p = buf_;
    float* d0 = p; p += 4 * 180;
    float* d1 = p; p += 16 * 180;
    float* d2 = p; p += 16 * 60;
    float* d3 = p; p += 24 * 60;
    float* d4 = p; p += 24 * 30;
    float* d5 = p; p += 24 * 30;
    float* gx = p; p += 30 * 24;
    float* go = p; p += 30 * 48;
    float* gt = p; p += 30 * 24;
    float* fu = p; p += 30 * 24;
    float* hd = p;

    // 可分离卷积前端（DWConv + BN/GELU 已折叠）：
    nn::Conv1d(x,  4, 360, m::g_stage_dw0_q, m::g_stage_dw0_scale, m::g_stage_dw0_b, 4, 5, 2, 2, 4, d0);
    for (int i = 0; i < 4 * 180; ++i) d0[i] = nn::Gelu(d0[i]);
    nn::Conv1d(d0, 4, 180, m::g_stage_pw0_q, m::g_stage_pw0_scale, m::g_stage_pw0_b, 16, 1, 1, 0, 1, d1);
    for (int i = 0; i < 16 * 180; ++i) d1[i] = nn::Gelu(d1[i]);
    nn::Conv1d(d1, 16, 180, m::g_stage_dw1_q, m::g_stage_dw1_scale, m::g_stage_dw1_b, 16, 3, 3, 1, 16, d2);
    for (int i = 0; i < 16 * 60; ++i) d2[i] = nn::Gelu(d2[i]);
    nn::Conv1d(d2, 16, 60, m::g_stage_pw1_q, m::g_stage_pw1_scale, m::g_stage_pw1_b, 24, 1, 1, 0, 1, d3);
    for (int i = 0; i < 24 * 60; ++i) d3[i] = nn::Gelu(d3[i]);
    nn::Conv1d(d3, 24, 60, m::g_stage_dw2_q, m::g_stage_dw2_scale, m::g_stage_dw2_b, 24, 3, 2, 1, 24, d4);
    for (int i = 0; i < 24 * 30; ++i) d4[i] = nn::Gelu(d4[i]);
    nn::Conv1d(d4, 24, 30, m::g_stage_pw2_q, m::g_stage_pw2_scale, m::g_stage_pw2_b, 24, 1, 1, 0, 1, d5);
    for (int i = 0; i < 24 * 30; ++i) d5[i] = nn::Gelu(d5[i]);

    // (24,30) -> [T=30][24] 作为 GRU 输入
    for (int t = 0; t < 30; ++t) {
        for (int c = 0; c < 24; ++c) {
            gx[t * 24 + c] = d5[c * 30 + t];
        }
    }
    nn::GruBidir(gx, 30, 24, m::g_stage_gru_w_q, m::g_stage_gru_w_scale,
                 m::g_stage_gru_r_q, m::g_stage_gru_r_scale,
                 m::g_stage_gru_b, 24, go);

    // 门控融合：gt = sigmoid(W_gate * [hf;hb]), fused = gt*hf + (1-gt)*hb
    for (int t = 0; t < 30; ++t) {
        nn::Fc(go + t * 48, 48, 24, m::g_stage_gate_w_q, m::g_stage_gate_w_scale,
               0, m::g_stage_gate_b, gt + t * 24);
        for (int j = 0; j < 24; ++j) {
            const float g = nn::Sigmoid(gt[t * 24 + j]);
            const float hf = go[t * 48 + j];
            const float hb = go[t * 48 + 24 + j];
            fu[t * 24 + j] = g * hf + (1.0f - g) * hb;
        }
    }
    // 全局平均池化
    float pooled[24] = {0};
    for (int t = 0; t < 30; ++t) {
        for (int j = 0; j < 24; ++j) pooled[j] += fu[t * 24 + j];
    }
    for (int j = 0; j < 24; ++j) pooled[j] /= 30.0f;

    // 头：LayerNorm -> FC(24->48) -> GELU -> FC(48->3)
    float ln[24];
    nn::LayerNorm(pooled, 24, m::g_stage_ln_w, m::g_stage_ln_b, ln);
    nn::Fc(ln, 24, 48, m::g_stage_fc0_q, m::g_stage_fc0_scale, 0, m::g_stage_fc0_b, hd);
    for (int i = 0; i < 48; ++i) hd[i] = nn::Gelu(hd[i]);
    nn::Fc(hd, 48, kCls, m::g_stage_fc1_q, m::g_stage_fc1_scale, 0, m::g_stage_fc1_b, logits_out);
    if (probs_out) nn::Softmax(logits_out, kCls, probs_out);
    return ESP_OK;
}

float StageNet::SelfTest() const {
    float logits[kCls];
    if (Forward(m::g_stage_test_in, logits, nullptr) != ESP_OK) return -1.0f;
    float maxe = 0.0f;
    for (int i = 0; i < kCls; ++i) maxe = fmaxf(maxe, fabsf(logits[i] - m::g_stage_test_out[i]));
    return maxe;
}

// ===========================================================================
// MotionNet：3 组卷积(7/5/1) + BN + 双向 GRU + MLP 头，逐窗 3 类
// ===========================================================================
MotionNet* MotionNet::GetInstance() {
    static MotionNet inst;
    return &inst;
}

esp_err_t MotionNet::Init() {
    if (ready_) return ESP_OK;
    // concat[144][T] + 输入转置[14][T] + GRU 输入[T][144] + GRU 输出[T][96]
    const size_t n = (size_t)3 * kMotionHid * kMaxT + (size_t)kIn * kMaxT
                   + (size_t)kMaxT * 3 * kMotionHid + (size_t)kMaxT * 2 * kMotionHid;
    buf_ = AllocF32(n);
    if (buf_ == nullptr) return ESP_ERR_NO_MEM;
    buf_bytes_ = n * sizeof(float);
    ready_ = true;
    ESP_LOGI("MotionNet", "init ok, scratch=%u B (PSRAM)", (unsigned)buf_bytes_);
    return ESP_OK;
}

void MotionNet::Deinit() {
    if (buf_) { heap_caps_free(buf_); buf_ = nullptr; }
    ready_ = false;
}

esp_err_t MotionNet::Forward(const float* feat, int T, float* logits_out, float* probs_out) const {
    if (!ready_ || T <= 0 || T > kMaxT) return ESP_ERR_INVALID_ARG;
    float* cat = buf_;                                   // [144][T]
    float* xin = buf_ + (size_t)3 * kMotionHid * kMaxT;  // [14][T]
    float* gx = xin + (size_t)kIn * kMaxT;               // [T][144]
    float* go = gx + (size_t)T * 3 * kMotionHid;         // [T][96]

    // feat [T][14] -> xin[14][T]
    for (int t = 0; t < T; ++t) {
        for (int c = 0; c < kIn; ++c) xin[(size_t)c * kMaxT + t] = feat[t * kIn + c];
    }

    // 三条卷积分支直接写入 cat 的三个通道段（每段 48 通道）
    nn::Conv1d(xin, kIn, T, m::g_motion_c7_q, m::g_motion_c7_scale, m::g_motion_c7_b,
               kMotionHid, 7, 1, 3, 1, cat + 0 * kMotionHid * T);
    nn::Conv1d(xin, kIn, T, m::g_motion_c5_q, m::g_motion_c5_scale, m::g_motion_c5_b,
               kMotionHid, 5, 1, 2, 1, cat + 1 * kMotionHid * T);
    nn::Conv1d(xin, kIn, T, m::g_motion_c1_q, m::g_motion_c1_scale, m::g_motion_c1_b,
               kMotionHid, 1, 1, 0, 1, cat + 2 * kMotionHid * T);
    const int ch = 3 * kMotionHid;
    for (int c = 0; c < ch; ++c) {
        float* row = cat + (size_t)c * T;
        for (int t = 0; t < T; ++t) if (row[t] < 0.0f) row[t] = 0.0f;   // ReLU
        // BatchNorm（推理态，ONNX 导出 epsilon=0）
        const float inv = 1.0f / sqrtf(m::g_motion_bn_v[c]);
        const float a = m::g_motion_bn_w[c] * inv;
        const float b = m::g_motion_bn_b[c] - m::g_motion_bn_m[c] * a;
        for (int t = 0; t < T; ++t) row[t] = row[t] * a + b;
    }

    // [144][T] -> [T][144]
    for (int t = 0; t < T; ++t) {
        for (int c = 0; c < ch; ++c) {
            gx[t * ch + c] = cat[(size_t)c * T + t];
        }
    }
    nn::GruBidir(gx, T, ch, m::g_motion_gru_w_q, m::g_motion_gru_w_scale,
                 m::g_motion_gru_r_q, m::g_motion_gru_r_scale,
                 m::g_motion_gru_b, kMotionHid, go);

    float h[48];
    for (int t = 0; t < T; ++t) {
        nn::Fc(go + t * 2 * kMotionHid, 2 * kMotionHid, kMotionHid,
               m::g_motion_fc0_q, m::g_motion_fc0_scale, 0, m::g_motion_fc0_b, h);
        nn::Relu(h, kMotionHid);
        nn::Fc(h, kMotionHid, kCls, m::g_motion_fc1_q, m::g_motion_fc1_scale, 0,
               m::g_motion_fc1_b, logits_out + t * kCls);
        if (probs_out) nn::Softmax(logits_out + t * kCls, kCls, probs_out + t * kCls);
    }
    return ESP_OK;
}

float MotionNet::SelfTest() const {
    float logits[kMaxT * kCls];
    if (Forward(m::g_motion_test_in, kMaxT, logits, nullptr) != ESP_OK) return -1.0f;
    float maxe = 0.0f;
    for (int i = 0; i < kMaxT * kCls; ++i) {
        maxe = fmaxf(maxe, fabsf(logits[i] - m::g_motion_test_out[i]));
    }
    return maxe;
}

}  // namespace sleep_pillow
