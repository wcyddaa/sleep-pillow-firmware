// stage_infer.cc - 睡眠分期（EdgeBiGatedNet int8）
#include "stage_infer.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "model_nets.h"

namespace sleep_pillow {

static constexpr const char* TAG_ = "StageInfer";

// PSRAM 优先、失败回退内部 SRAM 的通用分配（与 motion_infer::ring_ 一致风格）
static float* AllocFloatPsram(size_t n) {
    float* p = (float*)heap_caps_malloc(n * sizeof(float),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == nullptr) {
        p = (float*)heap_caps_malloc(n * sizeof(float), MALLOC_CAP_8BIT);
    }
    return p;
}

StageInfer* StageInfer::GetInstance() {
    static StageInfer inst;
    return &inst;
}

esp_err_t StageInfer::Init() {
    if (initialized_) return ESP_OK;
    if (StageNet::GetInstance()->Init() != ESP_OK) {
        ESP_LOGE(TAG_, "StageNet 初始化失败（内存不足？）");
        return ESP_ERR_NO_MEM;
    }
    // 把推理用大数组从任务栈挪到 PSRAM，避免 stage_infer 任务栈溢出
    infer_buf_ = AllocFloatPsram((size_t)StageNet::kCh * (size_t)StageNet::kLen);
    build_pad_ = AllocFloatPsram((size_t)kStageSeqLen);
    if (infer_buf_ == nullptr || build_pad_ == nullptr) {
        ESP_LOGE(TAG_, "PSRAM 缓冲分配失败（infer_buf=%p, build_pad=%p）",
                 infer_buf_, build_pad_);
        if (infer_buf_) { heap_caps_free(infer_buf_); infer_buf_ = nullptr; }
        if (build_pad_) { heap_caps_free(build_pad_); build_pad_ = nullptr; }
        return ESP_ERR_NO_MEM;
    }
    memset(infer_buf_, 0, (size_t)StageNet::kCh * (size_t)StageNet::kLen * sizeof(float));
    memset(build_pad_, 0, (size_t)kStageSeqLen * sizeof(float));
    initialized_ = true;
    ESP_LOGI(TAG_, "初始化完成: 输入 %dx%d @%dHz (3x30s 上下文), 3 类, PSRAM buf=%u+%uB",
             StageNet::kCh, StageNet::kLen, kStageFs,
             (unsigned)((size_t)StageNet::kCh * (size_t)StageNet::kLen * sizeof(float)),
             (unsigned)((size_t)kStageSeqLen * sizeof(float)));
    return ESP_OK;
}

esp_err_t StageInfer::Deinit() {
    StageNet::GetInstance()->Deinit();
    if (infer_buf_) { heap_caps_free(infer_buf_); infer_buf_ = nullptr; }
    if (build_pad_) { heap_caps_free(build_pad_); build_pad_ = nullptr; }
    initialized_ = false;
    return ESP_OK;
}

esp_err_t StageInfer::Predict(const float* ch4x360, StageResult* out) {
    if (!initialized_ || ch4x360 == nullptr || out == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    float logits[StageNet::kCls];
    float probs[StageNet::kCls];
    esp_err_t err = StageNet::GetInstance()->Forward(ch4x360, logits, probs);
    if (err != ESP_OK) return err;
    int best = 0;
    for (int i = 1; i < StageNet::kCls; ++i) if (probs[i] > probs[best]) best = i;
    out->probs[0] = probs[0];
    out->probs[1] = probs[1];
    out->probs[2] = probs[2];
    out->stage = static_cast<SleepStage>(best);
    out->valid = true;
    return ESP_OK;
}

esp_err_t StageInfer::PredictFrom(const StageFeatureBuilder& builder, StageResult* out) {
    if (infer_buf_ == nullptr) return ESP_ERR_INVALID_STATE;
    float* x = infer_buf_;        // PSRAM 复用：4*360 floats = 5760B，原在任务栈
    builder.Build(x);
    return Predict(x, out);
}

float StageInfer::ModelSelfTest() {
    return StageNet::GetInstance()->SelfTest();
}

// ---------------------------------------------------------------------------
// StageFeatureBuilder
// ---------------------------------------------------------------------------
void StageFeatureBuilder::Push(float ihr, float resp, float motion) {
    if (filled_ < kStageSeqLen) {
        buf_[0][filled_] = ihr;
        buf_[1][filled_] = resp;
        buf_[2][filled_] = motion;
        buf_[3][filled_] = 1.0f;
        ++filled_;
    } else {
        for (int c = 0; c < 4; ++c) {
            memmove(&buf_[c][0], &buf_[c][1], sizeof(float) * (kStageSeqLen - 1));
        }
        buf_[0][kStageSeqLen - 1] = ihr;
        buf_[1][kStageSeqLen - 1] = resp;
        buf_[2][kStageSeqLen - 1] = motion;
        buf_[3][kStageSeqLen - 1] = 1.0f;
    }
}

void StageFeatureBuilder::Reset() {
    memset(buf_, 0, sizeof(buf_));
    filled_ = 0;
}

static void ZScoreSegment(const float* src, float* dst, int n) {
    float mean = 0.0f;
    for (int i = 0; i < n; ++i) mean += src[i];
    mean /= (float)n;
    float var = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float d = src[i] - mean;
        var += d * d;
    }
    var /= (float)n;
    const float inv = 1.0f / sqrtf(var + 1e-6f);
    for (int i = 0; i < n; ++i) dst[i] = (src[i] - mean) * inv;
}

void StageFeatureBuilder::Build(float* out_4x360) const {
    const int n = (filled_ > 0) ? filled_ : 1;
    // 不足 360 时左侧补 0，保持 360 长度
    const int pad = kStageSeqLen - n;
    // 借用 StageInfer 单例的 build_pad_ 作为 4 通道复用的 tmp 缓冲（PSRAM，4*1440B 复用）
    // 若 StageInfer 还未 Init（极少见，正常流程 Init 在前），回退到内部 SRAM 局部数组
    float local_tmp[kStageSeqLen];
    float* tmp = StageInfer::GetInstance()->GetBuildPad();
    if (tmp == nullptr) tmp = local_tmp;
    for (int c = 0; c < 4; ++c) {
        for (int i = 0; i < pad; ++i) tmp[i] = 0.0f;
        for (int i = 0; i < n; ++i) tmp[pad + i] = buf_[c][i];
        float* dst = out_4x360 + (size_t)c * kStageSeqLen;
        if (c == 0 || c == 1) {
            ZScoreSegment(tmp, dst, kStageSeqLen);          // IHR/Resp 整段 z-score
        } else if (c == 2) {
            // Motion：逐 30s epoch（120 点）z-score
            for (int e = 0; e < kStageCtxEpochs; ++e) {
                ZScoreSegment(tmp + e * kStagePerEpoch, dst + e * kStagePerEpoch,
                              kStagePerEpoch);
            }
        } else {
            for (int i = 0; i < kStageSeqLen; ++i) dst[i] = 1.0f;   // RespMask = 1
        }
    }
}

}  // namespace sleep_pillow
