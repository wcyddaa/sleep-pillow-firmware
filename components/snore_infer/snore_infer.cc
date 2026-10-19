// snore_infer.cc - 鼾声检测：64 维手工特征前端 + int8 MLP + 事件聚合
// ============================================================================
// 前端严格对齐 sleep-train/src/snoring_cnn.py::extract_handcrafted_features：
//   512 点 FFT(hop 160, 周期 hann, 两端各补 256 零 → 101 帧)
//   → 功率谱(257) → 26 通道 mel → log10(+1e-10) → DCT-II(ortho) 前 13 阶 MFCC
//   → ΔMFCC(前向差分, 首列复制)
//   → 谱质心/带宽/滚降(85%)/平坦度 + 逐帧 ZCR/RMS
//   → (MFCC13 + Δ13 + 6 类统计) 各取 mean/std = 64 维
// 分类器：int8 MLP（权重来自 snoring-classify 的 edge_snore_mlp_int8.onnx）
// ============================================================================
#include "snore_infer.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "model_nets.h"
#include "sleep_models.h"

namespace sleep_pillow {

namespace m = sleep_pillow::models;

static constexpr int kFftN   = 512;
static constexpr int kFftBin = 257;   // rfft 频点数
static constexpr int kHop    = 160;
static constexpr int kWin    = 512;
static constexpr int kPad    = 256;   // nperseg/2
static constexpr int kFrames = 101;   // 1s 音频的帧数
static constexpr float kPi   = 3.14159265358979f;

// 当输入不足 1 秒时按实际长度处理，最多 kFrames 帧
static inline float Clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

SnoreInfer* SnoreInfer::GetInstance() {
    static SnoreInfer instance;
    return &instance;
}

// PSRAM 优先、失败回退内部 SRAM 的通用分配
static float* AllocFloatPsram(size_t n) {
    float* p = (float*)heap_caps_malloc(n * sizeof(float),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == nullptr) {
        p = (float*)heap_caps_malloc(n * sizeof(float), MALLOC_CAP_8BIT);
    }
    return p;
}

esp_err_t SnoreInfer::Init() {
    if (initialized_) return ESP_OK;

    // FFT 旋转因子（exp(-j2πn/N)）与工作区
    tw_re_ = (float*)heap_caps_malloc(kFftN * sizeof(float), MALLOC_CAP_8BIT);
    tw_im_ = (float*)heap_caps_malloc(kFftN * sizeof(float), MALLOC_CAP_8BIT);
    fft_re_ = (float*)heap_caps_malloc(kFftN * sizeof(float), MALLOC_CAP_8BIT);
    fft_im_ = (float*)heap_caps_malloc(kFftN * sizeof(float), MALLOC_CAP_8BIT);
    if (!tw_re_ || !tw_im_ || !fft_re_ || !fft_im_) {
        ESP_LOGE(TAG, "FFT 工作区分配失败");
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < kFftN; ++i) {
        const float ang = -2.0f * kPi * (float)i / (float)kFftN;
        tw_re_[i] = cosf(ang);
        tw_im_[i] = sinf(ang);
    }

    // 把每帧任务栈上的大数组挪到 PSRAM：power[257]+mag[257]+feat[64]
    power_ = AllocFloatPsram((size_t)kFftBin);
    mag_   = AllocFloatPsram((size_t)kFftBin);
    feat_  = AllocFloatPsram(kSnoreFeatureDim);
    if (!power_ || !mag_ || !feat_) {
        ESP_LOGE(TAG, "PSRAM 缓冲分配失败（power=%p mag=%p feat=%p）", power_, mag_, feat_);
        if (power_) { heap_caps_free(power_); power_ = nullptr; }
        if (mag_)   { heap_caps_free(mag_);   mag_   = nullptr; }
        if (feat_)  { heap_caps_free(feat_);  feat_  = nullptr; }
        return ESP_ERR_NO_MEM;
    }

    if (SnoreNet::GetInstance()->Init() != ESP_OK) {
        return ESP_ERR_NO_MEM;
    }
    initialized_ = true;
    ESP_LOGI(TAG, "初始化完成: 帧长 %u 采样点 (%dHz), 特征 %u 维, int8 MLP, PSRAM buf=%u+%u+%uB",
             (unsigned)kSnoreFrameSamples, kSnoreSampleRate, (unsigned)kSnoreFeatureDim,
             (unsigned)((size_t)kFftBin * sizeof(float)),
             (unsigned)((size_t)kFftBin * sizeof(float)),
             (unsigned)(kSnoreFeatureDim * sizeof(float)));
    return ESP_OK;
}

esp_err_t SnoreInfer::Deinit() {
    if (tw_re_) { heap_caps_free(tw_re_); tw_re_ = nullptr; }
    if (tw_im_) { heap_caps_free(tw_im_); tw_im_ = nullptr; }
    if (fft_re_) { heap_caps_free(fft_re_); fft_re_ = nullptr; }
    if (fft_im_) { heap_caps_free(fft_im_); fft_im_ = nullptr; }
    if (power_)  { heap_caps_free(power_);  power_  = nullptr; }
    if (mag_)    { heap_caps_free(mag_);    mag_    = nullptr; }
    if (feat_)   { heap_caps_free(feat_);   feat_   = nullptr; }
    initialized_ = false;
    return ESP_OK;
}

void SnoreInfer::SetThreshold(float threshold) {
    if (threshold < 0.0f) threshold = 0.0f;
    if (threshold > 1.0f) threshold = 1.0f;
    threshold_ = threshold;
}

// 迭代 radix-2 FFT（原地，N=kFftN）
static void Fft512(float* re, float* im, const float* tw_re, const float* tw_im) {
    for (int i = 1, j = 0; i < kFftN; ++i) {
        int bit = kFftN >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    for (int len = 2; len <= kFftN; len <<= 1) {
        const int half = len >> 1;
        const int step = kFftN / len;
        for (int i = 0; i < kFftN; i += len) {
            for (int j = 0; j < half; ++j) {
                const int tw = j * step;
                const float wr = tw_re[tw], wi = tw_im[tw];
                const int a = i + j, b = a + half;
                const float tr = re[b] * wr - im[b] * wi;
                const float ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;  im[b] = im[a] - ti;
                re[a] += tr;         im[a] += ti;
            }
        }
    }
}

esp_err_t SnoreInfer::ExtractFeatures(const int16_t* pcm, size_t samples,
                                      float* out_features, size_t max_dims,
                                      size_t* out_dims) {
    if (pcm == nullptr || out_features == nullptr || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialized_) return ESP_ERR_INVALID_STATE;
    if (max_dims < kSnoreFeatureDim) return ESP_ERR_INVALID_ARG;

    const int n = (int)samples;
    // 帧数：与 scipy.signal.stft(len=kWin, hop=kHop, padded=True) 一致
    int frames = (n + 2 * kPad - kWin) / kHop + 1;
    if (frames < 1) frames = 1;
    if (frames > kFrames) frames = kFrames;

    // 累加器（population mean/std，与 numpy 默认 ddof=0 一致）
    float mfcc_sum[m::kSnoreMfcc] = {0}, mfcc_sq[m::kSnoreMfcc] = {0};
    float del_sum[m::kSnoreMfcc] = {0}, del_sq[m::kSnoreMfcc] = {0};
    float pub_sum[6] = {0}, pub_sq[6] = {0};   // centroid,bw,rolloff,flatness,zcr,rms

    float logmel[m::kSnoreMels];
    float mfcc[m::kSnoreMfcc];
    float mfcc_prev[m::kSnoreMfcc];
    bool has_prev = false;

    const float fmin_bin = (float)kSnoreSampleRate / (float)kFftN;  // 频率分辨率

    for (int t = 0; t < frames; ++t) {
        const int base = t * kHop - kPad;
        for (int i = 0; i < kFftN; ++i) {
            const int s = base + i;
            const float w = 0.5f * (1.0f - cosf(2.0f * kPi * (float)i / (float)kFftN));
            // ★ 量纲必须与训练端一致：
            //   训练端 load_audio() 只在「整数 WAV」时除以 info.max，
            //   而本模型的归一化常量 rms_mean≈1762 / mfcc0_mean≈-1.38
            //   证明实际训练输入是 **int16 原值量纲**（没做 0~1 归一化）。
            //   所以这里不能除 32768，否则 log-mel 整体下移 log10(32768²)≈9.03，
            //   会让 mfcc0 偏 ~1.9σ、rms 偏 3 个数量级。
            fft_re_[i] = (s >= 0 && s < n) ? ((float)pcm[s]) * w : 0.0f;
            fft_im_[i] = 0.0f;
        }
        Fft512(fft_re_, fft_im_, tw_re_, tw_im_);

        // 功率谱 / 幅度谱（0..256）—— 复用 Init 分配的 PSRAM 缓冲
        float* power = power_;
        float* mag   = mag_;
        for (int k = 0; k < kFftBin; ++k) {
            const float p = fft_re_[k] * fft_re_[k] + fft_im_[k] * fft_im_[k];
            power[k] = p;
            mag[k] = sqrtf(p);
        }

        // mel → log10 → MFCC
        const float* fb = m::g_snore_mel_fb;
        for (int i = 0; i < m::kSnoreMels; ++i) {
            float acc = 0.0f;
            const float* row = fb + (size_t)i * kFftBin;
            for (int k = 0; k < kFftBin; ++k) acc += row[k] * power[k];
            logmel[i] = log10f(acc + 1e-10f);
        }
        const float* dct = m::g_snore_dct;
        for (int i = 0; i < m::kSnoreMfcc; ++i) {
            float acc = 0.0f;
            const float* row = dct + (size_t)i * m::kSnoreMels;
            for (int j = 0; j < m::kSnoreMels; ++j) acc += row[j] * logmel[j];
            mfcc[i] = acc;
        }

        // 谱统计
        float total = 0.0f;
        for (int k = 0; k < kFftBin; ++k) total += mag[k];
        float eps = 1e-12f;
        float tot = total + eps;
        float centroid = 0.0f, bandwidth = 0.0f, logmean = 0.0f;
        for (int k = 0; k < kFftBin; ++k) {
            const float f = fmin_bin * (float)k;
            centroid += f * mag[k];
            logmean += logf(mag[k] + eps);
        }
        centroid /= tot;
        for (int k = 0; k < kFftBin; ++k) {
            const float d = fmin_bin * (float)k - centroid;
            bandwidth += d * d * mag[k];
        }
        bandwidth = sqrtf(bandwidth / tot);
        float flatness = expf(logmean / (float)kFftBin) / (total / (float)kFftBin + eps);
        // rolloff(85%)
        float rolloff = 0.0f;
        {
            float cum = 0.0f;
            for (int k = 0; k < kFftBin; ++k) {
                cum += mag[k];
                if (cum / tot >= 0.85f) { rolloff = fmin_bin * (float)k; break; }
            }
        }

        // 逐帧 ZCR / RMS（用未补零的原始波形切片，与训练端一致）
        //
        // ★ ZCR 必须逐字复刻训练端公式（sleep-train/src/snoring_cnn.py:327）：
        //     zcr = mean(|diff(sign(seg))|) / (2 * len(seg))
        //   展开后 = Σ|sign(v_i)-sign(v_{i-1})| / (len-1) / (2*len)
        //   旧实现写成 zc/(2*len)，比训练端**大 ~270 倍**；
        //   权重里 zcr_mean=0.000272、std=0.000208，喂 270 倍的值相当于偏离
        //   几百个标准差 → MLP 饱和 → 有无鼾声都被判成鼾声。
        float zcr = 0.0f, rms = 0.0f;
        {
            const int seg0 = t * kHop;
            int seglen = kWin;
            if (seg0 + seglen > n) seglen = n - seg0;
            if (seglen > 0) {
                float ss = 0.0f;
                int zc_sum = 0;          // Σ|sign(v_i)-sign(v_{i-1})|
                int prev_sign = 0;
                for (int i = 0; i < seglen; ++i) {
                    const int16_t v = pcm[seg0 + i];
                    const int cur_sign = (v > 0) ? 1 : (v < 0 ? -1 : 0);
                    if (i > 0) {
                        zc_sum += (cur_sign > prev_sign) ? (cur_sign - prev_sign)
                                                         : (prev_sign - cur_sign);
                    }
                    prev_sign = cur_sign;
                    // RMS 同样用 int16 量纲（训练端未归一化）
                    const float vf = (float)v;
                    ss += vf * vf;
                }
                if (seglen > 1) {
                    zcr = ((float)zc_sum / (float)(seglen - 1)) / (2.0f * (float)seglen);
                }
                rms = sqrtf(ss / (float)seglen);
            }
        }

        // ΔMFCC：前向差分，首帧复制（与 np.diff(prepend=首列) 一致）
        if (!has_prev) {
            memcpy(mfcc_prev, mfcc, sizeof(mfcc));
            has_prev = true;
        }
        float delta[m::kSnoreMfcc];
        for (int i = 0; i < m::kSnoreMfcc; ++i) {
            delta[i] = mfcc[i] - mfcc_prev[i];
            mfcc_prev[i] = mfcc[i];
        }

        // 累加
        for (int i = 0; i < m::kSnoreMfcc; ++i) {
            mfcc_sum[i] += mfcc[i];  mfcc_sq[i] += mfcc[i] * mfcc[i];
            del_sum[i]  += delta[i]; del_sq[i]  += delta[i] * delta[i];
        }
        const float pub[6] = {centroid, bandwidth, rolloff, flatness, zcr, rms};
        for (int i = 0; i < 6; ++i) {
            pub_sum[i] += pub[i];
            pub_sq[i] += pub[i] * pub[i];
        }
    }

    const float inv = 1.0f / (float)frames;
    size_t o = 0;
    for (int arr = 0; arr < 2; ++arr) {
        const float* s = (arr == 0) ? mfcc_sum : del_sum;
        const float* q = (arr == 0) ? mfcc_sq : del_sq;
        for (int i = 0; i < m::kSnoreMfcc; ++i) {
            const float mu = s[i] * inv;
            const float var = q[i] * inv - mu * mu;
            out_features[o++] = mu;
            out_features[o++] = sqrtf(var > 0.0f ? var : 0.0f);
        }
    }
    for (int i = 0; i < 6; ++i) {
        const float mu = pub_sum[i] * inv;
        const float var = pub_sq[i] * inv - mu * mu;
        out_features[o++] = mu;
        out_features[o++] = sqrtf(var > 0.0f ? var : 0.0f);
    }

    // ★ 诊断：把 64 维特征和训练集的 mean/std 逐维对比，找出偏离最大的一维。
    //   正常 |z| < ~4；若某维 |z| 很大，说明**输入与训练分布不一致**
    //   （量纲 / ZCR / 增益 / 接线问题），而不是"模型不行"。
    {
        float worst_z = 0.0f;
        int   worst_i = -1;
        for (int i = 0; i < (int)kSnoreFeatureDim; ++i) {
            const float sd = (m::g_snore_std[i] > 1e-6f) ? m::g_snore_std[i] : 1e-6f;
            const float z = (out_features[i] - m::g_snore_mean[i]) / sd;
            const float az = fabsf(z);
            if (az > worst_z) { worst_z = az; worst_i = i; }
        }
        if (worst_z > 6.0f && worst_i >= 0) {
            ESP_LOGW(TAG,
                     "特征偏出训练分布: 最大|z|=%.1f @dim%d (实测%.4f / 训练 %.4f±%.4f)",
                     (double)worst_z, worst_i,
                     (double)out_features[worst_i],
                     (double)m::g_snore_mean[worst_i],
                     (double)m::g_snore_std[worst_i]);
        }
    }

    if (out_dims != nullptr) *out_dims = kSnoreFeatureDim;
    return ESP_OK;
}

esp_err_t SnoreInfer::Predict(const int16_t* pcm, size_t samples, float* out_prob) {
    if (pcm == nullptr || out_prob == nullptr || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    // 复用 Init 分配的 PSRAM 缓冲；提取+前向 全程不踩任务栈大数组
    float* feat = feat_;
    size_t dims = 0;
    esp_err_t err = ExtractFeatures(pcm, samples, feat, kSnoreFeatureDim, &dims);
    if (err != ESP_OK) return err;

    float logits[SnoreNet::kOut];   // 仅 2 floats = 8B，留栈无压力
    SnoreNet::GetInstance()->Forward(feat, logits);
    // 二分类：softmax(logits)[1] = sigmoid(l0 - l1)
    const float p = 1.0f / (1.0f + expf(logits[0] - logits[1]));
    *out_prob = Clamp01(p);
    return ESP_OK;
}

esp_err_t SnoreInfer::PredictFrame(const int16_t* pcm, float* out_prob) {
    return Predict(pcm, kSnoreFrameSamples, out_prob);
}

esp_err_t SnoreInfer::PredictFull(const int16_t* pcm, size_t samples, SnoreResult* out) {
    if (out == nullptr) return ESP_ERR_INVALID_ARG;
    esp_err_t err = Predict(pcm, samples, &out->probability);
    if (err != ESP_OK) return err;
    out->is_snore = (out->probability >= threshold_);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// 事件聚合（流式实现，与 build_edge_snore.py::aggregate_events 等价）
// ---------------------------------------------------------------------------
void SnoreInfer::AccumulateStats() {
    const float hours = (stats_.windows > 0)
        ? (float)stats_.windows / 3600.0f : 0.0f;
    stats_.snore_ratio = (stats_.windows > 0)
        ? (float)stats_.snore_windows / (float)stats_.windows : 0.0f;
    stats_.snore_index_per_hour = (hours > 0.0f) ? (float)stats_.events / hours : 0.0f;
}

void SnoreInfer::CloseSegment(uint32_t start, uint32_t end) {
    const uint32_t len = end - start + 1;
    if (len < 2) return;                       // 去毛刺：min_on_windows=2
    if (has_last_seg_ && (start > last_seg_end_) &&
        (start - last_seg_end_ - 1) <= 2) {    // 合并：gap <= merge_gap_windows=2
        stats_.snore_windows += len;
        stats_.snore_duration_sec = (float)stats_.snore_windows;
        last_seg_end_ = end;
    } else {
        stats_.events += 1;
        stats_.snore_windows += len;
        stats_.snore_duration_sec = (float)stats_.snore_windows;
        last_seg_start_ = start;
        last_seg_end_ = end;
        has_last_seg_ = true;
    }
    AccumulateStats();
}

void SnoreInfer::UpdateAggregation(float prob) {
    // 平滑 3 窗（边界复制）
    p_ring_[0] = p_ring_[1];
    p_ring_[1] = p_ring_[2];
    p_ring_[2] = prob;
    if (ring_n_ < 3) ++ring_n_;
    float sum = 0.0f;
    if (ring_n_ == 1) {
        sum = p_ring_[2] * 3.0f;
    } else if (ring_n_ == 2) {
        sum = p_ring_[2] + p_ring_[1] + p_ring_[2];
    } else {
        sum = p_ring_[0] + p_ring_[1] + p_ring_[2];
    }
    const float s = sum / 3.0f;

    const uint32_t t = stats_.windows;
    stats_.windows += 1;

    // 滞回双阈值
    if (active_state_) {
        if (s <= 0.4f) active_state_ = false;
    } else {
        if (s >= 0.6f) active_state_ = true;
    }

    if (active_state_) {
        if (!in_run_) { in_run_ = true; run_start_ = t; }
    } else if (in_run_) {
        in_run_ = false;
        CloseSegment(run_start_, t - 1);
    }
    AccumulateStats();
}

void SnoreInfer::FlushAggregation() {
    if (in_run_ && stats_.windows > 0) {
        CloseSegment(run_start_, stats_.windows - 1);
        in_run_ = false;
    }
}

float SnoreInfer::ModelSelfTest() {
    return SnoreNet::GetInstance()->SelfTest();
}

}  // namespace sleep_pillow
