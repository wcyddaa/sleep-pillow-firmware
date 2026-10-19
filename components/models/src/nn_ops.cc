// nn_ops.cc - 端侧最小推理算子实现
#include "nn_ops.h"

#include <math.h>
#include <string.h>

namespace sleep_pillow {
namespace nn {

float Sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

float Gelu(float x) {
    // exact GELU: 0.5*x*(1+erf(x/sqrt(2)))
    return 0.5f * x * (1.0f + erff(x * 0.70710678f));
}

void Fc(const float* x, int in, int out,
        const int8_t* q, float scale, int zp, const float* bias, float* y) {
    for (int o = 0; o < out; ++o) {
        y[o] = 0.0f;
    }
    for (int i = 0; i < in; ++i) {
        const float xi = x[i];
        if (xi == 0.0f) continue;
        const int8_t* row = q + (size_t)i * out;
        for (int o = 0; o < out; ++o) {
            y[o] += (float)((int)row[o] - zp) * xi;
        }
    }
    for (int o = 0; o < out; ++o) {
        y[o] = y[o] * scale + bias[o];
    }
}

void Conv1d(const float* x, int cin, int lin,
            const int8_t* q, float scale, const float* bias,
            int cout, int k, int stride, int pad, int groups, float* y) {
    const int lout = (lin + 2 * pad - k) / stride + 1;
    const int cpg = cin / groups;      // 每组输入通道
    const int opg = cout / groups;     // 每组输出通道
    for (int o = 0; o < cout; ++o) {
        for (int t = 0; t < lout; ++t) {
            y[(size_t)o * lout + t] = 0.0f;
        }
    }
    for (int g = 0; g < groups; ++g) {
        for (int oc = 0; oc < opg; ++oc) {
            const int o = g * opg + oc;
            const int8_t* w = q + (size_t)o * cpg * k;
            float* yo = y + (size_t)o * lout;
            for (int ic = 0; ic < cpg; ++ic) {
                const float* xi = x + (size_t)(g * cpg + ic) * lin;
                const int8_t* wk = w + (size_t)ic * k;
                for (int kk = 0; kk < k; ++kk) {
                    const int qw = wk[kk];
                    if (qw == 0) continue;
                    const float wf = (float)qw;
                    const int base = -pad + kk;
                    for (int t = 0; t < lout; ++t) {
                        const int s = base + t * stride;
                        if (s < 0 || s >= lin) continue;
                        yo[t] += wf * xi[s];
                    }
                }
            }
        }
    }
    for (int o = 0; o < cout; ++o) {
        const float bo = bias[o];
        float* yo = y + (size_t)o * lout;
        for (int t = 0; t < lout; ++t) {
            yo[t] = yo[t] * scale + bo;
        }
    }
}

static void GruOneDir(const float* x, int T, int in,
                      const int8_t* wq, float ws, const int8_t* rq, float rs,
                      const float* b_ih, const float* b_hh, int hid,
                      int dir_out_offset, float* out, bool reverse) {
    const int H = hid;
    const int g3 = 3 * H;
    float h[96];
    float gi[192];
    float gh[192];
    memset(h, 0, sizeof(float) * H);

    for (int step = 0; step < T; ++step) {
        const int t = reverse ? (T - 1 - step) : step;
        const float* xt = x + (size_t)t * in;
        // gi[g] = sum_i W[g][i] * x[i] + b_ih[g]
        for (int g = 0; g < g3; ++g) {
            gi[g] = b_ih[g];
        }
        for (int i = 0; i < in; ++i) {
            const float xi = xt[i];
            if (xi == 0.0f) continue;
            for (int g = 0; g < g3; ++g) {
                gi[g] += (float)wq[(size_t)g * in + i] * xi;
            }
        }
        for (int g = 0; g < g3; ++g) gi[g] *= ws;
        // gh[g] = sum_h R[g][h] * h[h] + b_hh[g]
        for (int g = 0; g < g3; ++g) {
            float acc = b_hh[g];
            for (int j = 0; j < H; ++j) {
                acc += (float)rq[(size_t)g * H + j] * h[j];
            }
            gh[g] = acc * rs;
        }
        // z = sigmoid(gi_z + gh_z); r = sigmoid(gi_r + gh_r); n = tanh(gi_n + r*gh_n)
        for (int j = 0; j < H; ++j) {
            const float z = Sigmoid(gi[j] + gh[j]);
            const float r = Sigmoid(gi[H + j] + gh[H + j]);
            const float n = tanhf(gi[2 * H + j] + r * gh[2 * H + j]);
            h[j] = (1.0f - z) * n + z * h[j];
        }
        float* ot = out + (size_t)t * (2 * H) + dir_out_offset;
        memcpy(ot, h, sizeof(float) * H);
    }
}

void GruBidir(const float* x, int T, int in,
              const int8_t* wq, float ws, const int8_t* rq, float rs,
              const float* b, int hid, float* out) {
    const int g3 = 3 * hid;
    const size_t w_stride = (size_t)g3 * in;
    const size_t r_stride = (size_t)g3 * hid;
    const size_t b_stride = (size_t)2 * g3;
    // 正向
    GruOneDir(x, T, in, wq, ws, rq, rs,
              b, b + g3, hid, 0, out, false);
    // 反向
    GruOneDir(x, T, in, wq + w_stride, ws, rq + r_stride, rs,
              b + b_stride, b + b_stride + g3, hid, hid, out, true);
}

void LayerNorm(const float* x, int n, const float* w, const float* bias, float* y) {
    float mean = 0.0f;
    for (int i = 0; i < n; ++i) mean += x[i];
    mean /= (float)n;
    float var = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float d = x[i] - mean;
        var += d * d;
    }
    var /= (float)n;
    const float inv = 1.0f / sqrtf(var + 1e-5f);
    for (int i = 0; i < n; ++i) {
        y[i] = (x[i] - mean) * inv * w[i] + bias[i];
    }
}

void Relu(float* x, int n) {
    for (int i = 0; i < n; ++i) {
        if (x[i] < 0.0f) x[i] = 0.0f;
    }
}

int ArgMax(const float* x, int n) {
    int best = 0;
    for (int i = 1; i < n; ++i) {
        if (x[i] > x[best]) best = i;
    }
    return best;
}

void Softmax(const float* x, int n, float* y) {
    float mx = x[0];
    for (int i = 1; i < n; ++i) {
        if (x[i] > mx) mx = x[i];
    }
    float sum = 0.0f;
    for (int i = 0; i < n; ++i) {
        y[i] = expf(x[i] - mx);
        sum += y[i];
    }
    for (int i = 0; i < n; ++i) {
        y[i] /= sum;
    }
}

}  // namespace nn
}  // namespace sleep_pillow
