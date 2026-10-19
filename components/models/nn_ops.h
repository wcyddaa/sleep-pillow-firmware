// nn_ops.h - 端侧最小推理算子（int8 权重 + float 激活，按需反量化）
// ============================================================================
// 只用 ESP-IDF 自带数学库，不依赖 external 组件。权重以 per-tensor 对称 int8 存放，
// 计算时逐元素反量化后做 float 乘加；shape 布局约定见各函数注释。
// ============================================================================
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace sleep_pillow {
namespace nn {

float Sigmoid(float x);
float Gelu(float x);

// 全连接：y[o] = scale * sum_i (q[i*out + o] - zp) * x[i] + bias[o]
// q 布局 [in][out]（行优先）
void Fc(const float* x, int in, int out,
        const int8_t* q, float scale, int zp, const float* bias, float* y);

// 1D 卷积（支持 depthwise/分组）：x [cin][lin]，q 布局 [cout][cin/groups][k]
// y [cout][lout]，lout = (lin + 2*pad - k) / stride + 1
void Conv1d(const float* x, int cin, int lin,
            const int8_t* q, float scale, const float* bias,
            int cout, int k, int stride, int pad, int groups, float* y);

// 双向 GRU（ONNX linear_before_reset=1 语义，门序 zrh）
// x [T][in]；W q 布局 [d][3H][in]；R q 布局 [d][3H][H]；b [d][2*3H]（b_ih 后 b_hh）
// out [T][2H]，前 H 维为正向、后 H 维为反向
void GruBidir(const float* x, int T, int in,
              const int8_t* wq, float ws, const int8_t* rq, float rs,
              const float* b, int hid, float* out);

// LayerNorm（最后一维）
void LayerNorm(const float* x, int n, const float* w, const float* bias, float* y);

// 就地 ReLU
void Relu(float* x, int n);

int  ArgMax(const float* x, int n);
void Softmax(const float* x, int n, float* y);

}  // namespace nn
}  // namespace sleep_pillow
