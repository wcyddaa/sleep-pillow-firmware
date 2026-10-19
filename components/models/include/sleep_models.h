// sleep_models.h - 三个端侧模型的 int8 权重与元数据声明
// ============================================================================
// 由 components/models/tools/export_onnx_to_c.py 从训练工程的 ONNX 自动生成/导出：
//   snore  : snoring-classify/outputs/edge_snore_mlp_int8.onnx  (MLP 64->48->24->2)
//   stage  : PSG-classify/outputs/model_stage_distill.onnx      (EdgeBiGatedNet 4x360->3, 双向堆叠蒸馏学生)
//   motion : BCG-classify/outputs/motion_bigru_fcn.onnx         (BiGRU-FCN 14xT->3)
//
// 量化方案：权重 per-tensor 对称 int8（snore 沿用 ONNX 导出时的 int8 权重 + scale/zp），
//           bias 保持 float32；推理时"int8 权重 + float 激活"按需反量化。
// ============================================================================
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace sleep_pillow {
namespace models {

// ---------------------------------------------------------------- 维度常量
inline constexpr int kSnoreIn   = 64;
inline constexpr int kSnoreH0   = 48;
inline constexpr int kSnoreH1   = 24;
inline constexpr int kSnoreOut  = 2;

inline constexpr int kStageCh   = 4;      // IHR / Resp / Motion / RespMask
inline constexpr int kStageLen  = 360;    // 3 个 30s epoch @4Hz
inline constexpr int kStageT    = 30;     // 前端下采样后的步数
inline constexpr int kStageHid  = 24;     // 双向 GRU 单向隐藏维
inline constexpr int kStageCls  = 3;      // Wake / Light / Deep

inline constexpr int kMotionIn  = 14;     // 多尺度体动特征数
inline constexpr int kMotionT   = 128;    // 固定 128 个 10s 窗
inline constexpr int kMotionHid = 48;
inline constexpr int kMotionCls = 3;      // rest / motion / bedempty

// ---------------------------------------------------------------- snore
extern const float  g_snore_mean[kSnoreIn];
extern const float  g_snore_std[kSnoreIn];
extern const int8_t g_snore_fc0_q[kSnoreIn * kSnoreH0];
extern const float  g_snore_fc0_scale;
extern const int    g_snore_fc0_zp;
extern const float  g_snore_fc0_b[kSnoreH0];
extern const int8_t g_snore_fc1_q[kSnoreH0 * kSnoreH1];
extern const float  g_snore_fc1_scale;
extern const int    g_snore_fc1_zp;
extern const float  g_snore_fc1_b[kSnoreH1];
extern const int8_t g_snore_fc2_q[kSnoreH1 * kSnoreOut];
extern const float  g_snore_fc2_scale;
extern const int    g_snore_fc2_zp;
extern const float  g_snore_fc2_b[kSnoreOut];

// ---------------------------------------------------------------- stage
extern const int8_t g_stage_dw0_q[4 * 1 * 5];      extern const float g_stage_dw0_scale; extern const float g_stage_dw0_b[4];
extern const int8_t g_stage_pw0_q[4 * 16];         extern const float g_stage_pw0_scale; extern const float g_stage_pw0_b[16];
extern const int8_t g_stage_dw1_q[16 * 1 * 3];     extern const float g_stage_dw1_scale; extern const float g_stage_dw1_b[16];
extern const int8_t g_stage_pw1_q[16 * 24];        extern const float g_stage_pw1_scale; extern const float g_stage_pw1_b[24];
extern const int8_t g_stage_dw2_q[24 * 1 * 3];     extern const float g_stage_dw2_scale; extern const float g_stage_dw2_b[24];
extern const int8_t g_stage_pw2_q[24 * 24];        extern const float g_stage_pw2_scale; extern const float g_stage_pw2_b[24];
extern const int8_t g_stage_gru_w_q[2 * 3 * kStageHid * kStageHid]; extern const float g_stage_gru_w_scale;
extern const int8_t g_stage_gru_r_q[2 * 3 * kStageHid * kStageHid]; extern const float g_stage_gru_r_scale;
extern const float  g_stage_gru_b[2 * 2 * 3 * kStageHid];
extern const int8_t g_stage_gate_w_q[2 * kStageHid * kStageHid];    extern const float g_stage_gate_w_scale;
extern const float  g_stage_gate_b[kStageHid];
extern const float  g_stage_ln_w[kStageHid];
extern const float  g_stage_ln_b[kStageHid];
extern const int8_t g_stage_fc0_q[kStageHid * (2 * kStageHid)];     extern const float g_stage_fc0_scale;
extern const float  g_stage_fc0_b[2 * kStageHid];
extern const int8_t g_stage_fc1_q[(2 * kStageHid) * kStageCls];     extern const float g_stage_fc1_scale;
extern const float  g_stage_fc1_b[kStageCls];

// ---------------------------------------------------------------- motion
extern const int8_t g_motion_c7_q[kMotionHid * kMotionIn * 7]; extern const float g_motion_c7_scale; extern const float g_motion_c7_b[kMotionHid];
extern const int8_t g_motion_c5_q[kMotionHid * kMotionIn * 5]; extern const float g_motion_c5_scale; extern const float g_motion_c5_b[kMotionHid];
extern const int8_t g_motion_c1_q[kMotionHid * kMotionIn * 1]; extern const float g_motion_c1_scale; extern const float g_motion_c1_b[kMotionHid];
extern const float  g_motion_bn_w[3 * kMotionHid];
extern const float  g_motion_bn_b[3 * kMotionHid];
extern const float  g_motion_bn_m[3 * kMotionHid];
extern const float  g_motion_bn_v[3 * kMotionHid];
extern const int8_t g_motion_gru_w_q[2 * 3 * kMotionHid * (3 * kMotionHid)]; extern const float g_motion_gru_w_scale;
extern const int8_t g_motion_gru_r_q[2 * 3 * kMotionHid * kMotionHid];       extern const float g_motion_gru_r_scale;
extern const float  g_motion_gru_b[2 * 2 * 3 * kMotionHid];
extern const int8_t g_motion_fc0_q[(2 * kMotionHid) * kMotionHid]; extern const float g_motion_fc0_scale; extern const float g_motion_fc0_b[kMotionHid];
extern const int8_t g_motion_fc1_q[kMotionHid * kMotionCls];       extern const float g_motion_fc1_scale; extern const float g_motion_fc1_b[kMotionCls];

// ---------------------------------------------------------------- snore 前端常量
// 26 通道三角 mel 滤波器组（26 x 257）与 13x26 DCT-II(ortho) 矩阵，
// 与 sleep-train/src/snoring_cnn.py 的 mel_filterbank/dct 一致（已用 scipy 校验）
inline constexpr int kSnoreMels = 26;
inline constexpr int kSnoreFftBins = 257;   // 512 点 FFT 的 rfft 频点数
inline constexpr int kSnoreMfcc = 13;
extern const float g_snore_mel_fb[kSnoreMels * kSnoreFftBins];
extern const float g_snore_dct[kSnoreMfcc * kSnoreMels];

// ---------------------------------------------------------------- 上电自检向量
// 用随机输入在 PC 上跑 onnxruntime 得到的期望输出，用于验证端侧算子实现
extern const float g_snore_test_in[kSnoreIn];
extern const float g_snore_test_out[kSnoreOut];
extern const float g_stage_test_in[kStageCh * kStageLen];
extern const float g_stage_test_out[kStageCls];
extern const float g_motion_test_in[kMotionT * kMotionIn];
extern const float g_motion_test_out[kMotionT * kMotionCls];

}  // namespace models
}  // namespace sleep_pillow
