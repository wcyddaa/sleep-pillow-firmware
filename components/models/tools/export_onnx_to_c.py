#!/usr/bin/env python3
"""把三个端侧 ONNX 模型导出为 ESP32-S3 固件可用的 int8 C 数组。

只用 onnx + numpy + onnxruntime（不需要 tensorflow）：
  * 从 ONNX 里取出推理期参数（BN 已在训练侧折叠进 Conv / BatchNormalization 保留原样）
  * 用 numpy 复刻每个模型的 forward，并与 onnxruntime 比对，确认端侧算子语义
    （Conv/GELU/GRU/LayerNorm/门控/BN）完全一致
  * 以 per-tensor 对称 int8 + float32 scale 量化权重，bias 保持 float32
  * 生成 components/models/models/*.cc 权重数组 + include/sleep_models.h 元数据
  * 导出随机测试向量（输入 + 期望输出）供固件上电自检

用法（PowerShell）：
    python components/models/tools/export_onnx_to_c.py
只写 sleep-pillow 工程；三个训练工程只读。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import onnx

HERE = Path(__file__).resolve()
COMP = HERE.parents[1]                 # components/models
PROJ_ROOT = HERE.parents[5]            # projects/

OUT_MODELS = COMP / "models"
OUT_INCLUDE = COMP / "include"

SNORE_ONNX = PROJ_ROOT / "snoring-classify" / "outputs" / "edge_snore_mlp_int8.onnx"
# 用最好的端侧分期模型：双向堆叠蒸馏学生（EdgeBiGatedNet，结构/model_edge 一致，权重更优）
STAGE_ONNX = PROJ_ROOT / "PSG-classify" / "outputs" / "model_stage_distill.onnx"
MOTION_ONNX = PROJ_ROOT / "BCG-classify" / "outputs" / "motion_bigru_fcn.onnx"

MOTION_T = 128
SEED = 20261007


# ---------------------------------------------------------------------------
# ONNX 取值
# ---------------------------------------------------------------------------
def tensor_to_np(t):
    if t.raw_data:
        dt = onnx.helper.tensor_dtype_to_np_dtype(t.data_type)
        return np.frombuffer(t.raw_data, dtype=dt).reshape(list(t.dims))
    return onnx.numpy_helper.to_array(t)


class Graph:
    def __init__(self, path: Path):
        self.model = onnx.load(str(path))
        self.g = self.model.graph
        self.init: dict[str, np.ndarray] = {}
        for i in self.g.initializer:
            self.init[i.name] = np.asarray(tensor_to_np(i))
        for n in self.g.node:
            if n.op_type == "Constant":
                for a in n.attribute:
                    if a.name == "value" and a.type == onnx.AttributeProto.TENSOR:
                        self.init[n.output[0]] = np.asarray(onnx.numpy_helper.to_array(a.t))

    def __call__(self, name: str) -> np.ndarray:
        return np.asarray(self.init[name], dtype=np.float32)

    def raw(self, name: str) -> np.ndarray:
        return np.asarray(self.init[name])


# ---------------------------------------------------------------------------
# 量化 / C 生成
# ---------------------------------------------------------------------------
def quantize_int8(w: np.ndarray):
    w = np.asarray(w, dtype=np.float32)
    amax = float(np.max(np.abs(w))) if w.size else 0.0
    scale = (amax / 127.0) if amax > 0 else 1.0
    q = np.clip(np.round(w / scale), -127, 127).astype(np.int8).reshape(-1)
    return q, scale


def _f32lit(v) -> str:
    s = f"{float(v):.9g}"
    if ('e' not in s) and ('E' not in s) and ('.' not in s):
        s += ".0"
    return s + "f"


class CFile:
    def __init__(self, title: str):
        self.lines = [f"// {title}", "// 由 components/models/tools/export_onnx_to_c.py 自动生成，请勿手改",
                      '#include "sleep_models.h"',
                      "namespace sleep_pillow {", "namespace models {", ""]

    def weight(self, base: str, w: np.ndarray):
        q, s = quantize_int8(w)
        vals = ",".join(str(int(v)) for v in q)
        self.lines.append(f"alignas(16) const int8_t g_{base}_q[] = {{{vals}}};")
        self.lines.append(f"const float g_{base}_scale = {s:.10g}f;")
        return q.size

    def f32(self, base: str, a: np.ndarray):
        vals = ",".join(_f32lit(v) for v in np.asarray(a, np.float32).reshape(-1))
        self.lines.append(f"alignas(16) const float g_{base}[] = {{{vals}}};")

    def int8_zp(self, base: str, q: np.ndarray, scale: float, zp: int):
        vals = ",".join(str(int(v)) for v in np.asarray(q, np.int8).reshape(-1))
        self.lines.append(f"alignas(16) const int8_t g_{base}_q[] = {{{vals}}};")
        self.lines.append(f"const float g_{base}_scale = {scale:.10g}f;")
        self.lines.append(f"const int g_{base}_zp = {int(zp)};")

    def dump(self, path: Path):
        self.lines.append("")
        self.lines.append("}  // namespace models")
        self.lines.append("}  // namespace sleep_pillow")
        path.write_text("\n".join(self.lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# numpy 参考实现（与固件 C 同构）
# ---------------------------------------------------------------------------
def erf(x):
    return np.vectorize(math.erf)(x)


def gelu(x):
    return (0.5 * x * (1.0 + erf(x / math.sqrt(2.0)))).astype(np.float32)


def sigmoid(x):
    return (1.0 / (1.0 + np.exp(-x))).astype(np.float32)


def conv1d(x, w, b, stride, pad, groups):
    N, Cin, L = x.shape
    Cout, Cing, K = w.shape
    Lout = (L + 2 * pad - K) // stride + 1
    xp = np.pad(x, ((0, 0), (0, 0), (pad, pad)))
    out = np.zeros((N, Cout, Lout), np.float32)
    cpg, opg = Cin // groups, Cout // groups
    for g in range(groups):
        xs = xp[:, g * cpg:(g + 1) * cpg, :]
        for oc in range(opg):
            co = g * opg + oc
            acc = np.zeros((N, Lout), np.float32)
            for ic in range(cpg):
                for k in range(K):
                    acc = acc + xs[:, ic, k:k + stride * Lout:stride] * w[co, ic, k]
            out[:, co, :] = acc + b[co]
    return out


def layer_norm(x, w, b, eps=1e-5):
    mu = x.mean(-1, keepdims=True)
    var = x.var(-1, keepdims=True)
    return ((x - mu) / np.sqrt(var + eps) * w + b).astype(np.float32)


def onnx_gru(x, W, R, B, hidden):
    """x:(T,N,In) W,R:(D,3H,In)/(D,3H,H) B:(D,2*3H) -> (T,N,D*H)；ONNX 门序 zrh。"""
    if x.ndim == 2:
        x = x[:, None, :]
    D = W.shape[0]
    T, N = x.shape[0], x.shape[1]
    outs = []
    for d in range(D):
        Wd, Rd, Bd = W[d], R[d], B[d]
        b_ih, b_hh = Bd[:3 * hidden], Bd[3 * hidden:]
        h = np.zeros((N, hidden), np.float32)
        seq = np.zeros((T, N, hidden), np.float32)
        order = range(T) if d == 0 else range(T - 1, -1, -1)
        for t in order:
            gi = x[t] @ Wd.T + b_ih
            gh = h @ Rd.T + b_hh
            z = sigmoid(gi[:, :hidden] + gh[:, :hidden])
            r = sigmoid(gi[:, hidden:2 * hidden] + gh[:, hidden:2 * hidden])
            n = np.tanh(gi[:, 2 * hidden:] + r * gh[:, 2 * hidden:]).astype(np.float32)
            h = (1.0 - z) * n + z * h
            seq[t] = h
        outs.append(seq)
    return np.concatenate(outs, axis=2)


def snore_fc(g: Graph, i: int):
    """返回 (int8 权重 [in,out], scale, zero_point) —— 来自 int8 ONNX 的 MatMulInteger 权重。"""
    q = g.raw(f"net.{i}.weight_quantized").astype(np.int8)
    scale = float(g.raw(f"net.{i}.weight_scale").reshape(-1)[0])
    zp = int(g.raw(f"net.{i}.weight_zero_point").reshape(-1)[0])
    return q, scale, zp


def snore_ref(g: Graph, x):
    """x:(1,64) -> logits(1,2)；权重走 int8 + zero_point 反量化。"""
    h = (x - g("mean")) / (g("onnx::Div_18") + 1e-6)
    for i, relu in [(0, True), (3, True), (6, False)]:
        q, s, zp = snore_fc(g, i)
        w = (q.astype(np.float32) - zp) * s
        h = h @ w + g(f"net.{i}.bias")
        if relu:
            h = np.maximum(h, 0.0)
    return h.astype(np.float32)


def stage_ref(g: Graph, x):
    """x:(1,4,360) -> logits(1,3), gate(1,24)"""
    convs = [("front.0.0", 5, 2, 2, 4), ("front.0.3", 1, 1, 0, 1),
             ("front.1.0", 3, 3, 1, 16), ("front.1.3", 1, 1, 0, 1),
             ("front.2.0", 3, 2, 1, 24), ("front.2.3", 1, 1, 0, 1)]
    h = x
    for name, k, s, p, grp in convs:
        h = gelu(conv1d(h, g(name + ".weight"), g(name + ".weight_bias"), s, p, grp))
    h = np.transpose(h, (2, 0, 1))                      # (30,1,24)
    W, R, B = g("val_190"), g("val_191"), g("val_192")
    hid = W.shape[1] // 3
    out = onnx_gru(h, W, R, B, hid)                     # (30,1,48)
    Y = out[:, 0, :].reshape(30, 2, hid).transpose(1, 0, 2)[None]  # (1,2,30,hid)
    ob = Y.transpose(0, 2, 1, 3).reshape(1, 30, 2 * hid)           # (1,30,48)
    hf, hb = ob[..., :hid], ob[..., hid:]
    gg = sigmoid(ob @ g("val_231") + g("gate.bias"))               # (1,30,24)
    fused = gg * hf + (1.0 - gg) * hb
    pooled = fused.mean(axis=1)                                    # (1,24)
    y = layer_norm(pooled, g("head.0.weight"), g("head.0.bias"))
    y = gelu(y @ g("head.1.weight").T + g("head.1.bias"))
    y = y @ g("head.4.weight").T + g("head.4.bias")
    return y.astype(np.float32), gg.mean(axis=(0, 1)).astype(np.float32)


def motion_gru_w(g: Graph):
    """PyTorch GRU weight_ih (r,z,n) -> ONNX (z,r,h)，再拼两个方向。"""
    def reorder(wih):
        h = wih.shape[0] // 3
        return np.concatenate([wih[h:2 * h], wih[:h], wih[2 * h:]], axis=0)
    return np.stack([reorder(g("gru.weight_ih_l0")), reorder(g("gru.weight_ih_l0_reverse"))])


def motion_ref(g: Graph, x, T):
    """x:(1,T,14) -> logits(1,T,3)"""
    h = np.transpose(x, (0, 2, 1))
    branches = []
    for name, k, p in [("c7", 7, 3), ("c5", 5, 2), ("c1", 1, 0)]:
        c = conv1d(h, g(name + ".0.weight"), g(name + ".0.bias"), 1, p, 1)
        branches.append(np.maximum(c, 0.0))
    h = np.concatenate(branches, axis=1)
    # BatchNormalization（ONNX 导出 epsilon=0，推理态用 running stats）
    m = g("bn.running_mean")[None, :, None].astype(np.float32)
    v = g("bn.running_var")[None, :, None].astype(np.float32)
    h = ((h - m) / np.sqrt(v) * g("bn.weight")[None, :, None] + g("bn.bias")[None, :, None]).astype(np.float32)
    h = np.transpose(h, (0, 2, 1))                      # (1,T,144)
    W, R, B = motion_gru_w(g), g("val_107"), g("val_108")
    hid = R.shape[2]
    out = onnx_gru(h[0], W, R, B, hid)                  # (T,1,96)
    ys = []
    for b in range(out.shape[1]):
        y = out[:, b, :] @ g("val_124") + g("head.0.bias")
        y = np.maximum(y, 0.0)
        y = y @ g("val_126") + g("head.3.bias")
        ys.append(y)
    return np.stack(ys, axis=0).astype(np.float32)      # (B,T,3)


# ---------------------------------------------------------------------------
# 导出
# ---------------------------------------------------------------------------
def export_snore():
    g = Graph(SNORE_ONNX)
    c = CFile("鼾声分类 MLP (2 层隐藏 64->48->24->2) — snoring-classify edge_snore_mlp_int8")
    c.f32("snore_mean", g("mean"))
    c.f32("snore_std", g("onnx::Div_18"))
    for base, i in [("snore_fc0", 0), ("snore_fc1", 3), ("snore_fc2", 6)]:
        q, s, zp = snore_fc(g, i)
        c.int8_zp(base, q, s, zp)
        c.f32(base + "_b", g(f"net.{i}.bias"))
    rng = np.random.default_rng(SEED)
    x = rng.normal(0, 1, (1, 64)).astype(np.float32)
    y = snore_ref(g, x)
    c.f32("snore_test_in", x)
    c.f32("snore_test_out", y)
    c.dump(OUT_MODELS / "model_snore.cc")
    export_snore_fe()
    return x, y, "snore"


# --- 鼾声 64 维手工特征前端常量（与 sleep-train/src/snoring_cnn.py 一致）-----
SR, N_FFT, HOP, WIN, N_MELS_FE, N_MFCC = 16000, 512, 160, 512, 26, 13


def hz_to_mel(f):
    return 2595.0 * np.log10(1.0 + np.asarray(f) / 700.0)


def mel_to_hz(mv):
    return 700.0 * (10.0 ** (np.asarray(mv) / 2595.0) - 1.0)


def mel_filterbank(sr, n_fft, n_mels, fmin=0.0, fmax=None):
    fmax = fmax or sr / 2.0
    mel_points = np.linspace(hz_to_mel(fmin), hz_to_mel(fmax), n_mels + 2)
    hz_points = mel_to_hz(mel_points)
    bins = np.floor((n_fft + 1) * hz_points / sr).astype(int)
    fb = np.zeros((n_mels, n_fft // 2 + 1), np.float32)
    for i in range(n_mels):
        left, center, right = bins[i], bins[i + 1], bins[i + 2]
        if center > left:
            fb[i, left:center] = (np.arange(left, center) - left) / (center - left)
        if right > center:
            fb[i, center:right] = (right - np.arange(center, right)) / (right - center)
    return fb


def dct2_ortho_matrix(n_mfcc, n_mels):
    """scipy.fft.dct(type=2, norm='ortho') 的前 n_mfcc 行（行 x 输入）。"""
    n = np.arange(n_mels)
    k = np.arange(n_mfcc)[:, None]
    m = np.cos(np.pi * k * (2 * n[None, :] + 1) / (2 * n_mels)) * np.sqrt(2.0 / n_mels)
    m[0] *= 1.0 / np.sqrt(2.0)
    return m.astype(np.float32)


def _check_fe():
    """若 scipy 可用，校验 mel 滤波器组 / DCT 与训练端一致。"""
    try:
        from scipy.fft import dct
        from scipy.signal import stft  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        print("[fe] scipy 不可用，跳过前端常量校验:", exc)
        return
    fb = mel_filterbank(SR, N_FFT, N_MELS_FE)
    rng = np.random.default_rng(0)
    logmel = rng.normal(0, 1, (N_MELS_FE, 17)).astype(np.float32)
    ref = dct(logmel, type=2, axis=0, norm="ortho")[:N_MFCC]
    mine = dct2_ortho_matrix(N_MFCC, N_MELS_FE) @ logmel
    print(f"[fe] mel_fb{fb.shape} DCT max|diff|={float(np.max(np.abs(ref - mine))):.3e}")


def export_snore_fe():
    c = CFile("鼾声 64 维前端常量：26 通道 mel 滤波器组 + 13x26 DCT-II(ortho) 矩阵")
    c.f32("snore_mel_fb", mel_filterbank(SR, N_FFT, N_MELS_FE))
    c.f32("snore_dct", dct2_ortho_matrix(N_MFCC, N_MELS_FE))
    c.dump(OUT_MODELS / "model_snore_fe.cc")


def export_stage():
    g = Graph(STAGE_ONNX)
    c = CFile("睡眠分期 EdgeBiGatedNet (4x360 -> Wake/Light/Deep) — PSG-classify model_stage_distill（双向堆叠蒸馏）")
    for base, name in [("stage_dw0", "front.0.0"), ("stage_pw0", "front.0.3"),
                       ("stage_dw1", "front.1.0"), ("stage_pw1", "front.1.3"),
                       ("stage_dw2", "front.2.0"), ("stage_pw2", "front.2.3")]:
        c.weight(base, g(name + ".weight")); c.f32(base + "_b", g(name + ".weight_bias"))
    c.weight("stage_gru_w", g("val_190"))
    c.weight("stage_gru_r", g("val_191")); c.f32("stage_gru_b", g("val_192"))
    c.weight("stage_gate_w", g("val_231")); c.f32("stage_gate_b", g("gate.bias"))
    c.f32("stage_ln_w", g("head.0.weight")); c.f32("stage_ln_b", g("head.0.bias"))
    # head 是 Gemm(transB=1)，权重存成 [out,in]；转成 [in,out] 便于端侧统一 fc
    c.weight("stage_fc0", g("head.1.weight").T); c.f32("stage_fc0_b", g("head.1.bias"))
    c.weight("stage_fc1", g("head.4.weight").T); c.f32("stage_fc1_b", g("head.4.bias"))
    rng = np.random.default_rng(SEED + 1)
    x = rng.normal(0, 1, (1, 4, 360)).astype(np.float32)
    y, gate = stage_ref(g, x)
    c.f32("stage_test_in", x)
    c.f32("stage_test_out", y)
    c.dump(OUT_MODELS / "model_stage.cc")
    return x, y, "stage"


def export_motion():
    g = Graph(MOTION_ONNX)
    c = CFile("BCG 体动 BiGRU-FCN (T x 14 -> rest/motion/bedempty) — BCG-classify motion_bigru_fcn")
    for base, name in [("motion_c7", "c7"), ("motion_c5", "c5"), ("motion_c1", "c1")]:
        c.weight(base, g(name + ".0.weight")); c.f32(base + "_b", g(name + ".0.bias"))
    c.f32("motion_bn_w", g("bn.weight")); c.f32("motion_bn_b", g("bn.bias"))
    c.f32("motion_bn_m", g("bn.running_mean")); c.f32("motion_bn_v", g("bn.running_var"))
    c.weight("motion_gru_w", motion_gru_w(g))
    c.weight("motion_gru_r", g("val_107")); c.f32("motion_gru_b", g("val_108"))
    c.weight("motion_fc0", g("val_124")); c.f32("motion_fc0_b", g("head.0.bias"))
    c.weight("motion_fc1", g("val_126")); c.f32("motion_fc1_b", g("head.3.bias"))
    rng = np.random.default_rng(SEED + 2)
    x = rng.normal(0, 1, (1, MOTION_T, 14)).astype(np.float32)
    y = motion_ref(g, x, MOTION_T)
    c.f32("motion_test_in", x)
    c.f32("motion_test_out", y)
    c.dump(OUT_MODELS / "model_motion.cc")
    return x, y, "motion"


def verify(name, onnx_path, x, input_name, ref_fn):
    import onnxruntime as ort
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    got = sess.run(None, {input_name: x})
    mine = ref_fn()
    err = float(np.max(np.abs(got[0] - mine)))
    print(f"[verify] {name}: ORT out0{got[0].shape} vs numpy{np.asarray(mine).shape}  max|diff|={err:.3e}")
    return err


def main():
    OUT_MODELS.mkdir(parents=True, exist_ok=True)
    OUT_INCLUDE.mkdir(parents=True, exist_ok=True)
    _check_fe()
    errs = []

    xs, ys, _ = export_snore()
    # 固件走"int8 权重 + float 激活"（weight-only），与 fp32 ONNX 对齐校验
    snore32 = PROJ_ROOT / "snoring-classify" / "outputs" / "edge_snore_mlp.onnx"
    errs.append(verify("snore(w-only vs fp32)", snore32, xs, "features",
                       lambda: snore_ref(Graph(SNORE_ONNX), xs)))

    xst, yst, _ = export_stage()
    errs.append(verify("stage", STAGE_ONNX, xst, "input", lambda: stage_ref(Graph(STAGE_ONNX), xst)[0]))

    xm, ym, _ = export_motion()
    errs.append(verify("motion", MOTION_ONNX, xm, "feat", lambda: motion_ref(Graph(MOTION_ONNX), xm, MOTION_T)))

    print("[result] max|diff| =", errs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
