// inmp441_mic.h - 双 INMP441 立体声 I2S 采集 → 单声道近场输出
// ============================================================
// 硬件：两个 INMP441 并联在同一组 I2S 总线上，只有 L/R 引脚不同。
//   总线引脚、32bit 槽宽、INMP441 对齐修正等底层细节由板级抽象层
//   封装（components/board/boards/board_sleep_pillow_v1.cc），
//   本组件只经 BoardMicInput 统一接口取交错立体声数据。
//
// 输出：单声道 16kHz PCM（与 snore_infer 的输入契约一致）
//
// 「近场选麦 + 降噪」策略（对应需求：立体声降噪、采 1.5m 内更近的那路）：
//   1) 每帧分别算左右声道 RMS
//   2) 两路能量接近（比值 <= kAvgRatio）或近乎静音 → 取**平均**
//      （两麦噪声不相关，平均等效降噪）
//   3) 一路明显更响（> kSwitchRatio 倍）→ 判为更靠近声源，取该路
//      带滞回（cur_pick_），避免边界帧来回翻转导致波形撕裂
//
// 注：不做额外滤波/增益。snore_infer 的手工特征前端是对齐训练集口径的，
//     在其前面插入自定义 DSP 会偏移输入分布、影响模型表现。
// ============================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// 板级抽象层统一硬件接口（板型提供实现，本组件只面向接口）
class BoardMicInput;

namespace sleep_pillow {

// 采集参数。注：**定义在命名空间层**，不能放进 Inmp441Mic 内部 ——
// 否则 Init() 的默认参数 `Config{}` 会因类尚未完整而编译失败。
// 只保留业务侧参数；总线/引脚/时序属于板级抽象层，不在此出现。
struct Inmp441MicConfig {
    int sample_rate = 16000;  // 与 snore_infer / kSnoreSampleRate 一致
    // 预留的最大单声道帧数（决定 PSRAM scratch 大小）。1 秒 @16k = 16000
    int max_mono_samples = 16000;

    // 双麦合成策略
    //   false（默认）= 两路平均 → 经典的「双麦降噪」，
    //                  并且**与训练端一致**（load_audio 对立体声就是 mean(axis=1)）
    //   true          = 近场选麦：一路明显更响时取该路（声源更近），接近时平均
    // 注：模型只会对「训练时的输入分布」可靠。除非确有需要，
    //     保持 false，把偏差变量降到最少。
    bool near_field_select = false;
};

// 麦克风自检结论
enum class MicHealth : int {
    kOk = 0,     // 正常
    kNoData,     // 读不到数据 / 采样全为 0
    kTooQuiet,   // 有数据但几乎静音
    kDcOffset,   // DC 偏置过大
    kClipping,   // 削顶失真
};
const char* MicHealthText(MicHealth h);

class Inmp441Mic {
public:
    static Inmp441Mic* GetInstance();

    // 经板型 BoardMicInput 打开采集（幂等）。失败返回错误码，可重试。
    esp_err_t Init(const Inmp441MicConfig& cfg = Inmp441MicConfig{});
    esp_err_t Deinit();
    bool IsReady() const { return ready_; }

    // 读 mono_samples 个**单声道**样本到 dst。
    // 内部按 mono_samples 个立体声帧读取，再经近场选麦合成单声道。
    // 返回实际写入的单声道样本数；< mono_samples 表示超时/数据不足。
    size_t ReadMono(int16_t* dst, size_t mono_samples, uint32_t timeout_ms);

    // 最近一帧的诊断信息（供日志/前端排查）
    float LastRmsLeft()  const { return last_rms_l_; }
    float LastRmsRight() const { return last_rms_r_; }
    // DC 偏置（原始信号的均值）。INMP441 常有明显 DC 偏置，
    // 偏大时会污染 FFT 的直流分量 → 特征偏移 → 模型误判。
    float LastDcLeft()   const { return last_dc_l_; }
    float LastDcRight()  const { return last_dc_r_; }
    // 峰值（原始信号 |x| 最大值），用于判断是否削顶/是否完全静音
    int   LastPeakLeft() const { return last_peak_l_; }
    int   LastPeakRight()const { return last_peak_r_; }
    // 本帧采用：0=左麦 1=右麦 2=左右平均 3=两路都不可用(静音)
    int   LastPick()     const { return last_pick_; }
    // ★ 逐声道质量评分：2=干净可用 1=削顶 0=无数据（没接电源/没焊好）
    int   LastScoreLeft()  const { return last_score_l_; }
    int   LastScoreRight() const { return last_score_r_; }
    // ★ 本帧近场降噪增益（0.0=完全抑制远场/底噪，1.0=近场全通），供日志排查
    float LastNearGain() const { return last_near_gain_; }
    // ★ 本帧被减掉的直流偏置（int16 量纲），用于确认 DC 去除是否生效
    int   LastDcRemoved() const { return last_dc_removed_; }
    // ★ 本帧前 8 个原始采样（左声道，未经任何处理）：用于判断波形是否连续。
    //   连续平滑变化 = 真实音频；大幅跳变(0/±32767 交替) = 位错位或纯噪声。
    const int16_t* LastRawHead() const { return last_raw_head_; }
    size_t ShortFrames() const { return short_frames_; }

    // ★ 自检：读 0.5 秒，判断麦克风是否真的在工作（接线/供电/增益）。
    //   rms_out / dc_out / peak_out 可选，回传实测统计量。
    MicHealth SelfTest(float* rms_out = nullptr, float* dc_out = nullptr,
                       int* peak_out = nullptr);

private:
    Inmp441Mic() = default;
    ~Inmp441Mic() = default;
    Inmp441Mic(const Inmp441Mic&) = delete;
    Inmp441Mic& operator=(const Inmp441Mic&) = delete;

    // 近场选麦 + 接近时平均，把交错立体声合成单声道
    //   ★ 入参是 int32_t：I2S 用 32bit 槽（INMP441 是 24bit 器件，
    //     16bit 槽装不下 1BCLK 延迟 + 24bit 数据，会声道混叠+位错位）
    void MixToMono(const int32_t* stereo, size_t frames, int16_t* mono);

    BoardMicInput*          input_ = nullptr;  // 板型提供的 I2S 采集实现（底层封装）
    Inmp441MicConfig        cfg_{};
    bool                    ready_ = false;

    int32_t* stereo_scratch_ = nullptr;  // PSRAM：缓存一个整块的交错立体声（32bit 槽）
    size_t   scratch_cap_    = 0;        // 可缓存的立体声帧数

    // 选麦滞回状态
    int    cur_pick_   = 0;              // 0=左 1=右
    float  last_rms_l_ = 0.0f;
    float  last_rms_r_ = 0.0f;
    float  last_dc_l_  = 0.0f;
    float  last_dc_r_  = 0.0f;
    int    last_peak_l_ = 0;
    int    last_peak_r_ = 0;
    int    last_pick_  = 0;
    int    last_score_l_ = 2;            // 逐声道质量：2=干净 1=削顶 0=无数据
    int    last_score_r_ = 2;
    float  last_near_gain_ = 1.0f;       // 本帧近场降噪增益
    int    last_dc_removed_ = 0;         // 本帧减掉的 DC 偏置
    int16_t last_raw_head_[8] = {0};     // 本帧前 8 个原始采样（左声道）
    size_t short_frames_ = 0;            // 累计"没读满"的帧数

    static constexpr const char* TAG = "Inmp441Mic";
    static constexpr float kSwitchRatio = 1.35f;  // 另一路要响这么多倍才切换
    static constexpr float kAvgRatio    = 1.15f;  // 两路在这个比例内 → 平均
    static constexpr float kSilenceRms  = 1.0f;   // 低于此视为静音，直接平均

    // ★ 近场降噪阈值（只保留 1.5~2 米内的声音）
    //   kNoiseFloorRms 以下 → 判为背景噪声/远场，直接抑制为 0
    //   kRefRms 以上       → 全增益（近场，正常说话 30~50cm 量级）
    //   两者之间           → 线性过渡（越远衰减越多）
    // 标定依据：INMP441 16bit 输出，安静室内底噪 RMS 约 30~100，
    //           1.5~2m 处轻声说话 RMS 约 300~800，
    //           50cm 内正常说话 RMS 约 1500~4000。
    static constexpr float kNoiseFloorRms = 250.0f;   // 远场/底噪门限
    static constexpr float kRefRms        = 1500.0f;  // 近场满增益门限

    // ★ 逐声道判活门限：
    //   kDeadRms  —— 低于此视为「没接电源/没焊好」，该路直接弃用。
    //                INMP441 正常工作时底噪 rms 就有几十，全 0 或个位数说明没供电。
    //   kClipPeak —— 峰值触到此值视为削顶失真；该路仍可用，但优先选另一路。
    static constexpr float kDeadRms  = 5.0f;
    static constexpr int   kClipPeak = 32700;

    // ★★ 近场降噪增益开关
    //
    //   false（默认）= 关闭。原因：模型训练时是从 WAV 直接读**原始幅度**，
    //                  没有做任何按幅度缩放。开启这个增益会让 rms 特征整体
    //                  偏小，落在训练分布之外 → 模型误判（实测安静时
    //                  rms=2% 仍被判定 prob=0.99 鼾声）。
    //
    //   true          = 开启（只保留近场声音，远场按幅度衰减）。
    //                   仅在确认模型对大幅衰减不敏感时使用。
    static constexpr bool kEnableNearFieldGain = false;

    // ★ 输入数字衰减（0.0~1.0）：用于压制「声源太近导致削顶」。
    //   1.0 = 不衰减（默认）；0.5 = 幅度减半（-6dB）；0.25 = -12dB。
    //   注意：削顶是**不可逆**的，衰减只能防止以后再削顶，救不回已经失真的帧。
    //   真正治本还是把麦克风挪远（30~50cm）或加硬件衰减电阻。
    static constexpr float kInputAtten = 1.0f;
};

}  // namespace sleep_pillow
