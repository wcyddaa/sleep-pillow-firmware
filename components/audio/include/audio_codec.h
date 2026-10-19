#pragma once

#include <stdint.h>
#include <stddef.h>
#include <functional>
#include "esp_err.h"
#include "driver/i2s_std.h"

namespace sleep_pillow {

// ============================================================
// 音频错误码定义（基于 esp_err_t 扩展）
// ============================================================
// ESP_OK          成功
// ESP_FAIL        通用失败
// ESP_ERR_NO_MEM  内存不足
// ESP_ERR_INVALID_ARG   参数无效
// ESP_ERR_INVALID_STATE 状态无效（如未初始化）
// ESP_ERR_NOT_FOUND     设备未找到（I2C 无应答）
// ESP_ERR_TIMEOUT       超时
// ------------------------------------------------------------
// 自定义错误码区间：0xE000 ~ 0xE0FF
#define AUDIO_ERR_CODEC_NOT_INIT    ESP_ERR_INVALID_STATE  // Codec 未初始化
#define AUDIO_ERR_CODEC_NOT_FOUND   ESP_ERR_NOT_FOUND      // I2C 无应答
#define AUDIO_ERR_I2S_CONFIG        make_audio_err(0xE001) // I2S 配置失败
#define AUDIO_ERR_DMA_ALLOC         make_audio_err(0xE002) // DMA 缓冲区分配失败
#define AUDIO_ERR_NOT_RECORDING    make_audio_err(0xE003)  // 未在录音状态
#define AUDIO_ERR_NOT_PLAYING      make_audio_err(0xE004)  // 未在播放状态

// 辅助：构造 esp_err_t
static inline esp_err_t make_audio_err(int code) {
    return (esp_err_t)(0xE000 | code);
}

// ============================================================
// 音频数据回调函数类型
// ------------------------------------------------------------
// 当 I2S DMA 接收到音频数据时触发。
// 参数：
//   data   - 音频数据指针（PCM 16-bit 交错）
//   size   - 数据长度（字节）
//   ctx    - 用户上下文（注册时传入）
// ============================================================
using on_audio_data_cb_t = std::function<void(const uint8_t* data,
                                              size_t size,
                                              void* ctx)>;

// ============================================================
// DMA 缓冲区配置结构
// ============================================================
struct AudioDmaConfig {
    int dma_buf_count = 6;        // DMA 缓冲区块数
    int dma_buf_len   = 1024;     // 每块长度（字节）
    int dma_desc_num  = 6;        // DMA 描述符数量（与 buf_count 一致）
};

// ============================================================
// AudioCodec：音频编解码器抽象接口
// ------------------------------------------------------------
// 定义统一的音频 Codec 操作接口，ES8311 / ES7210 等具体
// 驱动实现此接口。上层通过 Board::GetAudioCodec() 获取实例。
//
// 数据流路径：
//   录音（ADC）：Mic → ES8311 ADC → I2S RX → DMA → 回调
//   播放（DAC）：WriteData() → I2S TX → DMA → ES8311 DAC → Speaker
// ============================================================
class AudioCodec {
public:
    virtual ~AudioCodec() = default;

    // -------- 生命周期 --------

    // 初始化 Codec（含 I2C 寄存器配置 + I2S 通道建立）
    virtual esp_err_t Init() = 0;

    // 反初始化，释放 I2S 通道与 DMA 资源
    virtual esp_err_t Deinit() = 0;

    // -------- 音量控制 --------

    // 设置音量（0-100，0 为静音）
    virtual esp_err_t SetVolume(int volume) = 0;

    // 获取当前音量（0-100）
    virtual int GetVolume() = 0;

    // -------- 录音控制（ADC 路径）--------

    // 开始录音，启动 I2S RX DMA 接收
    virtual esp_err_t StartRecording() = 0;

    // 停止录音，关闭 I2S RX DMA
    virtual esp_err_t StopRecording() = 0;

    // -------- 播放控制（DAC 路径）--------

    // 开始播放，启动 I2S TX DMA 发送
    virtual esp_err_t StartPlayback() = 0;

    // 停止播放，关闭 I2S TX DMA
    virtual esp_err_t StopPlayback() = 0;

    // 写入音频数据到播放缓冲区（阻塞，返回实际写入字节数）
    virtual size_t WriteData(const uint8_t* data, size_t size) = 0;

    // -------- 采样率配置 --------

    // 设置采样率（支持 16000 / 24000 / 48000 Hz）
    virtual esp_err_t SetSampleRate(int rate) = 0;

    // 获取当前采样率
    virtual int GetSampleRate() = 0;

    // -------- DMA 配置 --------

    // 配置 DMA 缓冲区参数
    virtual esp_err_t SetDmaConfig(const AudioDmaConfig& config) = 0;

    // 获取当前 DMA 配置
    virtual AudioDmaConfig GetDmaConfig() = 0;

    // -------- 回调注册 --------

    // 注册音频数据回调（用于唤醒词检测 / 语音流上传）
    virtual void SetAudioDataCallback(on_audio_data_cb_t callback, void* ctx) = 0;

    // -------- 状态查询 --------

    // 是否正在录音
    virtual bool IsRecording() = 0;

    // 是否正在播放
    virtual bool IsPlaying() = 0;

    // Codec 是否已初始化
    virtual bool IsInitialized() = 0;

    // -------- 设备信息 --------

    // 获取 Codec 名称（如 "ES8311"）
    virtual const char* GetCodecName() = 0;

    // 获取 Codec I2C 地址
    virtual uint8_t GetI2cAddress() = 0;
};

}  // namespace sleep_pillow
