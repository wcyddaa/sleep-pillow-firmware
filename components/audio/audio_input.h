// audio_input.h - I2S 麦克风采集链路（单例）
// ------------------------------------------------------------
// 数据流：
//   codec(I2C 配置) → I2S RX DMA → 采集任务 → 环形缓冲 → Read()/ReadFrame1s()
//
// 采集任务以 AUDIO_DMA_FRAME_NUM 为块大小持续拉取 I2S 数据，
// 写入 AudioRingBuffer；上层（snore_infer）按 1 秒帧读取。
//
// 注意：采集任务优先级低于雷达任务，且只有 main 在
//       EDGE_DATA_MODE == EDGE_MODE_REAL 时才创建（见 main/app_main.cpp）。
// ------------------------------------------------------------
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_codec.h"
#include "audio_ring_buffer.h"

namespace sleep_pillow {

class AudioInput {
public:
    static AudioInput* GetInstance();

    // 使用板级宏 AUDIO_CODEC_SELECT 指定的默认 codec 初始化
    esp_err_t Init();

    // 使用外部提供的 codec 初始化（内部不负责释放）
    esp_err_t Init(AudioCodec* codec);

    // 反初始化：停止采集、释放 codec 与缓冲区
    esp_err_t Deinit();

    // 启动采集任务（会先调用 codec->StartRecording()）
    esp_err_t Start();

    // 停止采集任务（会调用 codec->StopRecording()）
    esp_err_t Stop();

    // 阻塞读取 frames 帧 PCM（int16），返回实际读取帧数
    // timeout_ms = portMAX_DELAY 表示一直等待
    size_t Read(int16_t* dst, size_t frames, uint32_t timeout_ms);

    // 读取一整帧（AUDIO_FRAME_SAMPLES = 1 秒）音频，返回实际帧数
    // 不足 1 秒（超时）时返回已读取的数量，调用方需判断
    size_t ReadFrame1s(int16_t* dst, uint32_t timeout_ms);

    // 环形缓冲当前可读帧数
    size_t Available() const;

    // 累计溢出丢弃帧数
    size_t OverrunFrames() const;

    bool IsRunning() const { return running_; }

    AudioCodec* GetCodec() { return codec_; }

    AudioRingBuffer& GetRingBuffer() { return ring_; }

private:
    AudioInput() = default;
    ~AudioInput() = default;
    AudioInput(const AudioInput&) = delete;
    AudioInput& operator=(const AudioInput&) = delete;

    static void CaptureTaskEntry(void* arg);
    void CaptureTask();

    AudioCodec*      codec_ = nullptr;
    bool             owns_codec_ = false;
    AudioRingBuffer  ring_;

    TaskHandle_t     task_ = nullptr;
    volatile bool    running_ = false;

    static constexpr const char* TAG = "AudioInput";
    static constexpr uint32_t kTaskStackSize   = 4096;
    static constexpr UBaseType_t kTaskPriority = 3;      // 低于雷达任务(4/5)
    static constexpr uint32_t kReadTimeoutMs   = 200;    // 单次 I2S 读取超时
    static constexpr uint32_t kPollIntervalMs  = 5;      // Read() 轮询间隔
};

}  // namespace sleep_pillow
