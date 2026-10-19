// audio_input.cc - I2S 麦克风采集链路实现
#include "audio_input.h"
#include "audio_board_config.h"
#include "es7210_codec.h"
#include "es8311_codec.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

namespace sleep_pillow {

// 按板级宏选择默认 codec
static AudioCodec* CreateDefaultCodec() {
#if (AUDIO_CODEC_SELECT == AUDIO_CODEC_TYPE_ES8311)
    return new ES8311Codec(ES8311_I2C_ADDR);
#else
    return new ES7210Codec(ES7210_I2C_ADDR);
#endif
}

AudioInput* AudioInput::GetInstance() {
    static AudioInput instance;
    return &instance;
}

esp_err_t AudioInput::Init() {
    return Init(CreateDefaultCodec());
}

esp_err_t AudioInput::Init(AudioCodec* codec) {
    if (codec == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (codec_ != nullptr) {
        ESP_LOGW(TAG, "已初始化，忽略重复调用");
        return ESP_OK;
    }

    codec_ = codec;
    // 若 codec 是本模块创建出来的（未初始化），记录所有权
    owns_codec_ = !codec->IsInitialized();

    esp_err_t err = codec_->Init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "codec 初始化失败: %s", esp_err_to_name(err));
        if (owns_codec_) {
            delete codec_;
            owns_codec_ = false;
        }
        codec_ = nullptr;
        return err;
    }

    AudioDmaConfig dma;
    dma.dma_buf_count = AUDIO_DMA_DESC_NUM;
    dma.dma_buf_len   = AUDIO_DMA_FRAME_NUM;
    dma.dma_desc_num  = AUDIO_DMA_DESC_NUM;
    codec_->SetDmaConfig(dma);
    codec_->SetSampleRate(AUDIO_SAMPLE_RATE_HZ);

    err = ring_.Init(AUDIO_RINGBUF_FRAMES);
    if (err != ESP_OK) {
        codec_->Deinit();
        codec_ = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "采集链路就绪: %s @%dHz (%d 声道, DMA %d x %d 帧)",
             codec_->GetCodecName(), codec_->GetSampleRate(),
             AUDIO_CHANNELS, AUDIO_DMA_DESC_NUM, AUDIO_DMA_FRAME_NUM);
    return ESP_OK;
}

esp_err_t AudioInput::Deinit() {
    Stop();

    if (codec_ != nullptr) {
        codec_->Deinit();
        if (owns_codec_) {
            delete codec_;
        }
        codec_ = nullptr;
        owns_codec_ = false;
    }
    ring_.Deinit();
    return ESP_OK;
}

esp_err_t AudioInput::Start() {
    if (codec_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (running_) {
        return ESP_OK;
    }

    esp_err_t err = codec_->StartRecording();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动录音失败: %s", esp_err_to_name(err));
        return err;
    }

    ring_.Clear();
    running_ = true;

    BaseType_t ret = xTaskCreate(CaptureTaskEntry, "audio_in",
                                 kTaskStackSize, this, kTaskPriority, &task_);
    if (ret != pdPASS) {
        running_ = false;
        codec_->StopRecording();
        task_ = nullptr;
        ESP_LOGE(TAG, "创建采集任务失败");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "音频采集任务已启动");
    return ESP_OK;
}

esp_err_t AudioInput::Stop() {
    if (!running_) {
        return ESP_OK;
    }

    running_ = false;

    // 等待采集任务自行退出
    for (int i = 0; i < 100 && task_ != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    task_ = nullptr;

    if (codec_ != nullptr) {
        codec_->StopRecording();
    }
    ESP_LOGI(TAG, "音频采集任务已停止 (累计溢出 %u 帧)",
             (unsigned)ring_.DroppedFrames());
    return ESP_OK;
}

void AudioInput::CaptureTaskEntry(void* arg) {
    static_cast<AudioInput*>(arg)->CaptureTask();
}

void AudioInput::CaptureTask() {
    const size_t chunk_frames = AUDIO_DMA_FRAME_NUM;
    int16_t* chunk = static_cast<int16_t*>(
        heap_caps_malloc(chunk_frames * sizeof(int16_t), MALLOC_CAP_8BIT));
    if (chunk == nullptr) {
        ESP_LOGE(TAG, "DMA 读取缓冲分配失败");
        running_ = false;
        task_ = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    while (running_) {
        size_t bytes = codec_->ReadData(reinterpret_cast<uint8_t*>(chunk),
                                        chunk_frames * sizeof(int16_t),
                                        kReadTimeoutMs);
        if (bytes == 0) {
            continue;   // 超时，继续等待下一块
        }

        size_t frames = bytes / sizeof(int16_t);
        ring_.Write(chunk, frames);
    }

    heap_caps_free(chunk);
    task_ = nullptr;
    vTaskDelete(nullptr);
}

size_t AudioInput::Read(int16_t* dst, size_t frames, uint32_t timeout_ms) {
    if (dst == nullptr || frames == 0) {
        return 0;
    }

    size_t got = 0;
    TickType_t start = xTaskGetTickCount();

    while (got < frames) {
        got += ring_.Read(dst + got, frames - got);
        if (got >= frames) {
            break;
        }

        if (timeout_ms != portMAX_DELAY) {
            uint32_t elapsed_ms =
                (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
            if (elapsed_ms >= timeout_ms) {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(kPollIntervalMs));
    }

    return got;
}

size_t AudioInput::ReadFrame1s(int16_t* dst, uint32_t timeout_ms) {
    if (dst == nullptr) {
        return 0;
    }

    size_t got = 0;
    TickType_t start = xTaskGetTickCount();

    while (got < AUDIO_FRAME_SAMPLES) {
        uint32_t remain = portMAX_DELAY;
        if (timeout_ms != portMAX_DELAY) {
            uint32_t elapsed_ms =
                (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
            if (elapsed_ms >= timeout_ms) {
                break;
            }
            remain = timeout_ms - elapsed_ms;
        }

        size_t n = Read(dst + got, AUDIO_FRAME_SAMPLES - got, remain);
        if (n == 0) {
            break;
        }
        got += n;
    }

    return got;
}

size_t AudioInput::Available() const {
    return ring_.Available();
}

size_t AudioInput::OverrunFrames() const {
    return ring_.DroppedFrames();
}

}  // namespace sleep_pillow
