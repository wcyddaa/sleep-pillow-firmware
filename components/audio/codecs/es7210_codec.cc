// es7210_codec.cc - ES7210音频编解码器实现 (桩文件)
// ES7210: 多通道录音ADC，I2S接口
#include "audio_codec.h"
#include "esp_log.h"

namespace sleep_pillow {

static const char* TAG = "ES7210";

class ES7210Codec : public AudioCodec {
public:
    ES7210Codec() = default;
    ~ES7210Codec() override = default;

    esp_err_t Init() override {
        ESP_LOGI(TAG, "ES7210 init (stub)");
        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t Deinit() override {
        initialized_ = false;
        return ESP_OK;
    }

    esp_err_t SetVolume(int volume) override {
        return ESP_ERR_NOT_SUPPORTED;
    }

    int GetVolume() override { return 0; }

    esp_err_t StartRecording() override {
        recording_ = true;
        return ESP_OK;
    }

    esp_err_t StopRecording() override {
        recording_ = false;
        return ESP_OK;
    }

    esp_err_t StartPlayback() override {
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t StopPlayback() override {
        return ESP_ERR_NOT_SUPPORTED;
    }

    size_t WriteData(const uint8_t* data, size_t size) override {
        return 0;
    }

    esp_err_t SetSampleRate(int rate) override {
        sample_rate_ = rate;
        return ESP_OK;
    }

    int GetSampleRate() override { return sample_rate_; }

    esp_err_t SetDmaConfig(const AudioDmaConfig& config) override {
        dma_config_ = config;
        return ESP_OK;
    }

    AudioDmaConfig GetDmaConfig() override { return dma_config_; }

    void SetAudioDataCallback(on_audio_data_cb_t callback, void* ctx) override {
        callback_ = callback;
        ctx_ = ctx;
    }

    bool IsRecording() override { return recording_; }
    bool IsPlaying() override { return false; }
    bool IsInitialized() override { return initialized_; }

    const char* GetCodecName() override { return "ES7210"; }
    uint8_t GetI2cAddress() override { return 0x40; }

private:
    bool initialized_ = false;
    bool recording_   = false;
    int  sample_rate_ = 16000;
    AudioDmaConfig dma_config_;
    on_audio_data_cb_t callback_ = nullptr;
    void* ctx_ = nullptr;
};

}  // namespace sleep_pillow
