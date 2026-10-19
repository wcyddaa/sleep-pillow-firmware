// es8311_codec.cc - ES8311音频编解码器实现 (桩文件)
// ES8311: 播放DAC + 录音ADC，I2S接口
#include "audio_codec.h"
#include "esp_log.h"

namespace sleep_pillow {

static const char* TAG = "ES8311";

class ES8311Codec : public AudioCodec {
public:
    ES8311Codec() = default;
    ~ES8311Codec() override = default;

    esp_err_t Init() override {
        ESP_LOGI(TAG, "ES8311 init (stub)");
        initialized_ = true;
        return ESP_OK;
    }

    esp_err_t Deinit() override {
        initialized_ = false;
        return ESP_OK;
    }

    esp_err_t SetVolume(int volume) override {
        volume_ = volume;
        return ESP_OK;
    }

    int GetVolume() override { return volume_; }

    esp_err_t StartRecording() override {
        recording_ = true;
        return ESP_OK;
    }

    esp_err_t StopRecording() override {
        recording_ = false;
        return ESP_OK;
    }

    esp_err_t StartPlayback() override {
        playing_ = true;
        return ESP_OK;
    }

    esp_err_t StopPlayback() override {
        playing_ = false;
        return ESP_OK;
    }

    size_t WriteData(const uint8_t* data, size_t size) override {
        return size;
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
    bool IsPlaying() override { return playing_; }
    bool IsInitialized() override { return initialized_; }

    const char* GetCodecName() override { return "ES8311"; }
    uint8_t GetI2cAddress() override { return 0x18; }

private:
    bool initialized_ = false;
    bool recording_   = false;
    bool playing_     = false;
    int  volume_      = 50;
    int  sample_rate_ = 16000;
    AudioDmaConfig dma_config_;
    on_audio_data_cb_t callback_ = nullptr;
    void* ctx_ = nullptr;
};

}  // namespace sleep_pillow
