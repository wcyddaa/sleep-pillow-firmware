// esp_sr_wake.cc - 语音唤醒词检测 (桩文件)
#include "esp_log.h"
#include <cstdint>
#include <cstddef>

static const char* TAG = "WakeWord";

extern "C" {

// 唤醒词检测初始化（桩）
int wake_word_init() {
    ESP_LOGI(TAG, "Wake word init (stub)");
    return 0;
}

// 唤醒词检测处理（桩）
// 输入音频数据，返回是否检测到唤醒词
int wake_word_detect(const int16_t* audio_data, int num_samples) {
    (void)audio_data;
    (void)num_samples;
    return 0;  // 0 = 未检测到
}

// 唤醒词检测释放（桩）
void wake_word_deinit() {
    ESP_LOGI(TAG, "Wake word deinit (stub)");
}

}  // extern "C"
