// app_config.cc - NVS 持久化三开关
#include "app_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <cstdio>

namespace sleep_pillow {

AppConfigStore* AppConfigStore::GetInstance() {
    static AppConfigStore inst;
    return &inst;
}

void AppConfigStore::LoadDefaults() {
    cfg_ = AppConfig{};  // 全 OFF
}

esp_err_t AppConfigStore::Init() {
    LoadDefaults();
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (%s), use defaults (全 OFF)", esp_err_to_name(err));
        return err;
    }
    uint8_t v = 0;
    if (nvs_get_u8(h, kKeySnore, &v) == ESP_OK) cfg_.snore_mode = static_cast<EdgeMode>(v & 0x3);
    if (nvs_get_u8(h, kKeyStage, &v) == ESP_OK) cfg_.stage_mode = static_cast<EdgeMode>(v & 0x3);
    if (nvs_get_u8(h, kKeyBcg,   &v) == ESP_OK) cfg_.bcg_mode   = static_cast<EdgeMode>(v & 0x3);
    nvs_close(h);
    ESP_LOGI(TAG, "loaded: snore=%d stage=%d bcg=%d",
             (int)cfg_.snore_mode, (int)cfg_.stage_mode, (int)cfg_.bcg_mode);
    return ESP_OK;
}

esp_err_t AppConfigStore::Save() {
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_set_u8(h, kKeySnore, (uint8_t)cfg_.snore_mode);
    nvs_set_u8(h, kKeyStage, (uint8_t)cfg_.stage_mode);
    nvs_set_u8(h, kKeyBcg,   (uint8_t)cfg_.bcg_mode);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t AppConfigStore::Set(const AppConfig& cfg) { cfg_ = cfg; return Save(); }
esp_err_t AppConfigStore::SetSnore(EdgeMode m)      { cfg_.snore_mode = m; return Save(); }
esp_err_t AppConfigStore::SetStage(EdgeMode m)      { cfg_.stage_mode = m; return Save(); }
esp_err_t AppConfigStore::SetBcg(EdgeMode m)        { cfg_.bcg_mode   = m; return Save(); }

const char* AppConfigStore::ToJson() {
    snprintf(json_buf_, sizeof(json_buf_),
             "{\"snore\":%d,\"stage\":%d,\"bcg\":%d}",
             (int)cfg_.snore_mode, (int)cfg_.stage_mode, (int)cfg_.bcg_mode);
    return json_buf_;
}

}  // namespace sleep_pillow
