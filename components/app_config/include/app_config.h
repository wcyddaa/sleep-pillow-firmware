// app_config.h - 运行时三开关配置（snore / stage / bcg），NVS 持久化
// ============================================================
// 三个端侧模型各自三态：OFF / SIM / REAL
// - OFF : 模型任务不创建 / 已创建的自检退出
// - SIM : 假数据进模型（仍是真实模型推理）
// - REAL: 真实传感器采集
// 任意组合允许；缺哪个 → 30s frame 对应字段 -1
// 启动期默认值：全 OFF
// 运行时通过 BLE 0x05 SET_CFG 写入；MQTT /device/{id}/cfg 上行
// ============================================================
#pragma once

#include <stdint.h>
#include "esp_err.h"

namespace sleep_pillow {

enum class EdgeMode : uint8_t {
    kOff  = 0,
    kSim  = 1,
    kReal = 2,
};

struct AppConfig {
    EdgeMode snore_mode = EdgeMode::kOff;
    EdgeMode stage_mode = EdgeMode::kOff;
    EdgeMode bcg_mode   = EdgeMode::kOff;
};

class AppConfigStore {
public:
    static AppConfigStore* GetInstance();

    // 启动期从 NVS 读（缺失则用 AppConfig{} 默认值，即全 OFF）
    esp_err_t Init();
    // 写 NVS
    esp_err_t Save();

    const AppConfig& Get() const { return cfg_; }
    // 整体覆盖
    esp_err_t Set(const AppConfig& cfg);
    // 单开关更新
    esp_err_t SetSnore(EdgeMode m);
    esp_err_t SetStage(EdgeMode m);
    esp_err_t SetBcg(EdgeMode m);

    // 序列化为 JSON（用于 MQTT / BLE 上行）
    const char* ToJson();

private:
    AppConfigStore() = default;
    void LoadDefaults();
    AppConfig cfg_{};
    char json_buf_[128] = {0};
    static constexpr const char* TAG = "AppCfg";
    static constexpr const char* kNvsNamespace = "edge_cfg";
    static constexpr const char* kKeySnore = "snore";
    static constexpr const char* kKeyStage = "stage";
    static constexpr const char* kKeyBcg   = "bcg";
};

}  // namespace sleep_pillow
