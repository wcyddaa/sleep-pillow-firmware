// wifi_manager.cc - Wi-Fi连接管理实现
#include "wifi.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <mutex>
#include <string.h>

static constexpr const char* TAG = "WifiManager";
// 阶段 1：等与 AP 完成 L2 关联
static constexpr int CONNECT_TIMEOUT_MS = 15000;
// 阶段 2：等 DHCP 分配 IP（★ 必须等！只关联不上网会导致 DNS 解析失败）
static constexpr int DHCP_TIMEOUT_MS    = 20000;
static constexpr int POLL_INTERVAL_MS   = 100;

// 串行化连接：开机回退连接与 BLE 配网连接可能来自不同任务
static std::mutex s_connect_mutex;

// ============================================================
// EnsureInited：Wi-Fi 子系统初始化（幂等，只做一次）
// ============================================================
bool WifiManager::EnsureInited() {
    if (inited_) {
        return true;
    }

    nvs_flash_init();
    esp_netif_init();

    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "创建默认事件循环失败: %s", esp_err_to_name(ret));
        return false;
    }
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(ret));
        return false;
    }
    esp_wifi_set_mode(WIFI_MODE_STA);

    inited_ = true;
    return true;
}

// ============================================================
// Connect：连接 Wi-Fi（可重复调用，用于切换凭据）
// ============================================================
bool WifiManager::Connect(const char* ssid, const char* password) {
    if (ssid == nullptr || password == nullptr) {
        ESP_LOGE(TAG, "SSID / 密码为空");
        return false;
    }

    std::lock_guard<std::mutex> lock(s_connect_mutex);
    if (!EnsureInited()) {
        return false;
    }

    // 已连着同一 AP 则跳过重连，避免打断在途的 MQTT 等长连接
    wifi_ap_record_t cur_ap = {};
    if (esp_wifi_sta_get_ap_info(&cur_ap) == ESP_OK &&
        strncmp(reinterpret_cast<const char*>(cur_ap.ssid), ssid,
                sizeof(cur_ap.ssid)) == 0) {
        connected_ = true;
        return true;
    }

    // 断开旧连接，避免用旧凭据继续连着
    connected_ = false;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // 已启动时 esp_wifi_start 返回 ESP_ERR_INVALID_STATE，属于正常
    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start 失败: %s", esp_err_to_name(ret));
        return false;
    }
    esp_wifi_connect();

    ESP_LOGI(TAG, "Connecting to %s ...", ssid);
    if (!WaitAssociated()) {
        return false;
    }
    if (!WaitGotIp()) {
        return false;
    }
    connected_ = true;
    ESP_LOGI(TAG, "Wi-Fi 就绪（已关联 + 已拿 IP），可以上云");
    return true;
}

// ============================================================
// ConnectStored：开机用 NVS 里已保存的凭据自动重连
//   没有这个，任何一次重启（含崩溃重启）都会让设备离线，
//   必须重新配网 —— 表现就是"手机一离开小程序，Wi-Fi 就断了"。
// ============================================================
bool WifiManager::ConnectStored() {
    std::lock_guard<std::mutex> lock(s_connect_mutex);
    if (!EnsureInited()) {
        return false;
    }

    wifi_config_t stored = {};
    if (esp_wifi_get_config(WIFI_IF_STA, &stored) != ESP_OK ||
        stored.sta.ssid[0] == 0) {
        ESP_LOGI(TAG, "NVS 里没有 Wi-Fi 凭据，等 BLE 配网");
        return false;
    }
    ESP_LOGI(TAG, "开机自动重连 Wi-Fi: %s", reinterpret_cast<const char*>(stored.sta.ssid));

    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_start 失败: %s", esp_err_to_name(ret));
        return false;
    }
    esp_wifi_connect();

    if (!WaitAssociated()) {
        return false;
    }
    if (!WaitGotIp()) {
        return false;
    }
    connected_ = true;
    ESP_LOGI(TAG, "Wi-Fi 自动重连成功，可以上云");
    return true;
}

// ---------- 阶段 1：等 L2 关联 ----------
bool WifiManager::WaitAssociated() {
    wifi_ap_record_t ap_info;
    for (int elapsed = 0; elapsed < CONNECT_TIMEOUT_MS; elapsed += POLL_INTERVAL_MS) {
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
    ESP_LOGE(TAG, "关联 AP 超时 (%d ms)，检查 SSID/密码/是否 2.4G", CONNECT_TIMEOUT_MS);
    return false;
}

// ---------- 阶段 2：等 DHCP 拿到 IP ----------
// 关键：光关联上 AP 不等于能上网。拿不到 IP / DNS 时，
// MQTT 会一直报 esp-tls: getaddrinfo() returns 202 (EAI_FAIL)。
bool WifiManager::WaitGotIp() {
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    for (int elapsed = 0; elapsed < DHCP_TIMEOUT_MS; elapsed += POLL_INTERVAL_MS) {
        esp_netif_ip_info_t ip{};
        if (netif != nullptr && esp_netif_get_ip_info(netif, &ip) == ESP_OK &&
            ip.ip.addr != 0) {
            ESP_LOGI(TAG, "获取 IP: " IPSTR "  网关: " IPSTR,
                     IP2STR(&ip.ip), IP2STR(&ip.gw));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
    connected_ = false;
    ESP_LOGE(TAG, "★ DHCP 超时：已关联 AP 但没拿到 IP —— 设备无法上网，"
                  "MQTT/DNS 必然失败。请检查路由器 DHCP 是否正常。");
    return false;
}

bool WifiManager::IsConnected() {
    wifi_ap_record_t ap_info;
    connected_ = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
    return connected_;
}
