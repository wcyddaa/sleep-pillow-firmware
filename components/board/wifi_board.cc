/**
 * WifiBoard 基类实现
 * Wi-Fi 连接公共逻辑
 */

#include "wifi_board.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

namespace sleep_pillow {

static const char* TAG = "WifiBoard";

// Wi-Fi 事件组标志位
static constexpr int WIFI_CONNECTED_BIT = BIT0;
static constexpr int WIFI_FAIL_BIT      = BIT1;

// 事件组句柄
static EventGroupHandle_t s_wifi_event_group = nullptr;

// 事件回调
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            ESP_LOGI(TAG, "Wi-Fi STA 已启动，尝试连接...");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_wifi_event_group) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGW(TAG, "Wi-Fi 断开连接");
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(TAG, "获取 IP: " IPSTR, IP2STR(&event->ip_info.ip));
            if (s_wifi_event_group) {
                xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            }
        }
    }
}

// ============================================================
// Init：初始化 Wi-Fi 子系统
// ============================================================
bool WifiBoard::Init() {
    if (wifi_initialized_) {
        ESP_LOGW(TAG, "Wi-Fi 已初始化，跳过");
        return true;
    }

    // 创建事件组
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == nullptr) {
        ESP_LOGE(TAG, "事件组创建失败");
        return false;
    }

    // 初始化 NVS（Wi-Fi 配置存储依赖）
    // 假设在 app_main 中已完成 nvs_flash_init()

    // 网络接口初始化
    esp_netif_create_default_wifi_sta();

    // Wi-Fi 初始化配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %s", esp_err_to_name(ret));
        return false;
    }

    // 注册事件回调
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, nullptr, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, nullptr, nullptr);

    // 设置 STA 模式
    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_initialized_ = true;
    ESP_LOGI(TAG, "Wi-Fi 子系统初始化完成（STA 模式）");
    return true;
}

// ============================================================
// GetNetwork：返回网络接口（当前为 Wi-Fi）
// ============================================================
Network* WifiBoard::GetNetwork() {
    return network_;
}

// ============================================================
// ConnectWifi：连接指定 SSID
// ============================================================
bool WifiBoard::ConnectWifi(const std::string& ssid, const std::string& password) {
    if (!wifi_initialized_) {
        ESP_LOGE(TAG, "Wi-Fi 未初始化，无法连接");
        return false;
    }

    // 清除之前的标志位
    xEventGroupClearBits(s_wifi_event_group,
                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    // 配置 SSID 和密码
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid.c_str(), sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password.c_str(), sizeof(wifi_config.sta.password) - 1);

    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "正在连接 Wi-Fi: %s", ssid.c_str());

    // 等待连接结果（超时 15 秒）
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi 连接成功: %s", ssid.c_str());
        return true;
    }

    ESP_LOGE(TAG, "Wi-Fi 连接失败: %s", ssid.c_str());
    return false;
}

// ============================================================
// DisconnectWifi：断开 Wi-Fi
// ============================================================
void WifiBoard::DisconnectWifi() {
    esp_wifi_disconnect();
    esp_wifi_stop();
    ESP_LOGI(TAG, "Wi-Fi 已断开");
}

// ============================================================
// IsWifiConnected：检查连接状态
// ============================================================
bool WifiBoard::IsWifiConnected() {
    wifi_ap_record_t ap_info;
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
}

// ============================================================
// GetIpAddress：获取 IP 地址
// ============================================================
std::string WifiBoard::GetIpAddress() {
    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == nullptr) {
        return "";
    }
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return "";
    }
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
    return std::string(ip_str);
}

}  // namespace sleep_pillow
