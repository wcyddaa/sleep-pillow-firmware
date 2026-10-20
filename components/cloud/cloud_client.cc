// cloud_client.cc - 华为云 IoTDA MQTT 接入实现
#include "cloud_client.h"

#include <time.h>

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/md.h"
#include "offline_queue.h"

static constexpr const char* TAG = "CloudClient";

namespace {

// 单 NTP server：阿里云主。
// sdkconfig 默认 CONFIG_LWIP_SNTP_MAX_SERVERS=1，要多 server 得改 menuconfig + LWIP。
constexpr const char* kNtpServer     = "ntp.aliyun.com";
constexpr int         kNtpTimeoutMs  = 30000;   // 单次等待 30s（默认 10s 太短）
constexpr size_t      kClientIdMax   = 256;     // 华为云上限

// 2023-11-14 22:13:20 UTC 之后的 epoch 秒视为"已同步"，避免 SNTP 未握手成功时
// time(nullptr) 仍然返回 boot 秒导致 30s 帧时间戳全是 1970+boot_seconds。
constexpr time_t      kSyncedEpochMin = 1700000000;

// 华为云鉴权时间戳：UTC 的 YYYYMMDDHH
std::string UtcHourStamp() {
    const time_t now = time(nullptr);
    struct tm utc {};
    gmtime_r(&now, &utc);
    // 缓冲区按 GCC 最坏情况估算放大，避免 -Werror=format-truncation
    char buf[64] = {};
    snprintf(buf, sizeof(buf), "%04d%02d%02d%02d",
             utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour);
    return std::string(buf);
}

std::string ToHexLower(const unsigned char* data, size_t len) {
    static const char kHex[] = "0123456789abcdef";
    std::string out;
    out.reserve(len * 2);
    for (size_t i = 0; i < len; ++i) {
        out.push_back(kHex[(data[i] >> 4) & 0x0F]);
        out.push_back(kHex[data[i] & 0x0F]);
    }
    return out;
}

// 打印 esp-mqtt 的错误详情，便于定位"连上后掉线"的原因
void LogMqttError(esp_mqtt_event_handle_t event) {
    const esp_mqtt_error_codes_t* e = event->error_handle;
    if (e == nullptr) {
        return;
    }
    ESP_LOGE(TAG, "  error_type=%d connect_return_code=%d sock_errno=%d",
             static_cast<int>(e->error_type), static_cast<int>(e->connect_return_code),
             static_cast<int>(e->esp_transport_sock_errno));
}

// 断网积压帧补传任务：MQTT 每次连上后由事件回调拉起，
// 按序把 SPIFFS 离线队列里的帧补传出去；发完或再次断开即退出。
// 帧自带 start/end 时间戳，后端按 (device_id, start) 幂等去重，
// 与实时帧交错到达也不会污染数据。
void DrainQueueTask(void* arg) {
    auto* self = static_cast<CloudClient*>(arg);
    auto& q    = sleep_pillow::OfflineQueue::GetInstance();
    char  buf[512];
    ESP_LOGI(TAG, "开始补传离线帧，积压 %u 帧", (unsigned)q.Count());
    while (self->IsConnected() && q.Count() > 0) {
        size_t len = 0;
        if (!q.PeekOldest(buf, sizeof(buf), &len)) {
            // 损坏帧已被自动剔除，稍后轮到下一帧
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (!self->PublishFrame(buf)) {
            ESP_LOGW(TAG, "补传发送失败，等下次重连再继续");
            break;
        }
        q.PopOldest();
        vTaskDelay(pdMS_TO_TICKS(100));   // 逐帧补传，避免瞬时洪峰
    }
    ESP_LOGI(TAG, "离线帧补传结束，剩余 %u 帧", (unsigned)q.Count());
    vTaskDelete(nullptr);
}

}  // namespace

CloudClient& CloudClient::GetInstance() {
    static CloudClient instance;
    return instance;
}

// ============================================================
// SyncTime：NTP 同步（幂等，只有首次真正初始化 SNTP）
//   - 单 server：当前 sdkconfig 默认 CONFIG_LWIP_SNTP_MAX_SERVERS=1，
//     .servers[] 只能放 1 个 server；要 4 server 兜底得 menuconfig 改 LWIP SNTP，
//     涉及 LWIP 内部数据结构，先用单 server + 3 次重试
//   - 长超时：单次 30s（默认 10s 在 WiFi 拿 IP 后立即调容易失败）
//   - 多轮重试：3 次 sync_wait
// ============================================================
bool CloudClient::SyncTime(int timeout_ms) {
    if (!sntp_inited_) {
        // CONFIG_LWIP_SNTP_MAX_SERVERS=1，所以用单 server 版宏
        esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(kNtpServer);
        cfg.wait_for_sync = false;

        const esp_err_t err = esp_netif_sntp_init(&cfg);
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "SNTP 已初始化，复用现有会话");
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "SNTP 初始化失败: %s", esp_err_to_name(err));
            return false;
        }
        sntp_inited_ = true;
    }

    for (int attempt = 1; attempt <= 3; ++attempt) {
        const esp_err_t err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(timeout_ms));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "NTP 时间同步完成（attempt %d, server=%s）",
                     attempt, kNtpServer);
            return true;
        }
        ESP_LOGW(TAG, "NTP 同步超时（attempt %d/3, %d ms），重试中...",
                 attempt, timeout_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGE(TAG, "NTP 同步 3 次均失败，30s 帧时间戳将用 boot 秒占位");
    return false;
}

// ============================================================
// IsTimeSynced：检查 time(nullptr) 是否已对齐到真实 UTC
// ============================================================
bool CloudClient::IsTimeSynced() const {
    return time(nullptr) > kSyncedEpochMin;
}

// ============================================================
// BuildPassword：password = HMAC-SHA256(key=UTC时间戳, msg=设备密钥)
// ============================================================
std::string CloudClient::BuildPassword() const {
    const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (md == nullptr) {
        ESP_LOGE(TAG, "SHA256 不可用");
        return std::string();
    }

    const std::string stamp = UtcHourStamp();
    unsigned char mac[32] = {};
    mbedtls_md_hmac(md,
                    reinterpret_cast<const unsigned char*>(stamp.data()), stamp.size(),
                    reinterpret_cast<const unsigned char*>(config_.secret.data()),
                    config_.secret.size(), mac);
    return ToHexLower(mac, sizeof(mac));
}

// ============================================================
// Start：建连（可重复调用，已启动则直接返回）
// ============================================================
bool CloudClient::Start(const CloudConfig& config) {
    if (client_ != nullptr) {
        ESP_LOGW(TAG, "MQTT 客户端已启动，忽略重复初始化");
        return true;
    }
    if (config.host.empty() || config.device_id.empty() || config.secret.empty()) {
        ESP_LOGE(TAG, "云端配置不完整（host/device_id/secret 不能为空）");
        return false;
    }
    config_ = config;

    // 断网缓存：挂载 SPIFFS 离线队列（失败仅告警，不影响正常上行/实时帧）
    sleep_pillow::OfflineQueue::GetInstance().Init();

    // sign_type=1 时平台会校验时间戳，尽力同步一次 NTP
    SyncTime(kNtpTimeoutMs);

    const std::string stamp = UtcHourStamp();
    client_id_    = config_.device_id + "_0_" + std::to_string(config_.sign_type) + "_" + stamp;
    password_     = BuildPassword();
    // 唯一上行主题：华为云标准系统主题。
    // 用系统主题而不是自定义主题，是因为自定义主题需要先在产品侧声明，
    // 否则平台直接丢弃（本项目历史上 cfg 走 /device/{id}/cfg 时就是这么一直没通）。
    // 上线报到与 30s 帧共用此主题，后端按 payload 的 evt 分流。
    topic_up_ = "$oc/devices/" + config_.device_id + "/sys/messages/up";

    if (client_id_.size() > kClientIdMax) {
        ESP_LOGW(TAG, "clientId 长度 %u 超过华为云 256 上限",
                 static_cast<unsigned>(client_id_.size()));
    }

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.transport            = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg.broker.address.hostname             = config_.host.c_str();
    mqtt_cfg.broker.address.port                 = config_.port;
    mqtt_cfg.credentials.client_id               = client_id_.c_str();
    mqtt_cfg.credentials.username                = config_.device_id.c_str();
    mqtt_cfg.credentials.authentication.password = password_.c_str();
    mqtt_cfg.session.keepalive                   = 60;
    mqtt_cfg.network.reconnect_timeout_ms        = 5000;
    mqtt_cfg.network.timeout_ms                  = 10000;

    client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (client_ == nullptr) {
        ESP_LOGE(TAG, "MQTT 客户端创建失败");
        return false;
    }

    esp_mqtt_client_register_event(client_, MQTT_EVENT_ANY, EventHandler, nullptr);

    const esp_err_t err = esp_mqtt_client_start(client_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT 启动失败: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "MQTT 已启动: %s:%u clientId=%s",
             config_.host.c_str(), static_cast<unsigned>(config_.port), client_id_.c_str());
    return true;
}

// ============================================================
// 事件回调
// ============================================================
void CloudClient::EventHandler(void* /*args*/, esp_event_base_t /*base*/,
                               int32_t /*event_id*/, void* event_data) {
    GetInstance().OnEvent(static_cast<esp_mqtt_event_handle_t>(event_data));
}

void CloudClient::OnEvent(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            connected_.store(true);
            ESP_LOGI(TAG, "已连接云端 IoTDA");
            // 断网期间积压的帧：重连成功后按序补传（队列空时任务秒退）
            if (xTaskCreate(DrainQueueTask, "cloud_drain", 4096, this, 2, nullptr)
                    != pdPASS) {
                ESP_LOGW(TAG, "离线帧补传任务创建失败");
            }
            // 不做上线报到：上下线由 IoTDA「设备状态变更」规则处理 ——
            // 设备一连上，平台自己会推 ONLINE 事件 → 后端自动注册 + 置在线。
            break;

        case MQTT_EVENT_DISCONNECTED:
            connected_.store(false);
            ESP_LOGW(TAG, "与云端断开，SDK 将自动重连");
            LogMqttError(event);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "上报成功 msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT 错误");
            LogMqttError(event);
            break;

        default:
            break;
    }
}

// ============================================================
// PublishFrame：30s 一帧结构化数据上行（JSON 由 EdgeAggregator 组装）
// 主题 $oc/devices/{id}/sys/messages/up（华为云系统主题）
// payload 形如 {"device_id":"..","evt":"frame","start":..,...,"cfg":{..}}
// ============================================================
bool CloudClient::PublishFrame(const char* json) {
    if (client_ == nullptr || !connected_.load()) {
        ESP_LOGW(TAG, "PublishFrame：MQTT 未就绪，跳过");
        return false;
    }
    if (json == nullptr) return false;
    const int msg_id = esp_mqtt_client_publish(client_, topic_up_.c_str(), json, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "PublishFrame publish 失败");
        return false;
    }
    return true;
}

// ============================================================
// PublishCfg：cfg 快照上行（与 PublishFrame 同主题，后端按 evt 分流）
// 刻意不进离线队列：断网期间积压的过期 cfg 补传上去，
// 反而可能把设备已改的新状态覆盖掉 —— 所以未连接就直接放弃。
// ============================================================
bool CloudClient::PublishCfg(const char* json) {
    if (client_ == nullptr || !connected_.load()) {
        ESP_LOGW(TAG, "PublishCfg：MQTT 未就绪，放弃本次快照（帧内 cfg 兜底）");
        return false;
    }
    if (json == nullptr) return false;
    const int msg_id = esp_mqtt_client_publish(client_, topic_up_.c_str(), json, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "PublishCfg publish 失败");
        return false;
    }
    return true;
}
