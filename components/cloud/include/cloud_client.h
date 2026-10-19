// cloud_client.h - 华为云 IoTDA MQTT 接入（设备侧）
// ============================================================
// 联网后由 AppMain 调用 Start() 建立 MQTT 长连接，之后设备只上行两类消息，
// **共用同一个华为云标准系统主题**：
//     $oc/devices/{device_id}/sys/messages/up
//   1) 上线报到      payload {"device_id":"..","evt":"online"}（MQTT 每次连上发一次）
//   2) 30s 结构化帧  payload {"device_id":"..","evt":"frame",...}（EdgeAggregator 组装）
// 后端按 payload 的 evt 字段分流，IoTDA 侧只需要**一条规则、一个后端地址**。
//
// 为什么不用自定义主题（/device/{id}/xxx）：自定义主题需要先在产品侧声明，
// 否则平台直接丢弃 —— 本项目历史上 cfg 走 /device/{id}/cfg 时就是这么一直没通。
//
// 注：三开关（snore/stage/bcg）是**设备本地状态**，走 BLE 0x04/0x05 读写，
//     本地权威性不变；BLE 切开关时会补发一次 evt:"cfg" 快照上行，
//     后端落 device_cfg 表做「云端可查」镜像。快照不进离线队列、断网即放弃
//     （过期 cfg 不应覆盖新状态），每帧携带的 cfg 字段在后端兜底刷新。
//
// 鉴权（华为云密钥鉴权，一机一密）：
//   clientId = {device_id}_0_{sign_type}_{UTC时间戳 YYYYMMDDHH}
//   username = {device_id}
//   password = HMAC-SHA256(key=UTC时间戳, msg=设备密钥)，小写 hex
// sign_type=0 表示平台不校验时间戳，设备长时间在线也不会因密码过期被踢。
// ============================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "esp_event.h"
#include "mqtt_client.h"

// 云端连接参数
struct CloudConfig {
    std::string host;                       // IoTDA 接入域名
    uint16_t    port = 1883;                // 1883=MQTT，8883=MQTTS
    std::string device_id;                  // 平台设备 ID（同时作为 username）
    std::string secret;                     // 设备密钥
    uint8_t     sign_type = 0;              // 0=不校验时间戳，1=校验时间戳
    std::string report_device_id;           // 上报 JSON 里的 device_id（后端已注册标识）
};

class CloudClient {
public:
    static CloudClient& GetInstance();

    // 同步 NTP -> 生成鉴权参数 -> 启动 MQTT 长连接（需先接好 Wi-Fi）；可重复调用
    bool Start(const CloudConfig& config);

    bool IsConnected() const { return connected_.load(); }

    // NTP 墙上时钟是否已经同步（30s 帧时间戳用 time(nullptr)，未同步前禁止发帧）
    bool IsTimeSynced() const;

    // 上报 JSON 用的 device_id（后端 devices 表里的标识）
    const std::string& report_device_id() const { return config_.report_device_id; }

    // 注：不再有"设备上线报到"。
    //     注册/上下线完全交给 IoTDA 的「设备状态变更」规则 —— 设备连上/断开时
    //     平台自己会推 ONLINE/OFFLINE 事件，后端据此注册与置状态，无需固件参与。
    //     （后端仍支持 payload 带 evt:"online"，但固件不再主动发。）

    // ★ 30s 一帧结构化数据上行：主题 /device/{id}/frame
    //   输入已经是 JSON 字符串（由 EdgeAggregator 组装，含 device_id 字段）
    bool PublishFrame(const char* json);

    // ★ cfg 快照上行（BLE 切开关时 AppMain 调用一次）：
    //   payload 形如 {"device_id":"..","evt":"cfg","cfg":{...},"ts":..}
    //   与 PublishFrame 同主题，后端按 evt 分流落 device_cfg 快照。
    //   未连接直接返回 false，不进离线队列（过期 cfg 不应补传覆盖新状态）。
    bool PublishCfg(const char* json);

    // 上行主题查询（供测试/调试用）
    // 两类消息共用标准系统主题 $oc/devices/{id}/sys/messages/up，
    // 后端按 payload 的 evt 字段分流（online / frame）。
    const std::string& GetUpTopic() const { return topic_up_; }

private:
    CloudClient() = default;

    // esp-mqtt 事件回调（在 MQTT 任务中执行）
    static void EventHandler(void* args, esp_event_base_t base, int32_t event_id,
                             void* event_data);
    void        OnEvent(esp_mqtt_event_handle_t event);

    // 通过 SNTP 同步墙上时钟（上报时间戳需要绝对时间）
    bool        SyncTime(int timeout_ms);
    // 按华为云规则计算鉴权密码
    std::string BuildPassword() const;

    CloudConfig config_;

    // 以下三者在 MQTT 配置里以 const char* 被引用，必须保持生命周期
    std::string client_id_;
    std::string password_;
    // 唯一上行主题：$oc/devices/{id}/sys/messages/up（华为云系统主题，平台一定放行）
    // 上线报到与 30s 帧都发到这里，用 payload 里的 evt 区分。
    std::string topic_up_;

    esp_mqtt_client_handle_t client_ = nullptr;
    bool sntp_inited_ = false;
    std::atomic<bool> connected_{false};
};
