// ble_provision.h - BLE 配网 + 采集开关 + 三开关配置
// ============================================================
// 设备作为 BLE 外设广播，手机连接后通过 GATT 下发指令：
//   1) 加密的 Wi-Fi 凭据 -> 设备解密后连接路由器（配网）
//   2) 采集开关指令      -> 控制心率采集的启停
//   3) 三开关配置指令    -> 读 / 写 snore/stage/bcg 各模式（明文 JSON）
//
// ---------------- 手机端需要实现的协议 ----------------
// 服务 UUID : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//   RX 特征 : 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
//             手机 -> 设备，属性 Write / Write Without Response
//   TX 特征 : 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
//             设备 -> 手机，属性 Notify
// 连接后请先把 MTU 协商到 256（Android 必须手动 requestMtu(256)，
// iOS 会自动协商，最大约 185 字节；本协议单包最大约 100 字节）。
//
// RX 载荷（手机 -> 设备，二进制）：
//   [0x01][IV(12B)][密文(N B)][TAG(16B)]
//       写入 Wi-Fi 凭据。
//       明文 = JSON 字符串，例如 {"ssid":"<你的WiFi名>","password":"<你的WiFi密码>"}
//       加密 = AES-128-GCM(密钥 kPsk, IV, 明文, 附加数据为空)
//   [0x02][0x01]  开启心率采集
//   [0x02][0x00]  停止心率采集
//   [0x04]                                  读三开关配置（设备回 JSON）
//   [0x05][JSON UTF-8]                      写三开关配置
//       JSON 形如 {"snore":0,"stage":1,"bcg":2}（0=OFF 1=SIM 2=REAL）
//
// TX 载荷（设备 -> 手机，UTF-8 JSON 文本）：
//   {"evt":"wifi","ok":true,"ssid":"<你的WiFi名>"}
//   {"evt":"wifi","ok":false,"ssid":"<你的WiFi名>"}
//   {"evt":"ctrl","sampling":true}
//   {"evt":"cfg","snore":0,"stage":0,"bcg":0}     // 响应 0x04 / 0x05
//   {"evt":"error","msg":"decrypt failed"}
// ============================================================
#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class BleProvisioner {
public:
    // 预共享密钥：手机端必须使用完全相同的 16 字节
    static constexpr uint8_t kPsk[16] = {
        'S', 'l', 'e', 'e', 'p', 'P', 'i', 'l', 'l', 'o', 'w', '-', '2', '0', '2', '6'};

    // Wi-Fi 凭据回调（在配网任务里执行，允许阻塞，例如连接耗时 5 秒）
    using WifiCredHandler = std::function<void(const std::string& ssid,
                                               const std::string& password)>;
    // 采集开关回调
    using SamplingHandler = std::function<void(bool enable)>;
    // 三开关配置回调（snore / stage / bcg 各 0/1/2）
    using CfgHandler = std::function<void(int snore, int stage, int bcg)>;

    static BleProvisioner& GetInstance();

    // 注册回调（必须在 Start() 之前调用）
    void SetHandlers(WifiCredHandler on_wifi, SamplingHandler on_sampling, CfgHandler on_cfg = nullptr);

    // 初始化 NimBLE 并以 device_name 开始广播
    bool Start(const std::string& device_name);

    // 设置设备身份 ID（IoTDA device_id），供手机通过 BLE 读特征读取以精确绑定
    void SetDeviceId(const std::string& id);

    // 向手机推送状态（未连接时静默忽略）。
    //
    // ★ 任何上下文都可以安全调用 —— 内部只入队，真正的 ble_gatts_notify_custom
    //   由独立的 notify 任务执行。
    //   原因：GATT 访问回调（GattAccessCb → HandleRx）运行在 NimBLE HostTask 里
    //         且持有 host lock，此时直接 notify 会二次抢锁并触发
    //         assert (xQueueSemaphoreTake / uxItemSize == 0) 崩溃重启。
    void Notify(const std::string& json);

    // 内部使用：GATT 收到一包数据后由 BLE 回调调用
    void HandleRx(const uint8_t* data, uint16_t len);

private:
    BleProvisioner() = default;

    // 配网任务：串行处理 BLE 下发的指令
    static void ProvisionTaskEntry(void* arg);

    // 真正的指令解析实现（AES-GCM 解密 / cJSON / 写 NVS / 启停模型任务）。
    // ★ 只能在 ble_prov 任务里调用 —— GATT 回调（HandleRx）只负责
    //   拷贝原始载荷入队，绝不在这里做解析（那会打爆 nimble_host 的栈）。
    void ProcessRx(const uint8_t* data, uint16_t len);

    // 通知任务：从 notify_queue_ 取 JSON 并真正下发（脱离 GATT 回调上下文）
    static void NotifyTaskEntry(void* arg);
    // 真正的发送实现，**只能在 NotifyTaskEntry 里调用**
    void SendNotifyRaw(const char* json);

    WifiCredHandler on_wifi_;
    SamplingHandler on_sampling_;
    CfgHandler      on_cfg_;
    QueueHandle_t queue_ = nullptr;

    // 待发通知队列：元素是定长字符串，避免在 GATT 回调里做堆分配
    static constexpr int kNotifyQueueLen = 12;
    static constexpr int kNotifyMaxLen   = 192;
    struct NotifyMsg {
        char json[kNotifyMaxLen];
    };
    QueueHandle_t notify_queue_ = nullptr;

    // 0x04 GET_CFG / 0x05 SET_CFG 走 task 直接处理（不通过 wifi/sampling 队列）
    struct CfgReq {
        bool is_set;       // false=GET, true=SET
        int  snore = 0;
        int  stage = 0;
        int  bcg   = 0;
    };
    void HandleCfg(const CfgReq& req);
};
