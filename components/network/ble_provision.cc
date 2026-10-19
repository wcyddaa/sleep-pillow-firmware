// ble_provision.cc - BLE 配网实现（NimBLE + AES-128-GCM）
// 协议细节见 ble_provision.h 顶部注释
#include "ble_provision.h"

#include <cstring>

#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mbedtls/gcm.h"
#include "nvs_flash.h"

#include "app_config.h"   // 写 NVS 用
#include "cJSON.h"

#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static constexpr const char* TAG = "BleProvision";

// ---------------- 协议常量 ----------------
static constexpr uint8_t kCmdWifiCred     = 0x01;
static constexpr uint8_t kCmdSampling     = 0x02;
static constexpr uint8_t kCmdGetCfg       = 0x04;
static constexpr uint8_t kCmdSetCfg       = 0x05;
static constexpr uint8_t kSamplingStart   = 0x01;

static constexpr int kIvLen      = 12;
static constexpr int kTagLen     = 16;
static constexpr int kMaxPayload = 256;  // 单包最大长度
static constexpr int kMtu        = 256;

// ---------------- 运行状态 ----------------
static uint8_t  s_own_addr_type  = 0;
static uint16_t s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_val_handle  = 0;
static char     s_device_name[32] = {};
static char     s_device_id[64]  = {};   // IoTDA device_id，供手机 BLE 读取

// ---------------- UUID（Nordic UART 风格 6E400001/2/3/4-...）----------------
static const ble_uuid128_t s_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_device_id_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x04, 0x00, 0x40, 0x6e);

// 读特征：返回设备身份 ID（IoTDA device_id）
static int DeviceIdAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt* ctxt, void* arg) {
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    return os_mbuf_append(ctxt->om, reinterpret_cast<const uint8_t*>(s_device_id),
                          strlen(s_device_id)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// ---------------- 配网任务事件 ----------------
enum class ProvEventType : uint8_t {
    kRaw,   // 原始 RX 载荷，由 ble_prov 任务解析
};

// ★ 事件只携带**原始 RX 载荷**，一切解析（AES-GCM 解密 / cJSON / 写 NVS /
//   启停模型任务）都由 ble_prov 任务完成。
//
//   原因：GattAccessCb 跑在 nimble_host 任务里（栈默认 4096，且同栈还要放
//   256B 收包缓冲和 esp_log 的 vsnprintf）。在那里做 cJSON 解析（约 2KB）
//   或 mbedtls 解密（约 1.5KB）会直接把栈打爆 → 实测报
//   `***ERROR*** A stack overflow in task nimble_host` → 设备重启 →
//   手机端表现为"读配置/点开关时蓝牙突然断连"。
struct ProvEvent {
    ProvEventType type;
    uint16_t len;
    uint8_t  pkt[kMaxPayload];   // 原始载荷（含首字节命令码）
};

// ---------------- GATT 读写回调 ----------------
static int GattAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg) {
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    uint8_t buf[kMaxPayload] = {};
    uint16_t len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
    if (rc != 0) {
        ESP_LOGE(TAG, "读取写入数据失败: %d", rc);
        return BLE_ATT_ERR_UNLIKELY;
    }
    BleProvisioner::GetInstance().HandleRx(buf, len);
    return 0;
}

static const struct ble_gatt_chr_def s_chrs[] = {
    {
        .uuid = &s_rx_uuid.u,
        .access_cb = GattAccessCb,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = nullptr,
        .cpfd = nullptr,
    },
    {
        .uuid = &s_tx_uuid.u,
        .access_cb = GattAccessCb,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &s_tx_val_handle,
        .cpfd = nullptr,
    },
    {
        .uuid = &s_device_id_uuid.u,
        .access_cb = DeviceIdAccessCb,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
        .cpfd = nullptr,
    },
    {},
};

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .includes = nullptr,
        .characteristics = s_chrs,
    },
    {},
};

// ---------------- 广播 / GAP ----------------
static int GapEventCb(struct ble_gap_event* event, void* arg);

static void StartAdvertising() {
    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<const uint8_t*>(s_device_name);
    fields.name_len = static_cast<uint8_t>(strlen(s_device_name));
    fields.name_is_complete = 1;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "配置广播数据失败: %d", rc);
        return;
    }

    // 128 位服务 UUID 放不进 31 字节的广播包，放扫描响应里
    struct ble_hs_adv_fields rsp_fields = {};
    rsp_fields.uuids128 = &s_svc_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGW(TAG, "配置扫描响应失败: %d", rc);
    }

    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, nullptr, BLE_HS_FOREVER,
                           &adv_params, GapEventCb, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "启动广播失败: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "开始广播，设备名: %s", s_device_name);
}

static int GapEventCb(struct ble_gap_event* event, void* arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "手机已连接 (conn=%d)", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "连接失败 status=%d，继续广播", event->connect.status);
            StartAdvertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "手机已断开 reason=%d，继续广播", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        StartAdvertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        StartAdvertising();
        return 0;

    default:
        return 0;
    }
}

// ---------------- NimBLE 主机回调 ----------------
static void OnReset(int reason) {
    ESP_LOGW(TAG, "BLE 协议栈重置，reason=%d", reason);
}

static void OnSync() {
    if (ble_hs_util_ensure_addr(0) != 0) {
        ESP_LOGE(TAG, "无可用 BLE 地址");
        return;
    }
    if (ble_hs_id_infer_auto(0, &s_own_addr_type) != 0) {
        ESP_LOGE(TAG, "BLE 地址类型推断失败");
        return;
    }
    StartAdvertising();
}

static void HostTask(void* param) {
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ---------------- BleProvisioner ----------------
BleProvisioner& BleProvisioner::GetInstance() {
    static BleProvisioner instance;
    return instance;
}

void BleProvisioner::SetHandlers(WifiCredHandler on_wifi, SamplingHandler on_sampling, CfgHandler on_cfg) {
    on_wifi_ = std::move(on_wifi);
    on_sampling_ = std::move(on_sampling);
    on_cfg_ = std::move(on_cfg);
}

void BleProvisioner::SetDeviceId(const std::string& id) {
    strncpy(s_device_id, id.c_str(), sizeof(s_device_id) - 1);
}

bool BleProvisioner::Start(const std::string& device_name) {
    if (device_name.empty() || device_name.size() >= sizeof(s_device_name)) {
        ESP_LOGE(TAG, "设备名非法（长度需 1~%u）", (unsigned)sizeof(s_device_name) - 1);
        return false;
    }
    strncpy(s_device_name, device_name.c_str(), sizeof(s_device_name) - 1);

    queue_ = xQueueCreate(4, sizeof(ProvEvent));
    if (queue_ == nullptr) {
        ESP_LOGE(TAG, "事件队列创建失败");
        return false;
    }

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_ret = nvs_flash_init();
    }
    if (nvs_ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS 初始化异常: %s", esp_err_to_name(nvs_ret));
    }

    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init 失败");
        return false;
    }

    ble_hs_cfg.reset_cb = OnReset;
    ble_hs_cfg.sync_cb = OnSync;
    ble_att_set_preferred_mtu(kMtu);

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_svcs);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "注册 GATT 服务失败: %d", rc);
        return false;
    }
    ble_svc_gap_device_name_set(s_device_name);

    // 通知队列必须先建：HandleRx 可能在任何时刻被 GATT 回调触发
    notify_queue_ = xQueueCreate(kNotifyQueueLen, sizeof(NotifyMsg));
    if (notify_queue_ == nullptr) {
        ESP_LOGE(TAG, "通知队列创建失败");
        return false;
    }

    nimble_port_freertos_init(HostTask);
    // ble_prov 的栈要够大：Wi-Fi 凭据回调（OnWifiCred）在这个任务里跑，
    // 里面会做 mbedtls HMAC（配网密码）、esp_netif/SNTP、esp_mqtt_client_init，
    // 全是吃栈的操作。原来 4096 偏紧，加大到 8192。
    xTaskCreate(ProvisionTaskEntry, "ble_prov", 8192, this, 5, nullptr);
    xTaskCreate(NotifyTaskEntry, "ble_notify", 4096, this, 4, nullptr);

    // 把它打出来，方便一眼确认 sdkconfig 的栈改动是否真的编进了固件
    // （排查 nimble_host 栈溢出时必须知道这个值）。
    ESP_LOGI(TAG, "BLE 配网已启动，等待手机连接 (nimble_host 栈=%d, ble_prov 栈=8192)",
             (int)CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE);
    return true;
}

// ============================================================
// Notify：任何上下文安全（只入队）
// ============================================================
void BleProvisioner::Notify(const std::string& json) {
    if (notify_queue_ == nullptr) {
        return;
    }
    NotifyMsg m = {};
    strncpy(m.json, json.c_str(), sizeof(m.json) - 1);
    if (xQueueSend(notify_queue_, &m, 0) != pdTRUE) {
        // 队列满：丢掉最旧的一条再塞，保证最新状态能发出去
        NotifyMsg dropped;
        xQueueReceive(notify_queue_, &dropped, 0);
        if (xQueueSend(notify_queue_, &m, 0) != pdTRUE) {
            ESP_LOGW(TAG, "通知队列持续拥塞，丢弃一条");
        }
    }
}

// ============================================================
// NotifyTaskEntry：真正的下发（已脱离 GATT 回调上下文）
// ============================================================
void BleProvisioner::NotifyTaskEntry(void* arg) {
    auto* self = static_cast<BleProvisioner*>(arg);
    NotifyMsg m;
    while (true) {
        if (xQueueReceive(self->notify_queue_, &m, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        self->SendNotifyRaw(m.json);
    }
}

void BleProvisioner::SendNotifyRaw(const char* json) {
    if (json == nullptr) {
        return;
    }
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_tx_val_handle == 0) {
        return;  // 没有手机连接，静默丢弃
    }
    const size_t len = strlen(json);
    struct os_mbuf* om = ble_hs_mbuf_from_flat(json, len);
    if (om == nullptr) {
        return;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify 失败: %d", rc);
    }
}

// ============================================================
// HandleRx：GATT 访问回调入口 —— ★ 只做「拷贝 + 入队」
//
// 本函数运行在 nimble_host 任务里（栈默认 4096，且同栈还要放 esp_log 的
// vsnprintf）。实测在这里做解析会报：
//     ***ERROR*** A stack overflow in task nimble_host has been detected.
// 于是设备重启、手机端表现为"读配置/点开关时蓝牙突然断连"。
// 所以这里只把原始载荷拷进事件并入队，一切解析交给 ProcessRx。
// ============================================================
void BleProvisioner::HandleRx(const uint8_t* data, uint16_t len) {
    if (data == nullptr || len < 1 || queue_ == nullptr) {
        return;
    }
    if (len > kMaxPayload) {
        len = kMaxPayload;
    }
    ProvEvent evt = {};
    evt.type = ProvEventType::kRaw;
    evt.len  = len;
    memcpy(evt.pkt, data, len);
    if (xQueueSend(queue_, &evt, 0) != pdTRUE) {
        ESP_LOGW(TAG, "事件队列已满，丢弃 BLE 指令");
    }
}

// ============================================================
// ProcessRx：真正的指令处理（跑在 ble_prov 任务，栈 8192）
// ============================================================
void BleProvisioner::ProcessRx(const uint8_t* data, uint16_t len) {
    if (data == nullptr || len < 1) {
        return;
    }
    const uint8_t cmd = data[0];
    // 排查"手机说设备无响应"时，只要看有没有这行就知道命令到没到设备。
    ESP_LOGI(TAG, "BLE RX cmd=0x%02x len=%u", cmd, static_cast<unsigned>(len));

    // ---- 读三开关配置：0x04 ----
    if (cmd == kCmdGetCfg) {
        CfgReq req{};
        req.is_set = false;
        HandleCfg(req);
        return;
    }

    // ---- 写三开关配置：0x05 + JSON（明文）----
    if (cmd == kCmdSetCfg) {
        if (len < 2) {
            Notify("{\"evt\":\"error\",\"msg\":\"set_cfg too short\"}");
            return;
        }
        CfgReq req{};
        req.is_set = true;
        cJSON* root = cJSON_ParseWithLength(
            reinterpret_cast<const char*>(data + 1), len - 1);
        if (root == nullptr) {
            Notify("{\"evt\":\"error\",\"msg\":\"set_cfg bad json\"}");
        } else {
            cJSON* jsn = cJSON_GetObjectItemCaseSensitive(root, "snore");
            cJSON* jst = cJSON_GetObjectItemCaseSensitive(root, "stage");
            cJSON* jbc = cJSON_GetObjectItemCaseSensitive(root, "bcg");
            req.snore = (cJSON_IsNumber(jsn) ? jsn->valueint : 0) & 0x3;
            req.stage = (cJSON_IsNumber(jst) ? jst->valueint : 0) & 0x3;
            req.bcg   = (cJSON_IsNumber(jbc) ? jbc->valueint : 0) & 0x3;
            cJSON_Delete(root);
        }
        HandleCfg(req);
        return;
    }

    // ---- 采集开关：明文 [0x02][0x01|0x00] ----
    if (cmd == kCmdSampling) {
        if (len < 2) {
            Notify("{\"evt\":\"error\",\"msg\":\"payload too short\"}");
            return;
        }
        if (on_sampling_) {
            on_sampling_(data[1] == kSamplingStart);
        }
        return;
    }

    // ---- Wi-Fi 凭据：AES-128-GCM 密文 [0x01][IV12][密文][TAG16] ----
    if (cmd == kCmdWifiCred) {
        if (len < 1 + kIvLen + kTagLen + 1) {
            ESP_LOGW(TAG, "Wi-Fi 凭据包长度不足: %u", len);
            Notify("{\"evt\":\"error\",\"msg\":\"payload too short\"}");
            return;
        }

        const uint8_t* iv         = data + 1;
        const uint8_t* cipher     = data + 1 + kIvLen;
        const size_t   cipher_len = static_cast<size_t>(len) - 1 - kIvLen - kTagLen;
        const uint8_t* tag        = data + len - kTagLen;
        if (cipher_len >= kMaxPayload) {
            ESP_LOGW(TAG, "Wi-Fi 凭据包过大: %u", (unsigned)cipher_len);
            Notify("{\"evt\":\"error\",\"msg\":\"payload too large\"}");
            return;
        }

        uint8_t plain[kMaxPayload] = {};
        mbedtls_gcm_context ctx;
        mbedtls_gcm_init(&ctx);
        int rc = mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES,
                                    reinterpret_cast<const unsigned char*>(kPsk), 128);
        if (rc == 0) {
            rc = mbedtls_gcm_auth_decrypt(&ctx, cipher_len, iv, kIvLen,
                                          nullptr, 0, tag, kTagLen, cipher, plain);
        }
        mbedtls_gcm_free(&ctx);
        if (rc != 0) {
            ESP_LOGE(TAG, "解密失败: -0x%04X（检查手机端密钥与算法是否一致）", -rc);
            Notify("{\"evt\":\"error\",\"msg\":\"decrypt failed\"}");
            return;
        }

        cJSON* root = cJSON_ParseWithLength(
            reinterpret_cast<const char*>(plain), cipher_len);
        if (root == nullptr) {
            ESP_LOGE(TAG, "明文不是合法 JSON");
            Notify("{\"evt\":\"error\",\"msg\":\"bad json\"}");
            return;
        }
        const cJSON* j_ssid = cJSON_GetObjectItemCaseSensitive(root, "ssid");
        const cJSON* j_pass = cJSON_GetObjectItemCaseSensitive(root, "password");
        if (!cJSON_IsString(j_ssid) || !cJSON_IsString(j_pass)) {
            cJSON_Delete(root);
            ESP_LOGE(TAG, "明文缺少 ssid / password 字段");
            Notify("{\"evt\":\"error\",\"msg\":\"missing ssid or password\"}");
            return;
        }
        const std::string ssid(j_ssid->valuestring);
        const std::string password(j_pass->valuestring);
        cJSON_Delete(root);

        ESP_LOGI(TAG, "收到加密 Wi-Fi 凭据: ssid=%s", ssid.c_str());
        if (on_wifi_) {
            on_wifi_(ssid, password);
        }
        return;
    }

    ESP_LOGW(TAG, "未知指令: 0x%02X", cmd);
    Notify("{\"evt\":\"error\",\"msg\":\"unknown command\"}");
}
void BleProvisioner::ProvisionTaskEntry(void* arg) {
    auto* self = static_cast<BleProvisioner*>(arg);
    ProvEvent evt;
    while (true) {
        if (xQueueReceive(self->queue_, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        // 事件只带原始载荷，全部解析都在这里做（本任务栈 8192，
        // 足够容纳 mbedtls 解密 + cJSON 解析 + NVS 写入）
        if (evt.type == ProvEventType::kRaw) {
            self->ProcessRx(evt.pkt, evt.len);
        }
    }
}

// ============================================================
// HandleCfg：0x04 GET / 0x05 SET
//   读：从 NVS 读 AppConfig，回 JSON 给手机
//   写：写 NVS + 触发 on_cfg 回调（app_main 据此重启三 task）
//
// ⚠️ 本函数由 ProcessRx 调用，运行在 ble_prov 任务里（栈 8192），
//    不在 nimble_host 的 GATT 回调上下文 —— 那里做 NVS 写会阻塞主机任务
//    并导致栈溢出。回包统一走 Notify()（内部入队，由 ble_notify 任务发送）。
// ============================================================
void BleProvisioner::HandleCfg(const CfgReq& req) {
    auto* store = sleep_pillow::AppConfigStore::GetInstance();
    if (!req.is_set) {
        // GET：直接回当前 cfg
        const char* js = store->ToJson();
        std::string out = std::string("{\"evt\":\"cfg\",") + std::string(js).substr(1);  // {"evt":"cfg","snore":...}
        // 上面的 substr(1) 把最外 { 替换成 {"evt":"cfg", 前缀
        // 但更稳的写法：直接拼
        out = std::string("{\"evt\":\"cfg\",\"snore\":") + std::to_string((int)store->Get().snore_mode)
            + ",\"stage\":" + std::to_string((int)store->Get().stage_mode)
            + ",\"bcg\":" + std::to_string((int)store->Get().bcg_mode) + "}";
        Notify(out.c_str());
        ESP_LOGI(TAG, "GET_CFG -> %s", out.c_str());
        return;
    }
    // SET：写 NVS + 回调
    using sleep_pillow::EdgeMode;
    sleep_pillow::AppConfig cfg = store->Get();
    cfg.snore_mode = (EdgeMode)req.snore;
    cfg.stage_mode = (EdgeMode)req.stage;
    cfg.bcg_mode   = (EdgeMode)req.bcg;
    if (store->Set(cfg) != ESP_OK) {
        Notify("{\"evt\":\"error\",\"msg\":\"set_cfg nvs fail\"}");
        return;
    }
    ESP_LOGI(TAG, "SET_CFG snore=%d stage=%d bcg=%d", req.snore, req.stage, req.bcg);
    if (on_cfg_) {
        on_cfg_(req.snore, req.stage, req.bcg);
    }
    Notify("{\"evt\":\"cfg_ok\"}");
}
