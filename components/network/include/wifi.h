// wifi.h - Wi-Fi连接管理接口
#pragma once

class WifiManager {
public:
    // 连接Wi-Fi（阻塞：先等关联，再等 DHCP 拿 IP）；可重复调用，用于 BLE 配网后切换凭据
    bool Connect(const char* ssid, const char* password);

    // ★ 开机自动重连：用 NVS 里已保存的凭据连接（WIFI_IF_STA 配置由 ESP-IDF 持久化）。
    //   没有凭据时返回 false（等 BLE 配网）。
    //   没有这个的话，每次重启（含崩溃重启）设备都会离线，必须重新配网。
    bool ConnectStored();

    // 检查是否已连接
    bool IsConnected();

private:
    // 首次调用时初始化 Wi-Fi 子系统（幂等）
    bool EnsureInited();

    // 等 L2 关联 + 等 DHCP 拿到 IP；返回 true 表示可以上网了
    bool WaitAssociated();
    bool WaitGotIp();

    bool connected_ = false;
    bool inited_ = false;
};
