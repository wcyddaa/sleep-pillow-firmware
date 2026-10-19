#pragma once

#include <string>
#include <functional>

namespace sleep_pillow {

// ============================================================
// Network：网络接口抽象
// ------------------------------------------------------------
// 定义统一的网络连接接口，Wi-Fi / BLE / 以太网均实现此接口。
// Board 基类通过 GetNetwork() 返回此指针。
// ============================================================
class Network {
public:
    virtual ~Network() = default;

    // 网络类型
    enum class Type {
        kWifi,
        kBluetooth,
        kEthernet,
    };

    // 初始化网络
    virtual bool Init() = 0;

    // 连接
    virtual bool Connect() = 0;

    // 断开
    virtual void Disconnect() = 0;

    // 是否已连接
    virtual bool IsConnected() = 0;

    // 获取网络类型
    virtual Type GetType() = 0;

    // 获取 IP 地址
    virtual std::string GetIpAddress() = 0;
};

}  // namespace sleep_pillow
