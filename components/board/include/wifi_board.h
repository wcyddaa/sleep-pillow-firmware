#pragma once

#include "board.h"
#include <string>

namespace sleep_pillow {

// 前向声明
class Network;

// ============================================================
// WifiBoard：Wi-Fi 板型基类
// ------------------------------------------------------------
// 提供 Wi-Fi 连接能力的公共实现，具体板型继承此类
// 即可获得 Wi-Fi 网络接口，无需重复实现连接逻辑。
// ============================================================
class WifiBoard : public Board {
public:
    WifiBoard() = default;
    ~WifiBoard() override = default;

    // -------- Wi-Fi 生命周期 --------

    // 初始化 Wi-Fi（子类 Init() 中调用）
    bool Init() override;

    // 返回 Wi-Fi 网络接口
    Network* GetNetwork() override;

    // -------- Wi-Fi 业务方法 --------

    // 连接 Wi-Fi（阻塞直到连接成功或超时）
    bool ConnectWifi(const std::string& ssid, const std::string& password);

    // 断开 Wi-Fi
    void DisconnectWifi();

    // 检查 Wi-Fi 是否已连接
    bool IsWifiConnected();

    // 获取 IP 地址（未连接返回空字符串）
    std::string GetIpAddress();

protected:
    Network* network_ = nullptr;  // Wi-Fi 网络接口实例
    bool     wifi_initialized_ = false;
};

}  // namespace sleep_pillow
