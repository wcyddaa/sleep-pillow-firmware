#pragma once

#include "board.h"
#include <string>

namespace sleep_pillow {

// 前向声明
class Network;

// ============================================================
// BluetoothBoard：BLE 板型基类
// ------------------------------------------------------------
// 提供 BLE 连接能力的公共实现，具体板型继承此类
// 即可获得蓝牙网络接口，适用于无 Wi-Fi 场景或 BLE 配网。
// ============================================================
class BluetoothBoard : public Board {
public:
    BluetoothBoard() = default;
    ~BluetoothBoard() override = default;

    // -------- BLE 生命周期 --------

    // 初始化 BLE（子类 Init() 中调用）
    bool Init() override;

    // 返回 BLE 网络接口
    Network* GetNetwork() override;

    // -------- BLE 业务方法 --------

    // 启动 BLE 广播
    bool StartAdvertising(const std::string& device_name);

    // 停止广播
    void StopAdvertising();

    // 检查 BLE 是否已初始化
    bool IsBleReady();

protected:
    Network* network_ = nullptr;  // BLE 网络接口实例
    bool     ble_initialized_ = false;
};

}  // namespace sleep_pillow
