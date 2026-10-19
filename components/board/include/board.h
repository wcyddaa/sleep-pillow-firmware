#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

namespace sleep_pillow { class Network; }

// ============================================================
// 板级抽象层（Board Layer）
// ------------------------------------------------------------
// 本项目支持多硬件底板，采用板级抽象层隔离硬件差异：
//   · 板型是唯一有权定义引脚/总线/时序的地方，封装底层硬件操作
//     （I2S、ADC、GPIO 等驱动调用）；
//   · 上层业务组件（mic/bcg/…）只面向下列统一接口编程，
//     不直接绑定硬件引脚，也不包含 IDF 底层驱动头文件；
//   · 新增底板 = 在 boards/ 下新增一个板型实现文件并在
//     board_registry 注册，上层组件零改动。
// ============================================================

// ---- 麦克风 I2S 采集接口（板型实现）----
// 语义约定：交错立体声帧，每个样本 int32（32bit 槽，高 16bit 为有效数据）。
class BoardMicInput {
public:
    virtual ~BoardMicInput() = default;

    // 打开采集（建通道 + 配时序 + enable）。sample_rate 是上层音频契约。
    // 幂等性由调用方保证：Close 后才允许再次 Open。
    virtual esp_err_t Open(int sample_rate) = 0;
    virtual void Close() = 0;

    // 读一整块交错立体声：frames 帧 = frames×L + frames×R 个 int32。
    // 实际读到的字节数经 bytes_read 返回，小于期望值表示超时/数据不足。
    virtual esp_err_t ReadStereo(int32_t* dst, size_t frames,
                                 size_t* bytes_read, uint32_t timeout_ms) = 0;
};

// ---- ADC 单端输入接口（板型实现）----
// 语义约定：ReadRaw 返回原始计数（本板 12bit → 0~4095）；
// 信号中点即板型偏置电压（见具体板型注释）。
class BoardAdcInput {
public:
    virtual ~BoardAdcInput() = default;

    virtual esp_err_t Open() = 0;
    virtual void Close() = 0;
    virtual esp_err_t ReadRaw(int* raw) = 0;
};

class Board {
public:
    virtual ~Board() = default;

    // 初始化板级硬件
    virtual bool Init() = 0;

    // 返回板名
    virtual const char* GetBoardName() = 0;

    // ---- 统一硬件接口：上层组件向板型索取底层实现 ----
    // 默认返回 nullptr 表示本板未提供该外设；组件侧必须显式报错，
    // 绝不静默回退（与「REAL 模式绝不回退 SIM」原则一致）。

    // 麦克风 I2S 采集（components/mic 消费）
    virtual BoardMicInput* GetMicInput() { return nullptr; }

    // 压电陶瓷 BCG 的 ADC 输入（components/bcg 消费）
    virtual BoardAdcInput* GetBcgAdc() { return nullptr; }

    // 返回网络接口（默认无网络，WifiBoard/BluetoothBoard 覆写）
    virtual sleep_pillow::Network* GetNetwork() { return nullptr; }

    // 单例模式
    static Board* GetInstance() { return instance_; }
    static void SetInstance(Board* inst) { instance_ = inst; }

protected:
    Board() = default;

private:
    inline static Board* instance_ = nullptr;
};

// 强制链接并注册默认板型（由 app_main 调用）
// 具体板型实现位于静态库 libboard.a 中，若无人引用则不会被链接器
// 提取，其全局对象 SleepPillowBoard g_board 的静态构造函数也不会执行，
// 导致 Board::GetInstance() 返回空指针。此函数提供一个显式引用入口。
void BoardEnsureRegistered();
