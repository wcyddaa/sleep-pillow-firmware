#pragma once

namespace sleep_pillow { class Network; }

class Board {
public:
    virtual ~Board() = default;

    // 初始化板级硬件
    virtual bool Init() = 0;

    // 返回板名
    virtual const char* GetBoardName() = 0;

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
