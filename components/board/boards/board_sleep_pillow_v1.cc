#include "board.h"
#include "esp_log.h"

static constexpr const char* TAG = "SleepPillowBoard";

// ------------------------------------------------------------
// SleepPillow v1.0 板型
//
// 注：原 C1001 毫米波雷达**已移除**（2026-10-14）：
//   1) 其 UART 占用 GPIO4/5，与双 INMP441 麦克风的 I2S 引脚
//      (WS=GPIO4 / BCLK=GPIO5) 硬件冲突；
//   2) 雷达串口协议一直未解出（radar_sensor 只发 0x55 试探），
//      "在床" 始终没有可用数据源。
//   GPIO4/5 现让给 I2S 麦克风，「在床」字段保持 -1（未接入）。
// ------------------------------------------------------------

class SleepPillowBoard : public Board {
public:
    SleepPillowBoard() {
        Board::SetInstance(this);
    }

    bool Init() override {
        ESP_LOGI(TAG, "Initializing SleepPillow board...");
        // 本板当前无板级外设需要在此初始化：
        // 音频走 components/mic（INMP441，由 snore 任务在 REAL 模式下按需 Init）
        return true;
    }

    const char* GetBoardName() override {
        return "SleepPillow";
    }
};

// 全局单例实例（静态构造时自动注册到Board::GetInstance()）
static SleepPillowBoard g_board;

// 强制链接入口：被 app_main 引用后，本目标文件才会从 libboard.a 中
// 被提取出来，g_board 才会存在；同时显式注册一次，避免依赖静态
// 构造函数的执行时机。
void BoardEnsureRegistered() {
    Board::SetInstance(&g_board);
}
