#include "board_registry.h"
#include "esp_log.h"

static const char* TAG = "BoardRegistry";

// ============================================================
// menuconfig 编译期板型选择
// ------------------------------------------------------------
// 通过 Kconfig 宏在编译期确定默认板型 ID。
// 在 sdkconfig / Kconfig.projbuild 中定义：
//   choice BOARD_TYPE
//       config BOARD_TYPE_SLEEP_PILLOW_V1
//           bool "Sleep Pillow V1"
//       config BOARD_TYPE_SLEEP_PILLOW_V2
//           bool "Sleep Pillow V2"
//       config BOARD_TYPE_ESP_BOX_3
//           bool "ESP-BOX-3"
//   endchoice
// ============================================================

// 编译期宏：CONFIG_BOARD_DEFAULT_ID 由 Kconfig 注入
#ifndef CONFIG_BOARD_DEFAULT_ID
#define CONFIG_BOARD_DEFAULT_ID "sleep_pillow_v1"
#endif

namespace sleep_pillow {

// ============================================================
// 单例获取（Meyers Singleton，C++11 线程安全）
// ============================================================
BoardRegistry& BoardRegistry::GetInstance() {
    static BoardRegistry instance;
    return instance;
}

// ============================================================
// 注册接口
// ============================================================
bool BoardRegistry::RegisterBoard(const char* board_id, BoardCreator creator) {
    if (board_id == nullptr || creator == nullptr) {
        ESP_LOGE(TAG, "注册失败：board_id 或 creator 为空");
        return false;
    }
    return RegisterBoard(std::string(board_id), creator);
}

bool BoardRegistry::RegisterBoard(const std::string& board_id, BoardCreator creator) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = registry_.find(board_id);
    if (it != registry_.end()) {
        ESP_LOGW(TAG, "板型 [%s] 已注册，跳过重复注册", board_id.c_str());
        return false;
    }

    registry_[board_id] = creator;
    ESP_LOGI(TAG, "板型注册成功：[%s]", board_id.c_str());
    return true;
}

// ============================================================
// 创建接口
// ============================================================
Board* BoardRegistry::CreateBoard(const char* board_id) {
    if (board_id == nullptr) {
        ESP_LOGE(TAG, "创建失败：board_id 为空");
        return nullptr;
    }
    return CreateBoard(std::string(board_id));
}

Board* BoardRegistry::CreateBoard(const std::string& board_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = registry_.find(board_id);
    if (it == registry_.end()) {
        ESP_LOGE(TAG, "创建失败：板型 [%s] 未注册", board_id.c_str());
        return nullptr;
    }

    Board* board = it->second();  // 调用创建函数
    if (board == nullptr) {
        ESP_LOGE(TAG, "创建失败：板型 [%s] 创建函数返回空指针", board_id.c_str());
        return nullptr;
    }

    ESP_LOGI(TAG, "板型 [%s] 实例创建成功", board_id.c_str());
    return board;
}

// ============================================================
// 查询接口
// ============================================================
bool BoardRegistry::IsRegistered(const char* board_id) {
    if (board_id == nullptr) return false;
    return IsRegistered(std::string(board_id));
}

bool BoardRegistry::IsRegistered(const std::string& board_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    return registry_.find(board_id) != registry_.end();
}

size_t BoardRegistry::GetRegisteredCount() {
    std::lock_guard<std::mutex> lock(mutex_);
    return registry_.size();
}

// ============================================================
// 获取编译期默认板型 ID
// ------------------------------------------------------------
// 由 menuconfig 的 CONFIG_BOARD_DEFAULT_ID 宏决定，
// 在应用层 Board::GetInstance() 中调用此方法确定默认板型。
// ============================================================
const char* BoardRegistry::GetDefaultBoardId() {
    return CONFIG_BOARD_DEFAULT_ID;
}

}  // namespace sleep_pillow
