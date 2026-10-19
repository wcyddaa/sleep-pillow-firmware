#pragma once

#include <string>
#include <functional>
#include <mutex>
#include <unordered_map>

namespace sleep_pillow {

// 前向声明
class Board;

// ============================================================
// 板型创建函数类型
// ============================================================
using BoardCreator = std::function<Board*()>;

// ============================================================
// BoardRegistry：板型注册中心
// ------------------------------------------------------------
// 负责管理所有板型的注册与动态创建。
// 采用 Meyers 单例（C++11 局部静态变量，天然线程安全），
// 内部用 std::mutex 保护 map 的读写操作。
// ============================================================
class BoardRegistry {
public:
    // -------- 单例访问 --------
    // C++11 保证局部静态变量初始化线程安全
    static BoardRegistry& GetInstance();

    // 禁止拷贝与赋值
    BoardRegistry(const BoardRegistry&) = delete;
    BoardRegistry& operator=(const BoardRegistry&) = delete;

    // -------- 注册接口 --------
    // 注册板型：board_id 唯一标识，creator 为创建函数
    // 返回值：true 注册成功；false board_id 已存在
    bool RegisterBoard(const char* board_id, BoardCreator creator);

    // 重载：支持 std::string 入参
    bool RegisterBoard(const std::string& board_id, BoardCreator creator);

    // -------- 创建接口 --------
    // 根据 board_id 创建板型实例，调用者负责释放内存
    // 未注册返回 nullptr
    Board* CreateBoard(const char* board_id);
    Board* CreateBoard(const std::string& board_id);

    // -------- 查询接口 --------
    // 检查 board_id 是否已注册
    bool IsRegistered(const char* board_id);
    bool IsRegistered(const std::string& board_id);

    // 获取已注册的板型数量
    size_t GetRegisteredCount();

    // 获取编译期默认板型 ID（由 menuconfig 决定）
    static const char* GetDefaultBoardId();

private:
    BoardRegistry() = default;
    ~BoardRegistry() = default;

    std::mutex mutex_;
    std::unordered_map<std::string, BoardCreator> registry_;
};

}  // namespace sleep_pillow

// ============================================================
// DECLARE_BOARD 宏：静态自动注册
// ------------------------------------------------------------
// 在具体板卡实现文件（.cc）末尾使用：
//   DECLARE_BOARD(SleepPillowV1Board, "sleep_pillow_v1")
//
// 原理：利用 C++ 静态对象初始化机制，在 main() 之前
//       向 BoardRegistry 自动注册板型，无需手动调用。
// ============================================================
#define DECLARE_BOARD(board_class, board_id)                              \
    namespace {                                                          \
        struct board_class##_AutoRegister {                              \
            board_class##_AutoRegister() {                               \
                ::sleep_pillow::BoardRegistry::GetInstance().RegisterBoard( \
                    board_id,                                            \
                    []() -> ::sleep_pillow::Board* {                      \
                        return new board_class();                        \
                    });                                                  \
            }                                                            \
        } board_class##_auto_register_;                                  \
    }
    