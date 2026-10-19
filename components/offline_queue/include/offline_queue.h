// offline_queue.h - 断网离线帧缓存（SPIFFS 环形队列）
// ============================================================
// 解决「断网即丢数据」：30s 帧 PublishFrame 失败时落盘到 SPIFFS
// 分区 store（256 帧 ≈ 2 小时），MQTT 重连成功后按序补传。
//
// 结构：file-per-frame
//   /store/f_000123.bin   序号单调递增，最老 = 序号最小
//   队列满 → 丢最老帧腾位（环形语义，优先保最新 2 小时数据）
//   写入走 临时文件 + rename，掉电不会留下半个坏帧
//
// 帧自带 start/end 时间戳，后端按 (device_id, start) 幂等去重，
// 补传造成的重复/乱序不会污染数据。
//
// 使用方：
//   EdgeAggregator::PublishCurrentFrame 失败 → Push
//   CloudClient MQTT 重连成功 → PeekOldest/PublishFrame/PopOldest 循环补传
// ============================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace sleep_pillow {

class OfflineQueue {
public:
    static OfflineQueue& GetInstance();

    // 挂载 /store 分区 + 扫描恢复队列状态（幂等，重复调用直接返回）。
    // 失败只影响离线缓存（退化为断网丢帧），不阻塞调用方。
    esp_err_t Init();

    // 当前积压帧数（0 = 无积压）。
    // 读 32 位对齐变量，Xtensa 上天然原子；仅用于判断/日志。
    size_t Count() const { return count_; }

    // 入队一帧。队列满时丢最老一帧腾位（环形语义）。
    bool Push(const char* data, size_t len);

    // 读最老一帧内容（不删除）。
    // 返回 false = 队列空；或最老帧缺失/损坏已被自动剔除（调用方可重试）。
    bool PeekOldest(char* buf, size_t buf_size, size_t* out_len);

    // 删除最老一帧（Peek + 上行成功后才调用）
    bool PopOldest();

private:
    OfflineQueue() = default;

    // 按序号生成文件路径 /store/f_%06u.bin
    void PathOf(uint32_t seq, char* buf, size_t buf_size) const;

    bool             mounted_  = false;
    uint32_t         next_seq_ = 0;   // 下一帧写入用的序号（单调递增）
    uint32_t         front_seq_ = 0;  // 最老帧序号（count_==0 时无意义）
    size_t           count_    = 0;   // 当前积压帧数
    SemaphoreHandle_t lock_    = nullptr;  // Push（聚合器任务）/Peek/Pop（补传任务）互斥

    static constexpr size_t      kCapacity    = 256;     // 256 帧 ≈ 2 小时
    static constexpr const char* kMountPoint  = "/store";
    static constexpr const char* kPartition   = "store";
    static constexpr const char* kTempPath    = "/store/.tmp";
    static constexpr const char* TAG          = "OfflineQueue";
};

}  // namespace sleep_pillow
