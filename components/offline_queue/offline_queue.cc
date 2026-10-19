// offline_queue.cc - SPIFFS 环形队列实现（file-per-frame）
#include "offline_queue.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/task.h"

namespace sleep_pillow {

OfflineQueue& OfflineQueue::GetInstance() {
    static OfflineQueue inst;
    return inst;
}

esp_err_t OfflineQueue::Init() {
    if (mounted_) return ESP_OK;

    const esp_vfs_spiffs_conf_t conf = {
        .base_path              = kMountPoint,
        .partition_label        = kPartition,
        .max_files              = 2,
        .format_if_mount_failed = true,   // 首次开机/损坏时格式化，保证可用
    };
    const esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS 挂载失败: %s（离线缓存不可用，断网仍会丢帧）",
                 esp_err_to_name(err));
        return err;
    }
    mounted_ = true;
    lock_ = xSemaphoreCreateMutex();

    // 扫描现存文件恢复状态：最老 = 最小序号；顺带清掉上次掉电残留的临时文件。
    // 正常运行无空洞（只从头部删除、尾部追加），有空洞也以实际文件为准。
    uint32_t min_seq = 0, max_seq = 0;
    size_t   n = 0;
    bool     have_any = false;
    DIR* dir = opendir(kMountPoint);
    if (dir != nullptr) {
        struct dirent* ent;
        while ((ent = readdir(dir)) != nullptr) {
            if (strcmp(ent->d_name, ".tmp") == 0) {
                unlink(kTempPath);
                continue;
            }
            unsigned seq = 0;
            if (sscanf(ent->d_name, "f_%u.bin", &seq) != 1) continue;
            if (!have_any || seq < min_seq) min_seq = seq;
            if (!have_any || seq > max_seq) max_seq = seq;
            have_any = true;
            ++n;
        }
        closedir(dir);
    }

    if (have_any) {
        front_seq_ = min_seq;
        next_seq_  = max_seq + 1;
        count_     = n;
        ESP_LOGI(TAG, "恢复离线缓存：%u 帧待补传（seq %u..%u）",
                 (unsigned)count_, (unsigned)min_seq, (unsigned)max_seq);
    } else {
        ESP_LOGI(TAG, "离线缓存就绪（空队列，容量 %u 帧）", (unsigned)kCapacity);
    }
    return ESP_OK;
}

void OfflineQueue::PathOf(uint32_t seq, char* buf, size_t buf_size) const {
    snprintf(buf, buf_size, "%s/f_%06u.bin", kMountPoint, (unsigned)seq);
}

bool OfflineQueue::Push(const char* data, size_t len) {
    if (!mounted_ || data == nullptr || len == 0) return false;
    if (xSemaphoreTake(lock_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;

    // 队列满 → 丢最老一帧腾位（环形语义：保最新数据）
    if (count_ >= kCapacity) {
        char path[64];
        PathOf(front_seq_, path, sizeof(path));
        unlink(path);
        ++front_seq_;
        --count_;
        ESP_LOGW(TAG, "队列已满（%u 帧），丢弃最老一帧", (unsigned)kCapacity);
    }

    // 临时文件 + rename：保证掉电不会留下半个坏帧
    unlink(kTempPath);
    FILE* fp = fopen(kTempPath, "wb");
    if (fp == nullptr) {
        xSemaphoreGive(lock_);
        ESP_LOGE(TAG, "Push：打开临时文件失败");
        return false;
    }
    const size_t w = fwrite(data, 1, len, fp);
    fclose(fp);
    if (w != len) {
        unlink(kTempPath);
        xSemaphoreGive(lock_);
        ESP_LOGE(TAG, "Push：写入不完整 %u/%u", (unsigned)w, (unsigned)len);
        return false;
    }

    char path[64];
    PathOf(next_seq_, path, sizeof(path));
    if (rename(kTempPath, path) != 0) {
        unlink(kTempPath);
        xSemaphoreGive(lock_);
        ESP_LOGE(TAG, "Push：rename 失败");
        return false;
    }

    if (count_ == 0) front_seq_ = next_seq_;
    ++next_seq_;
    ++count_;
    xSemaphoreGive(lock_);
    return true;
}

bool OfflineQueue::PeekOldest(char* buf, size_t buf_size, size_t* out_len) {
    if (out_len) *out_len = 0;
    if (!mounted_ || buf == nullptr || buf_size == 0) return false;
    if (xSemaphoreTake(lock_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    if (count_ == 0) {
        xSemaphoreGive(lock_);
        return false;
    }

    char path[64];
    PathOf(front_seq_, path, sizeof(path));
    FILE* fp = fopen(path, "rb");
    if (fp == nullptr) {
        // 异常掉电可能留下空洞 → 剔除该帧，下次 Peek 轮到下一帧
        unlink(path);
        ++front_seq_;
        --count_;
        xSemaphoreGive(lock_);
        ESP_LOGW(TAG, "最老帧缺失，已剔除");
        return false;
    }
    const size_t r = fread(buf, 1, buf_size - 1, fp);
    fclose(fp);
    buf[r] = '\0';
    if (r == 0) {
        // 空文件视为损坏 → 剔除，调用方重试
        unlink(path);
        ++front_seq_;
        --count_;
        xSemaphoreGive(lock_);
        ESP_LOGW(TAG, "最老帧为空，已剔除");
        return false;
    }
    if (out_len) *out_len = r;
    xSemaphoreGive(lock_);
    return true;
}

bool OfflineQueue::PopOldest() {
    if (!mounted_) return false;
    if (xSemaphoreTake(lock_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    if (count_ == 0) {
        xSemaphoreGive(lock_);
        return false;
    }
    char path[64];
    PathOf(front_seq_, path, sizeof(path));
    unlink(path);
    ++front_seq_;
    --count_;
    xSemaphoreGive(lock_);
    return true;
}

}  // namespace sleep_pillow
