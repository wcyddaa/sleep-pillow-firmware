// edge_aggregator.cc - 30s 结构化数据聚合器实现
#include "edge_aggregator.h"
#include "app_config.h"
#include "cloud_client.h"   // v1.0 用 CloudClient 单例发 frame/cfg
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "offline_queue.h"  // 上行失败 → 落盘离线队列，重连后补传
#include <inttypes.h>
#include <time.h>
#include <cstdio>
#include <cstring>

namespace sleep_pillow {

namespace {
inline time_t NowSec() {
    return time(nullptr);
}
}  // namespace

EdgeAggregator* EdgeAggregator::GetInstance() {
    static EdgeAggregator inst;
    return &inst;
}

void EdgeAggregator::SetSources(SnoreInfer* snore, StageInfer* stage, BcgAdc* bcg) {
    snore_ = snore;
    stage_ = stage;
    bcg_   = bcg;
}

// 结束当前鼾声事件：命中秒数 ≥kMinEventSec 才算一次事件入 pending，
// 否则视为偶发噪声丢弃。须持 snore_evt_lock_ 调用；**临界区内不打日志**。
bool EdgeAggregator::CloseSnoreEventLocked(SnoreEventSpan* out_ev) {
    if (out_ev) { out_ev->s = 0; out_ev->e = 0; }
    if (!evt_open_) return false;
    evt_open_ = 0;
    if (evt_secs_ < kMinEventSec) return false;   // 太短：偶发单秒噪声，不算事件

    if (evt_pending_cnt_ >= kMaxPendingEvents) {
        // 异常场景兜底（正常一窗口最多 ~4 次事件）：丢最老保最新
        memmove(evt_pending_, evt_pending_ + 1,
                (kMaxPendingEvents - 1) * sizeof(SnoreEventSpan));
        evt_pending_cnt_ = kMaxPendingEvents - 1;
    }
    const SnoreEventSpan ev{(uint32_t)evt_start_, (uint32_t)evt_last_hit_};
    evt_pending_[evt_pending_cnt_++] = ev;
    if (out_ev) *out_ev = ev;
    return true;
}

void EdgeAggregator::OnSnoreProb(float prob) {
    // 每秒调用一次：既累计窗口秒数，也记下"这一秒是否在打鼾"供实时区用
    // 注：只有**真正拿到有效音频**才会调到这里，所以顺便统计有效秒数，
    //     用于区分"没鼾声"和"没数据"。
    const time_t now = time(nullptr);
    const int hit = (prob >= 0.5f) ? 1 : 0;

    ++snore_valid_cnt_;
    last_snore_hit_ = hit;
    snore_sec_cnt_ += hit;

    // ★ 鼾声事件状态机（端侧事实，后端"次数/平均间隔"直接由此统计）：
    //   命中 → 开启/延续事件；距上次命中 ≥kMergeGapSec → 事件结束。
    //   事件跨窗口是常态：结束前不进 pending，结束所在窗口随帧上行。
    if (hit) {
        if (!evt_open_) {
            evt_open_  = 1;
            evt_start_ = now;
            evt_secs_  = 0;
        }
        ++evt_secs_;
        evt_last_hit_ = now;
    } else if (evt_open_ && now - evt_last_hit_ >= kMergeGapSec) {
        SnoreEventSpan ev{};
        portENTER_CRITICAL(&snore_evt_lock_);
        const bool recorded = CloseSnoreEventLocked(&ev);
        portEXIT_CRITICAL(&snore_evt_lock_);
        if (recorded) {
            ESP_LOGI(TAG, "鼾声事件: %us ~ %us（间隔 >=%ds 结束）",
                     (unsigned)ev.s, (unsigned)ev.e, kMergeGapSec);
        }
    }
}

void EdgeAggregator::OnStageResult(int stage_code) {
    last_stage_ = stage_code;
}

void EdgeAggregator::OnBcgDerived() {
    if (!bcg_) return;
    auto d = bcg_->GetDerived();
    if (d.heart_rate > 0) last_hr_ = d.heart_rate;
    if (d.resp_rate  > 0) last_rr_  = d.resp_rate;
    last_motion_ = d.motion ? 1 : 0;
}

void EdgeAggregator::LiveSnapshot(int* snoring, int* in_bed,
                                  int* hr, int* rr, int* motion,
                                  int* snore_sec, int* stage) const {
    const AppConfig& c = AppConfigStore::GetInstance()->Get();

    // 与 30s 帧同一套语义：通道 OFF → -1
    if (snoring)   *snoring   = (c.snore_mode == EdgeMode::kOff) ? -1 : last_snore_hit_;
    // 「在床」目前**没有数据源**：C1001 毫米波雷达已移除（其 UART 占用
    // GPIO4/5，与双 INMP441 麦克风的 I2S 引脚冲突，且协议一直未解出），
    // BCG 压电陶瓷也还没接硬件。所以先恒为 -1，前端按"未接入"显示。
    if (in_bed)    *in_bed    = -1;
    if (hr)        *hr        = (c.bcg_mode   == EdgeMode::kOff) ? -1 : last_hr_;
    if (rr)        *rr        = (c.bcg_mode   == EdgeMode::kOff) ? -1 : last_rr_;
    if (motion)    *motion    = (c.bcg_mode   == EdgeMode::kOff) ? -1 : last_motion_;
    if (stage)     *stage     = (c.stage_mode == EdgeMode::kOff) ? -1 : last_stage_;
    if (snore_sec) *snore_sec = (c.snore_mode == EdgeMode::kOff) ? -1 : snore_sec_cnt_;
}

void EdgeAggregator::ResetWindow() {
    win_start_ts_      = time(nullptr);
    snore_sec_cnt_     = 0;
    snore_valid_cnt_   = 0;
    // 保留 last_stage/last_hr/last_rr/last_motion 的最近值
    // 防止窗口结束时正好是切换瞬间
}

void EdgeAggregator::PublishCurrentFrame() {
    AppConfigStore* cfg = AppConfigStore::GetInstance();
    const AppConfig& c = cfg->Get();

    // ★ 冗余保险：cfg 可能在窗口期间被改成全 OFF，这种帧不上行
    if (c.snore_mode == sleep_pillow::EdgeMode::kOff &&
        c.stage_mode == sleep_pillow::EdgeMode::kOff &&
        c.bcg_mode   == sleep_pillow::EdgeMode::kOff) {
        ESP_LOGD(TAG, "cfg 已全 OFF，跳过本帧");
        return;
    }

    // ★ NTP 未同步 → 不上帧（避免把 boot 秒当时间戳发出去 → 后端/前端显示错乱）
    if (!CloudClient::GetInstance().IsTimeSynced()) {
        ESP_LOGW(TAG, "NTP 未同步，跳过本帧（避免错误时间戳）");
        return;
    }

    const time_t now = time(nullptr);
    SleepFrame f{};
    f.window_start_ts = (uint32_t)win_start_ts_;
    f.window_end_ts   = (uint32_t)now;
    // ★ 本窗口一帧有效音频都没拿到 → 报 -1（不可用），
    //   而不是 0（没有鼾声）。否则麦克风故障会被前端误读成"睡得很好"。
    f.snore_seconds = (c.snore_mode == EdgeMode::kOff || snore_valid_cnt_ == 0)
                          ? -1
                          : (int8_t)snore_sec_cnt_;
    f.stage           = (c.stage_mode == EdgeMode::kOff) ? -1 : (int8_t)last_stage_;
    f.heart_rate      = (c.bcg_mode   == EdgeMode::kOff) ? -1 : (int8_t)last_hr_;
    f.resp_rate       = (c.bcg_mode   == EdgeMode::kOff) ? -1 : (int8_t)last_rr_;
    f.motion          = (c.bcg_mode   == EdgeMode::kOff) ? -1 : (int8_t)last_motion_;

    f.flags = 0;
    if (c.snore_mode == EdgeMode::kOff) f.flags |= 0x01;
    if (c.stage_mode == EdgeMode::kOff) f.flags |= 0x02;
    if (c.bcg_mode   == EdgeMode::kOff) f.flags |= 0x04;

    // ★ 收集随本帧上行的鼾声事件（事件挂在「结束所在窗口」的帧上）：
    //   snore 通道被关 → 打开中的事件就地收尾；然后把 pending 整体取走清空。
    SnoreEventSpan evs[kMaxPendingEvents];
    size_t nev = 0;
    portENTER_CRITICAL(&snore_evt_lock_);
    if (c.snore_mode == EdgeMode::kOff) {
        SnoreEventSpan discarded;   // 收尾事件进 pending 即可，span 本身不需要
        CloseSnoreEventLocked(&discarded);
    }
    nev = evt_pending_cnt_;
    memcpy(evs, evt_pending_, nev * sizeof(SnoreEventSpan));
    evt_pending_cnt_ = 0;
    portEXIT_CRITICAL(&snore_evt_lock_);

    // snore_events 追加段：无事件时保持空串（JSON 紧凑，不含该字段）；
    // 有事件时形如 ,"snore_events":[{"s":..,"e":..},{..}]
    char evbuf[320] = "";
    size_t evlen = 0;
    if (nev > 0) {
        const int h = snprintf(evbuf, sizeof(evbuf), ",\"snore_events\":[");
        evlen = (h > 0) ? (size_t)h : 0;
        for (size_t i = 0; i < nev && evlen > 0; ++i) {
            char one[48];
            const int w1 = snprintf(one, sizeof(one), "%s{\"s\":%u,\"e\":%u}",
                                    (i == 0) ? "" : ",",
                                    (unsigned)evs[i].s, (unsigned)evs[i].e);
            if (w1 <= 0) break;
            if (evlen + (size_t)w1 + 2 > sizeof(evbuf)) {   // +2: 留 ']' 与 '\0'
                ESP_LOGW(TAG, "snore_events 超出缓冲，丢弃多余事件");
                break;
            }
            memcpy(evbuf + evlen, one, (size_t)w1);
            evlen += (size_t)w1;
        }
        if (evlen > 0) {
            evbuf[evlen++] = ']';
            evbuf[evlen]   = '\0';
        } else {
            evbuf[0] = '\0';
        }
    }

    // device_id 显式带上：不依赖 IoTDA 规则引擎转发的报文信封结构，
    // 后端 /ingest/huawei/frame 直接读顶层 device_id 即可定位设备。
    const std::string& dev_id = CloudClient::GetInstance().report_device_id();
    const int n = snprintf(f.json, sizeof(f.json),
        "{\"device_id\":\"%s\",\"evt\":\"frame\","
        "\"start\":%" PRIu32 ",\"end\":%" PRIu32 ",\"stage\":%d,\"hr\":%d,\"rr\":%d,"
        "\"motion\":%d,\"snore_sec\":%d,\"flags\":%u,"
        "\"cfg\":{\"snore\":%d,\"stage\":%d,\"bcg\":%d}%s}",
        dev_id.c_str(),
        f.window_start_ts, f.window_end_ts,
        (int)f.stage, (int)f.heart_rate, (int)f.resp_rate,
        (int)f.motion, (int)f.snore_seconds, (unsigned)f.flags,
        (int)c.snore_mode, (int)c.stage_mode, (int)c.bcg_mode,
        evbuf);
    f.json_len = (n > 0 && n < (int)sizeof(f.json)) ? n : 0;

    latest_ = f;

    // 组装失败（缓冲不够被截断）→ JSON 不完整，宁可丢这一帧也不发坏数据
    if (f.json_len == 0) {
        ESP_LOGE(TAG, "30s 帧 JSON 组装失败（截断），本帧跳过");
        return;
    }

    // 上行 MQTT：/device/{id}/frame
    // 失败（未连接/发送出错）→ 落 SPIFFS 离线队列，MQTT 重连后按序补传。
    // 注意：NTP 未同步的帧在上方已提前 return，不会进入离线队列
    //（boot 秒占位的帧补传上去也只会污染时间轴）。
    if (!CloudClient::GetInstance().PublishFrame(f.json)) {
        OfflineQueue::GetInstance().Push(f.json, f.json_len);
    }
    ESP_LOGI(TAG, "30s 帧: stage=%d hr=%d rr=%d motion=%d snore_sec=%d flags=0x%02x",
             (int)f.stage, (int)f.heart_rate, (int)f.resp_rate,
             (int)f.motion, (int)f.snore_seconds, (unsigned)f.flags);
}

esp_err_t EdgeAggregator::Start() {
    if (running_) return ESP_OK;
    // ★ 三开关全 OFF → 不启动聚合器，不上云任何数据
    using sleep_pillow::EdgeMode;
    const auto& c = sleep_pillow::AppConfigStore::GetInstance()->Get();
    if (c.snore_mode == EdgeMode::kOff &&
        c.stage_mode == EdgeMode::kOff &&
        c.bcg_mode   == EdgeMode::kOff) {
        ESP_LOGI(TAG, "三开关全 OFF，聚合器不启动、不上云");
        return ESP_OK;
    }

    // ★ 宽限期：不在开关刚打开的那一刻就开始 30s 封装，而是先等 kGraceSec 秒。
    //   这 kGraceSec 秒内如果用户又打开了别的开关，会一并进入同一个 30s 窗口；
    //   过了这 kGraceSec 才开始计时，之后再改开关要等下一个窗口。
    //   —— 避免"刚开一个开关就立刻开始算，结果窗口里只有部分通道"。
    grace_until_ts_ = NowSec() + kGraceSec;
    running_ = true;
    ESP_LOGI(TAG, "聚合器启动，%d 秒后开始第一个 30s 窗口", kGraceSec);

    // 起 1s tick 任务
    xTaskCreate([](void* arg) {
        auto* self = static_cast<EdgeAggregator*>(arg);
        while (self->running_) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            self->OnBcgDerived();
            const time_t now = NowSec();

            if (self->grace_until_ts_ != 0) {
                // 宽限期内：只收集各通道的最新值，不判断窗口边界
                if (now >= self->grace_until_ts_) {
                    self->grace_until_ts_ = 0;
                    self->ResetWindow();
                    ESP_LOGI(TAG, "宽限期结束，开始 30s 窗口");
                }
                continue;
            }

            if (now - self->win_start_ts_ >= kWindowSec) {
                self->PublishCurrentFrame();
                self->ResetWindow();
            }
        }
        vTaskDelete(nullptr);
    }, "edge_agg", 4096, this, 3, &task_);
    return ESP_OK;
}

void EdgeAggregator::Stop() {
    running_ = false;
    grace_until_ts_ = 0;
    // 清理鼾声事件状态：未结束的事件直接丢弃（不算完整事实）
    portENTER_CRITICAL(&snore_evt_lock_);
    evt_open_ = 0;
    evt_pending_cnt_ = 0;
    portEXIT_CRITICAL(&snore_evt_lock_);
    if (task_) { vTaskDelete(task_); task_ = nullptr; }
}

}  // namespace sleep_pillow
