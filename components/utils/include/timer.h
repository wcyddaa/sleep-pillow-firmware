// timer.h - 定时器工具
#pragma once
#include <stdint.h>

class Timer {
public:
    Timer();

    void Start();
    void Stop();
    void Reset();

    uint32_t GetElapsedMs() const;
    uint64_t GetElapsedUs() const;
    bool IsRunning() const;

private:
    uint64_t start_time_us_;
    bool running_;
};
