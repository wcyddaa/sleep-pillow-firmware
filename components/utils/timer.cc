// timer.cc - 定时器工具实现
#include "timer.h"
#include "esp_timer.h"

Timer::Timer() : start_time_us_(0), running_(false) {}

void Timer::Start() {
    start_time_us_ = esp_timer_get_time();
    running_ = true;
}

void Timer::Stop() {
    running_ = false;
}

void Timer::Reset() {
    start_time_us_ = esp_timer_get_time();
}

uint64_t Timer::GetElapsedUs() const {
    if (!running_) return 0;
    return esp_timer_get_time() - start_time_us_;
}

uint32_t Timer::GetElapsedMs() const {
    return static_cast<uint32_t>(GetElapsedUs() / 1000);
}

bool Timer::IsRunning() const {
    return running_;
}
