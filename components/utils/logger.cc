// logger.cc - 日志工具实现
#include "logger.h"
#include <stdarg.h>
#include <stdio.h>

static char format_buf[512];

static const char* format_msg(const char* fmt, va_list args) {
    vsnprintf(format_buf, sizeof(format_buf), fmt, args);
    return format_buf;
}

void Logger::Info(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ESP_LOGI(tag, "%s", format_msg(fmt, args));
    va_end(args);
}

void Logger::Warn(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ESP_LOGW(tag, "%s", format_msg(fmt, args));
    va_end(args);
}

void Logger::Error(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ESP_LOGE(tag, "%s", format_msg(fmt, args));
    va_end(args);
}

void Logger::Debug(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    ESP_LOGD(tag, "%s", format_msg(fmt, args));
    va_end(args);
}

void Logger::SetLevel(const char* tag, esp_log_level_t level) {
    esp_log_level_set(tag, level);
}
