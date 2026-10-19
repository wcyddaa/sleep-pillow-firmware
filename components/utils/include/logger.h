// logger.h - 日志工具
#pragma once
#include "esp_log.h"

class Logger {
public:
    static void Info(const char* tag, const char* fmt, ...);
    static void Warn(const char* tag, const char* fmt, ...);
    static void Error(const char* tag, const char* fmt, ...);
    static void Debug(const char* tag, const char* fmt, ...);
    static void SetLevel(const char* tag, esp_log_level_t level);
};
