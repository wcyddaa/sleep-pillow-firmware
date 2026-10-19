// app_config.h - 转发到 v2.0 端侧三开关 app_config.h
// ============================================================
// v1.0 这里原本放 Wi-Fi SSID / 设备 ID / 主题常量等基础宏,
// 但与 components/app_config/include/app_config.h 同名,
// main 的 INCLUDE_DIRS 在前,导致 v2.0 的 EdgeMode/AppConfigStore 类型被遮蔽。
//
// 现在统一转发到 v2.0 版本(EdgeMode / AppConfig / AppConfigStore)。
// v1.0 的 CLOUD_xxx / BLE_xxx 宏保留在这里（只有 main/app_main.cpp 使用）。
//
// 注：以下宏已删除（对应功能已废弃）：
//   WIFI_SSID / WIFI_PASSWORD  —— 配网改走 BLE（手机下发加密凭据）
//   SENSOR_READ_MS             —— 旧的传感器轮询间隔
//   CLOUD_REPORT_DEVICE_ID     —— 上报 device_id 直接用 CLOUD_DEVICE_ID
//   CLOUD_PUBLISH_MS           —— 上报窗口固定 30s（EdgeAggregator::kWindowSec）
// ============================================================
#pragma once

#define BLE_DEVICE_NAME "SleepPillow"

// ====== 云端接入（华为云 IoTDA MQTT）======
// broker 地址取控制台「总览 -> 接入信息」里的 MQTT 协议地址
#define CLOUD_MQTT_HOST         "6330a71a21.st1.iotda-device.cn-north-4.myhuaweicloud.com"
#define CLOUD_MQTT_PORT         1883

// 平台设备 ID 与密钥：真实值放 secrets.h（已被 .gitignore 忽略，不入库）。
// 首次克隆后：复制 secrets.h.example 为 secrets.h，
// 再填控制台「设备 -> 设备详情 -> MQTT 连接参数」里的值。
#include "secrets.h"
#if !defined(CLOUD_DEVICE_ID) || !defined(CLOUD_DEVICE_SECRET)
#error "缺少 main/include/secrets.h：请复制 secrets.h.example 为 secrets.h 并填写 IoTDA 设备 ID 与密钥"
#endif

// 0 = HMACSHA256 不校验时间戳（设备长期在线推荐），1 = 校验时间戳
#define CLOUD_SIGN_TYPE         0

// 日志级别
#define APP_LOG_LEVEL   ESP_LOG_INFO

// 转发到 v2.0 端侧三开关头(EdgeMode / AppConfig / AppConfigStore)
// 相对路径:本文件在 main/include/,要上溯两级才到项目根
#include "../../components/app_config/include/app_config.h"
