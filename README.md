# 智眠智能睡眠枕固件 (sleep-pillow)

ESP32-S3 端侧智能睡眠枕固件：在 MCU 上直接运行三个 int8 神经网络（鼾声识别 / 睡眠分期 / 体动检测），每 30 秒聚合一帧结构化数据经 MQTT 上报华为云 IoTDA，并支持 BLE 配网与每秒实时推送。

- 目标芯片：ESP32-S3（N16R8：16MB Flash + 8MB PSRAM）
- 框架：ESP-IDF v5.5+ / FreeRTOS / C++ 面向对象（组件化 + 板级抽象）
- 关联仓库：[`sleep-monitor-backend`](https://github.com/wcyddaa/sleep-monitor-backend)（云后端）、[`sleep-pillow-miniapp`](https://github.com/wcyddaa/sleep-pillow-miniapp)（微信小程序）

---

## 功能特性

- **板级抽象层**：本项目支持多硬件底板，采用板级抽象层隔离硬件差异。上层业务组件使用统一硬件接口（`BoardMicInput` / `BoardAdcInput`），不直接绑定硬件引脚，也不包含 IDF 底层驱动头文件；I2S/ADC 等硬件相关底层操作全部由板级抽象层封装。新增底板 = 在 `boards/` 下新增一个板型实现，上层组件零改动。当前版本仅完成 ESP32-S3 底板的适配。
- **端侧三模型推理**（自研 int8 前向引擎，不依赖 TFLite Micro）：
  - 鼾声识别：1s/16kHz 音频 → 64 维手工特征 → int8 MLP（64→48→24→2）
  - 睡眠分期：BCG 派生特征，每 30s 推理一次
  - 体动检测：140Hz BCG 采样 + 心率/呼吸派生
- **运行时三态开关**：每个模型独立 `OFF / SIM / REAL` 三态，NVS 持久化，BLE 下发即生效，重启保持
- **BLE 配网与实时推送**：NimBLE，AES-128-GCM 加密下发 Wi-Fi 凭据；每秒推送实时心率/呼吸/鼾声/体动/分期
- **华为云 IoTDA 上云**：MQTT 直连 + HMAC-SHA256 签名，30s 结构化帧上行
- **无数据伪装**：通道未启用或无有效数据时上报 `-1` 并以 `flags` 位标记，而不是 `0`

## 硬件组成

| 外设 | 接口 | 说明 |
|------|------|------|
| 双 INMP441 数字麦克风 | I2S0（BCLK=GPIO5, WS=GPIO4, DIN=GPIO6） | 16kHz/16bit 立体声；近场选麦——两路能量接近取平均（降噪），一路明显更响取该路；仅支持 3.3V 供电 |
| 压电陶瓷传感器（BCG） | ADC | 140Hz 采样，派生心率/呼吸/体动 |
| MAX98357 功放 | I2S1（BCLK=GPIO7, LRC=GPIO8, DIN=GPIO9） | 预留未接线，播放路径未实现 |

> 原 C1001 毫米波雷达已移除（UART 引脚与 I2S 冲突且串口协议未解出），因此 `in_bed`（在床）字段恒为 `-1`。

Flash 分区：`nvs` 24KB / `phy_init` 4KB / `factory` 3MB（无 OTA 分区）。

## 目录结构

```
sleep-pillow/
├── main/                        # 应用装配点
│   ├── app_main.cpp             # AppMain::Run()：启动流程唯一入口
│   └── include/
│       ├── app_config.h         # 云端 broker 地址等宏 + 转发三开关定义
│       ├── secrets.h.example    # 凭据模板（复制为 secrets.h，不入库）
│       └── app_main.h
├── components/
│   ├── app_config/              # EdgeMode / AppConfigStore（三开关 NVS 持久化）
│   ├── board/                   # 板级抽象层：统一硬件接口 + 注册表（工厂模式）
│   │   ├── include/board.h      #   BoardMicInput / BoardAdcInput 接口定义
│   │   └── boards/board_esp32s3.cc   # ESP32-S3 底板：I2S/ADC 底层封装 + 引脚定义
│   ├── mic/                     # 双 INMP441 近场选麦合成单声道（业务逻辑，硬件无关）
│   ├── bcg/                     # 压电 BCG 派生心率/呼吸/体动（业务逻辑，硬件无关）
│   ├── snore_infer/             # 鼾声推理（特征提取与训练端逐字对齐，内置 |z| 自检）
│   ├── stage_infer/             # 睡眠分期推理 + StageFeatureBuilder
│   ├── motion_infer/            # 体动推理
│   ├── models/                  # 自研 int8 推理运行时（nn_ops + model_nets + 权重）
│   │   └── tools/export_onnx_to_c.py   # ONNX → C 权重数组转换脚本
│   ├── edge_aggregator/         # 30s 窗口聚合（含 5s 宽限期）+ 帧组装
│   ├── cloud/                   # CloudClient：华为云 IoTDA MQTT 接入
│   ├── network/                 # wifi_manager + ble_provision（NimBLE + AES-128-GCM）
│   ├── audio/                   # 预留桩：ES8311/ES7210 codec + esp_sr 唤醒词（未参与构建）
│   └── utils/                   # logger / timer
├── partitions.csv
├── sdkconfig                    # 实际生效配置（有意入库，不含凭据）
└── sdkconfig.defaults
```

## 启动流程（AppMain::Run）

1. NVS 初始化（含分区擦除重建）→ `AppConfigStore` 载入三开关——必须最早执行，否则三开关不落盘、重启回全 OFF
2. Board 初始化；触发 `SnoreNet / StageNet / MotionNet` 单例构造
3. `ApplyEdgeCfg()`：按**已保存的** cfg 启停三个模型任务与聚合器（重启后无需手机重发）
4. 启动 BLE 配网（注册 Wi-Fi 凭据 / 采样 / 三开关回调）+ 每秒 BLE 实时推送任务
5. `AutoConnectTask`：开机自动连接已保存 Wi-Fi → NTP 校时 → 连接 IoTDA
6. 主循环 idle；实际工作由 FreeRTOS 任务承担

任务内部循环检查 cfg：切回 `OFF` 即自行退出；`ApplyEdgeCfg` 随时可由 BLE `SET_CFG` 触发重入。

## 三态开关

| 任务 | 开关 | SIM（合成数据） | REAL（真实数据） |
|------|------|----------------|------------------|
| 鼾声 | `snore_mode` | 固件合成 1s PCM | INMP441 采集 → 近场单声道 |
| 分期 | `stage_mode` | 合成 IHR/Resp/Motion 特征 | BCG 派生特征 |
| 体动 | `bcg_mode` | 合成 140Hz 信号 | 压电 ADC |

三开关为设备本地状态，手机经 BLE 读写（`0x05 SET_CFG`），同时作为字段随每帧上行。三开关全 OFF 时聚合器停止、不上云任何数据。

## 数据契约

**30s 结构化帧**（MQTT 上行至 `$oc/devices/{id}/sys/messages/up`，NTP 校时成功后才开始）：

```json
{"device_id":"...","evt":"frame","start":...,"end":...,"stage":..,"hr":..,
 "rr":..,"motion":..,"snore_sec":..,"flags":..,
 "cfg":{"snore":..,"stage":..,"bcg":..}}
```

- 顶层显式携带 `device_id`，后端 `/api/v1/ingest/huawei` 直接读取
- `snore_sec = -1` 表示窗口内无有效音频（不可用），区别于 `0`（整秒未检测到鼾声）；`flags` 位 0x01/0x02/0x04 对应三开关 OFF
- 鼾声按秒累加：单秒判定概率 `≥0.5` 计 1 秒

**BLE 实时推送**（每秒，手机未连接时静默丢弃）：

```json
{"evt":"rt","snoring":..,"in_bed":..,"hr":..,"rr":..,"motion":..,"snore":..,"stage":..}
```

**BLE 配网帧协议**：`[AA 55][CMD][LEN][PAYLOAD][SUM]`；Wi-Fi 凭据为 AES-128-GCM 密文（IV 12B + TAG 16B）。两端契约见 `components/network/ble_provision.h` 与小程序 `utils/bluetooth/BLEDataParser.js`。

## 构建与烧录

```bash
# 安装 ESP-IDF v5.5+ 后：
idf.py set-target esp32s3      # 首次
idf.py build
idf.py -p <PORT> flash monitor # 烧录 + 串口监控
idf.py fullclean && idf.py build  # 清理重建
```

## 配置云端凭据（首次克隆必做）

设备 ID 与密钥不入库。复制模板并填入华为云 IoTDA 控制台「设备详情 → MQTT 连接参数」中的值：

```
cp main/include/secrets.h.example main/include/secrets.h
```

```c
// main/include/secrets.h
#define CLOUD_DEVICE_ID         "your-iotda-device-id"
#define CLOUD_DEVICE_SECRET     "your-iotda-device-secret"
```

`secrets.h` 已被 `.gitignore` 的 `secret*.h` 规则忽略；未创建时编译会 `#error` 提示。broker 地址（`CLOUD_MQTT_HOST`，非机密）在 `main/include/app_config.h`。

## 模型更新流程

端侧模型权重由训练侧工程导出，**`components/models/models/*.cc` 为自动生成文件，禁止手改**：

```
1. snoring-classify 工程：python build_edge_snore.py      → edge_snore_mlp_int8.onnx
2. python components/models/tools/export_onnx_to_c.py     → model_snore.cc
3. 替换 components/models/models/ 下同名文件后 idf.py build && flash
4. 自检：开机日志查看 sanity 输出；运行时每帧特征 |z| < 6 为对齐正常
```

## 已知限制

- `audio/` 组件为预留桩（codec / 唤醒词未实现，未参与构建）
- 蓝牙板型（BluetoothBoard）已排除编译；启用需开 `CONFIG_BT_ENABLED` 并加回 CMake 依赖
- 运行中 Wi-Fi 掉线无自动重连（仅开机自动连接）；无 OTA 分区
- `in_bed` 无数据源（雷达已移除），恒为 `-1`

## 版本

- **v1.0+**（2026-10-08 定稿）：端侧三模型 + 三态开关 + 30s 结构化帧 + IoTDA 上云 + BLE 配网/实时推送
