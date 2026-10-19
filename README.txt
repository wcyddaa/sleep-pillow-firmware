===============================================================
  Sleep Pillow Base v1 - 项目说明
  ESP32 睡眠枕头基础架构
  构建日期: 2026-09-02
  最近更新: 2026-10-14 (移除 C1001 雷达 / 接入双 INMP441 麦克风)
===============================================================

一、项目概述
---------------------------------------------------------------
基于 ESP-IDF v5.5 的睡眠监测枕头固件，采用 C++ 面向对象设计。
核心功能：端侧三个模型（鼾声 / 睡眠分期 / BCG 体动）推理，
30 秒一帧结构化数据经 MQTT 上行华为云 IoTDA。

音频采集：双 INMP441 数字麦克风（I2S0 立体声 16kHz/16bit），
采集后近场选麦合成单声道，喂给鼾声模型。
注：原 C1001 毫米波雷达已移除，原因见 4.1。

目标芯片：ESP32-S3 (N16R8: 16MB Flash + 8MB PSRAM)
IDF 版本：v5.5.4


二、目录结构
---------------------------------------------------------------
sleep-pillow/
├── CMakeLists.txt                    # 根 CMake
├── sdkconfig.defaults                # 默认配置
├── partitions.csv                    # 分区表 (3分区: nvs/phy/factory)
├── README.txt                        # 本文件
│
├── main/                             # 应用主程序
│   ├── CMakeLists.txt
│   ├── main.cpp                      # 入口函数 (app_main)
│   ├── include/
│   │   ├── app_main.h                # 应用主逻辑声明
│   │   └── app_config.h             # 配置宏 (SSID/密码/采样间隔)
│   └── app_main.cpp                  # 应用主逻辑实现
│
└── components/                       # 本地组件
    ├── board/                        # 板级抽象层
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   └── board.h               # Board 抽象接口 + 单例
    │   ├── board_registry.cc         # 板型注册表 (工厂模式)
    │   ├── wifi_board.cc             # Wi-Fi 板型基类
    │   └── boards/
    │       └── board_sleep_pillow_v1.cc  # 睡眠枕头板型实现
    │
    ├── mic/                          # 麦克风采集层
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   └── inmp441_mic.h         # 双 INMP441 采集 + 近场选麦声明
    │   └── inmp441_mic.cc            # I2S 立体声采集 → 单声道实现
    │
    ├── network/                      # 网络层
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   └── wifi.h                # WifiManager 接口
    │   └── wifi_manager.cc           # Wi-Fi 阻塞连接管理
    │
    ├── audio/                        # 音频层 (桩文件，待实现)
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   └── audio_codec.h         # 音频编解码抽象接口
    │   ├── codecs/
    │   │   ├── es8311_codec.cc       # ES8311 桩
    │   │   └── es7210_codec.cc       # ES7210 桩
    │   └── wake_word/
    │       └── esp_sr_wake.cc        # 语音唤醒桩
    │
    └── utils/                        # 工具层
        ├── CMakeLists.txt
        ├── include/
        │   ├── logger.h              # 日志工具
        │   └── timer.h               # 定时器工具
        ├── logger.cc
        └── timer.cc


三、架构设计
---------------------------------------------------------------

3.1 分层关系

    ┌─────────────────────────────────┐
    │          main (应用层)           │
    │   app_main() → AppMain::Run()   │
    └──────────┬──────────────────────┘
               │ 依赖
    ┌──────────▼──────────────────────┐
    │       board (板级抽象层)         │
    │  Board 单例 → 具体板型实现       │
    └────┬─────────┬──────────────────┘
         │         │ 依赖
    ┌────▼────┐ ┌──▼──────────────────┐
    │  mic    │ │ network (可选)       │
    │ I2S 采集│ │ Wi-Fi 阻塞连接       │
    └─────────┘ └─────────────────────┘
         │              │
    ┌────▼──────────────▼──────────────┐
    │          utils (工具层)          │
    │  logger + timer                  │
    └──────────────────────────────────┘

3.2 核心设计模式

  - 单例模式：Board::GetInstance() 全局访问板级实例
    全局静态对象 g_board 在构造时自动注册到单例

  - 多态/工厂模式：
    Board 抽象类 → SleepPillowBoard 具体实现
    BoardRegistry 支持运行时注册/选择板型

  - 阻塞式连接：WifiManager::Connect() 带超时轮询，
    无事件回调，无自动重连


四、硬件配置
---------------------------------------------------------------

4.1 双 INMP441 麦克风（I2S0 立体声采集）
    通信接口：I2S0（master RX）
    引脚：    SCK/BCLK  = GPIO5
              WS/LRCLK  = GPIO4
              SD/DIN    = GPIO6
    参数：    16kHz / 16bit / 立体声（L=左麦、R=右麦）
    接线：    两麦 VDD/GND/SCK/WS/SD 全部并联，
              只有 L/R 不同：麦1 L/R→GND（左声道），麦2 L/R→3V3（右声道）
    供电：    INMP441 只能 3.3V，严禁 5V
    输出：    近场选麦——两路能量接近时取平均（降噪），
              一路明显更响时取该路（判为更近的声源）；
              合成单声道 → snore_infer（16000 点/帧）

4.1.1 已移除：C1001 毫米波雷达
    原占 UART1 TX=GPIO5 / RX=GPIO4，与上述 I2S 引脚硬件冲突；
    且串口协议一直未解出（驱动仅发 0x55 试探）。
    整体移除后，`in_bed`（在床）字段恒为 -1（未接入）。

4.2 预留未接线：MAX98357 功放（I2S1 播放）
    BCLK=GPIO7, LRC=GPIO8, DIN=GPIO9 —— 当前固件未实现播放路径。

4.2 Flash 分区
    nvs       0x9000   24KB   非易失存储
    phy_init  0xf000    4KB   RF校准
    factory   0x10000   3MB   工厂固件

4.3 内存占用 (v1)
    Flash 镜像：752 KB / 3 MB (25%)
    IRAM：       96 KB / 128 KB (75%)
    DRAM：       34 KB / 177 KB (19%)


五、组件依赖关系
---------------------------------------------------------------

  main     REQUIRES → board, mic, network, utils, cloud, app_config,
                      snore_infer, stage_infer, motion_infer, models,
                      edge_aggregator, bcg
  board    REQUIRES → driver, esp_wifi, esp_event
  mic      REQUIRES → driver, esp_timer
  network  REQUIRES → esp_wifi, esp_event, esp_netif, nvs_flash
  audio    REQUIRES → driver, esp_timer     (模板遗留，未参与构建)
  utils    REQUIRES → esp_timer

  ESP-IDF 公共依赖 (自动注入)：freertos, log, heap, soc, hal,
  esp_rom, esp_common, esp_system, cxx, newlib


六、启动流程
---------------------------------------------------------------

  1. app_main() 入口
  2. NVS 初始化 + AppConfigStore 载入三开关（默认全 OFF）
  3. Board::GetInstance()->Init()（本板无板级外设）
  4. 启动 EdgeAggregator（30s 窗口；三开关全 OFF 时内部直接 return）
  5. 启动 BLE 配网 + 每秒实时推送（0x04/0x05 读写三开关）
  6. 开机自动重连已保存 Wi-Fi → 上云（MQTT，NTP 校时后 30s 帧才上行）
  7. 配网成功 → ApplyEdgeCfg 按 cfg 启停三个端侧任务：
       鼾声任务  REAL: INMP441 I2S 采集 → 近场单声道 → snore_infer
                 SIM : 合成 1s PCM → snore_infer
       分期任务  4Hz 特征累积 → stage_infer（每 30s 推理一次）
       体动任务  140Hz 采样 → motion_infer + BCG 心率/呼吸派生
  8. 主循环 idle（实际工作由上面各任务与 FreeRTOS 调度承担）


七、v1 已知限制
---------------------------------------------------------------

  [1] 蓝牙板型 (BluetoothBoard) 已排除编译
      → 后续启用需在 sdkconfig 加 CONFIG_BT_ENABLED=y
      → board/CMakeLists.txt 加回 bluetooth_board.cc 和 bt 依赖

  [2] 音频组件为桩文件 (stub)
      → es8311_codec / es7210_codec / esp_sr_wake 仅占位
      → 需填充真实 I2S 驱动和语音唤醒逻辑

  [3] 「在床」字段无数据源
      → C1001 雷达已移除（GPIO4/5 冲突 + 协议未解出）
      → LiveSnapshot / 30s 帧里 in_bed 恒为 -1

  [4] Wi-Fi 无自动重连
      → 连接失败后仅警告，继续离线运行
      → 无断线重连机制

  [5] 无 OTA 支持
      → 分区表仅 factory 分区，无 OTA 分区


八、构建命令
---------------------------------------------------------------

  # 设置目标芯片 (首次)
  idf.py set-target esp32

  # 构建
  idf.py build

  # 烧录 + 串口监控
  idf.py flash monitor

  # 清理重建
  idf.py fullclean && idf.py build


九、版本历史
---------------------------------------------------------------

  v1.0+ 2026-10-08  ★ 端侧三模型 + 三开关 + 30s 结构化数据（架构定稿）
                    在 v1.0 基础上把"模拟思路"改成"假数据进真实模型推理"。
                    新增 components:
                      app_config      NVS 三开关（snore/stage/bcg 各 OFF/SIM/REAL）
                      snore_infer     64 维手工特征 + int8 MLP 鼾声检测
                      stage_infer     EdgeBiGatedNet int8 睡眠分期
                      motion_infer     多尺度 std + BiGRU-FCN 体动
                      bcg             压电陶瓷占位 + 派生心率/呼吸
                      edge_aggregator  30s 结构化数据聚合 → MQTT /device/{id}/frame
                      models          int8 权重（从 v2.0 导出直接搬）

                    ★ 三开关是【设备本地状态】，不经云端：
                      ble_provision   0x04 GET_CFG → 回 {"evt":"cfg","snore":..,"stage":..,"bcg":..}
                                      0x05 SET_CFG + JSON → 写 NVS + 回 {"evt":"cfg_ok"}
                      手机连上蓝牙后自动读，改开关也只走 BLE。
                      当前 cfg 作为字段随每一帧上行，后端不单独存。

                    ★ 设备只上行两个自定义主题（IoTDA 规则引擎转发到后端 webhook）：
                      /device/{id}/online   上线报到（MQTT 每次连上发一次）
                                            —— 三开关全 OFF 时设备不发帧，没有这条
                                               首次配网的设备永远不进 devices 表，小程序
                                               bind 会失败，所以它是必需的
                      /device/{id}/frame    30s 一帧结构化数据
                    帧 JSON 形如：
                      {"device_id":"..","evt":"frame","start":..,"end":..,"stage":..,
                       "hr":..,"rr":..,"motion":..,"snore_sec":..,"flags":..,
                       "cfg":{"snore":..,"stage":..,"bcg":..}}
                      未启用通道写 -1；flags bit0=snore_off bit1=stage_off bit2=bcg_off

                    ★ BLE 实时推送：每秒推一次当前窗口实时值
                      TX: {"evt":"rt","hr":65,"rr":15,"motion":0,"snore":3,"stage":1}
                      手机未连接时 Notify 静默忽略；-1 = 该通道开关为 OFF

                    删除（旧架构遗留）:
                      cloud_client    PublishSleepData / SleepData / VitalSign /
                                      AudioSegment / topic_($oc/.../messages/up) /
                                      PublishCfg / topic_cfg_ / cJSON 依赖
                      app_main        删 SimulateXxx / FlushWindow / s_sampling；
                                      PublishCfg 调用
                      app_config.h    删 WIFI_SSID/WIFI_PASSWORD/SENSOR_READ_MS/
                                      CLOUD_REPORT_DEVICE_ID/CLOUD_PUBLISH_MS

                    行为：
                      全 OFF → 三个 task 不创建，聚合器不启动，不上云任何帧
                              （仍发一次上线报到，保证云端能注册这台设备）
                      任一非 OFF（SIM/REAL）→ 30s frame 上云
                      SIM  → 假数据 → 真实模型推理 → 30s frame 上云
                      REAL → 真实传感器（压电陶瓷 + 麦克风）→ 真实模型 → 上云

  v1  2026-09-02  基础架构搭建完成，编译通过
                   修复: wifi→esp_wifi, bt排除, 类重定义,
                         esp_timer依赖缺失
