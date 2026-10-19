#include "board.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "esp_adc/adc_oneshot.h"

static constexpr const char* TAG = "SleepPillowBoard";

// ------------------------------------------------------------
// SleepPillow v1.0 底板（ESP32-S3 N16R8）
//
// 本文件是这块底板全部硬件事实的唯一权威定义处：
//   · 引脚映射 / 总线时序 / ADC 配置全部封装在下面的实现类里，
//     上层业务组件（mic/bcg）只经 Board 统一接口使用。
//   · 新增其他底板时，在 boards/ 下新增 board_xxx.cc 并注册即可，
//     上层组件零改动。
//
// 注：原 C1001 毫米波雷达**已移除**（2026-10-14）：
//   1) 其 UART 占用 GPIO4/5，与双 INMP441 麦克风的 I2S 引脚
//      (WS=GPIO4 / BCLK=GPIO5) 硬件冲突；
//   2) 雷达串口协议一直未解出（radar_sensor 只发 0x55 试探），
//      "在床" 始终没有可用数据源。
//   GPIO4/5 现让给 I2S 麦克风，「在床」字段保持 -1（未接入）。
// ------------------------------------------------------------

// ============================================================
// 双 INMP441 I2S 采集（底层封装）
// ============================================================
// 硬件（两个 INMP441 并联在同一组 I2S 总线上，只有 L/R 引脚不同）：
//   麦1（左声道）: VDD/GND/SCK/WS/SD 与总线并联, L/R -> GND
//   麦2（右声道）: VDD/GND/SCK/WS/SD 与总线并联, L/R -> 3V3
//   总线: SCK(BCLK)=GPIO5, WS(LRCLK)=GPIO4, SD(DIN)=GPIO6
//
// 时序要点（踩坑记录，勿动）：
//   · 32bit 槽宽：INMP441 每个 WS 半周期发 1bit 延迟 + 24bit 数据 = 25bit，
//     16bit 槽装不下，会造成声道混叠 + 位错位（表现为采样值乱跳、长期满幅）。
//   · left_align=false：ESP32-S3 的 Philips 宏默认 left_align=true
//     （MSB 紧贴 WS 边沿），而 INMP441 的 MSB 在 WS 边沿后延迟 1 BCLK。
//     差这 1 bit 就会整体错位。
//   · INMP441 仅支持 3.3V 供电。
class SleepPillowMicInput : public BoardMicInput {
public:
    esp_err_t Open(int sample_rate) override {
        if (rx_ != nullptr) return ESP_ERR_INVALID_STATE;

        // ---- 建 I2S RX 通道（master，仅接收）----
        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
        chan_cfg.dma_desc_num  = 6;      // DMA 描述符/缓冲块数
        chan_cfg.dma_frame_num = 256;    // 每块立体声帧数

        esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &rx_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_new_channel 失败: %s", esp_err_to_name(err));
            rx_ = nullptr;
            return err;
        }

        // 沿用 ESP-IDF 官方示例的聚合初始化写法（字段顺序与声明一致）
        i2s_std_config_t std_cfg = {
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sample_rate),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,                      // INMP441 不需要 MCLK
                .bclk = (gpio_num_t)kBclkPin,
                .ws   = (gpio_num_t)kWsPin,
                .dout = I2S_GPIO_UNUSED,                      // 只收不发
                .din  = (gpio_num_t)kDinPin,
                .invert_flags = {
                    .mclk_inv = 0,
                    .bclk_inv = 0,
                    .ws_inv   = 0,
                },
            },
        };
        // ★ 覆盖 slot 对齐方式：匹配 INMP441 的 1 BCLK 延迟（见类注释）
        std_cfg.slot_cfg.left_align = false;
        ESP_LOGW(TAG, "I2S slot: 32bit 槽, bit_shift=%d left_align=%d ws_width=%d data_bit_width=%d",
                 (int)std_cfg.slot_cfg.bit_shift, (int)std_cfg.slot_cfg.left_align,
                 (int)std_cfg.slot_cfg.ws_width, (int)std_cfg.slot_cfg.data_bit_width);

        err = i2s_channel_init_std_mode(rx_, &std_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_init_std_mode 失败: %s", esp_err_to_name(err));
            i2s_del_channel(rx_);
            rx_ = nullptr;
            return err;
        }

        err = i2s_channel_enable(rx_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_enable 失败: %s", esp_err_to_name(err));
            i2s_del_channel(rx_);
            rx_ = nullptr;
            return err;
        }

        sample_rate_ = sample_rate;
        ESP_LOGI(TAG, "I2S 麦克风采集打开: %dHz/32bit槽/立体声, BCLK=%d WS=%d DIN=%d",
                 sample_rate, kBclkPin, kWsPin, kDinPin);
        return ESP_OK;
    }

    void Close() override {
        if (rx_ != nullptr) {
            i2s_channel_disable(rx_);
            i2s_del_channel(rx_);
            rx_ = nullptr;
        }
    }

    esp_err_t ReadStereo(int32_t* dst, size_t frames,
                         size_t* bytes_read, uint32_t timeout_ms) override {
        if (rx_ == nullptr || dst == nullptr || bytes_read == nullptr) {
            return ESP_ERR_INVALID_STATE;
        }
        const size_t want_bytes = frames * 2 * sizeof(int32_t);
        *bytes_read = 0;
        return i2s_channel_read(rx_, dst, want_bytes, bytes_read, timeout_ms);
    }

private:
    // 引脚映射（本板唯一权威定义处）
    static constexpr int kBclkPin = 5;   // INMP441 SCK
    static constexpr int kWsPin   = 4;   // INMP441 WS / LRCLK
    static constexpr int kDinPin  = 6;   // INMP441 SD（两麦并联）

    i2s_chan_handle_t rx_ = nullptr;
    int sample_rate_ = 0;
};

// ============================================================
// 压电陶瓷 BCG ADC 输入（底层封装）
// ============================================================
// 硬件：压电陶瓷 → GPIO1 = ADC1_CHANNEL_0（ESP32-S3 硬件确认）。
// 偏置电路把信号中点抬到 VCC/2 = 1.65V（≈ 12bit raw 2048），
// 采集契约：ReadRaw 返回 12bit 原始计数 0~4095，中点 2048 = 偏置电压。
class SleepPillowAdcInput : public BoardAdcInput {
public:
    esp_err_t Open() override {
        if (opened_) return ESP_ERR_INVALID_STATE;

        // 1) 单元初始化（ADC1）
        //    显式写全所有字段：只写 unit_id 时编译器会报
        //    -Wmissing-field-initializers（clk_src / ulp_mode 未初始化）
        adc_oneshot_unit_init_cfg_t init_cfg = {
            .unit_id  = ADC_UNIT_1,
            .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        esp_err_t err = adc_oneshot_new_unit(&init_cfg, &handle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
            return err;
        }

        // 2) 通道配置
        //    attenuation=12dB（原 11dB 已废弃，两者行为一致）→ 量程≈0~3.1V，
        //    覆盖压电陶瓷经 1.65V 偏置后的输出
        //    bit_width=12 → raw ∈ [0, 4095]，中点 2048 对应 1.65V
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        err = adc_oneshot_config_channel(handle_, kChannel, &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
            adc_oneshot_del_unit(handle_);
            handle_ = nullptr;
            return err;
        }

        // 校准（可选，提升精度；校准失败不影响采集，只是精度略低）
        // ESP32-S3 提供 eFuse 校准值，需要时可接入 adc_cali 接口。
        opened_ = true;
        ESP_LOGI(TAG, "ADC 输入打开: ADC1_CH%d @ GPIO%02d（atten=12dB, 12bit）",
                 (int)kChannel, kGpioPin);
        return ESP_OK;
    }

    void Close() override {
        if (handle_ != nullptr) {
            adc_oneshot_del_unit(handle_);
            handle_ = nullptr;
        }
        opened_ = false;
    }

    esp_err_t ReadRaw(int* raw) override {
        if (!opened_ || handle_ == nullptr) return ESP_ERR_INVALID_STATE;
        return adc_oneshot_read(handle_, kChannel, raw);
    }

private:
    // 引脚映射（本板唯一权威定义处）
    static constexpr int kGpioPin = 1;                    // GPIO1
    static constexpr adc_channel_t kChannel = ADC_CHANNEL_0;  // GPIO1 → ADC1_CH0

    adc_oneshot_unit_handle_t handle_ = nullptr;
    bool opened_ = false;
};

// ============================================================
// 板型定义
// ============================================================
class SleepPillowBoard : public Board {
public:
    SleepPillowBoard() {
        Board::SetInstance(this);
    }

    bool Init() override {
        ESP_LOGI(TAG, "Initializing SleepPillow board...");
        // 底层外设（I2S 麦克风 / ADC）不在此初始化：上层组件按需经
        // GetMicInput()/GetBcgAdc() 索取，由组件在 REAL 模式下 Open。
        return true;
    }

    const char* GetBoardName() override {
        return "SleepPillow";
    }

    // ---- 统一硬件接口（本板实现）----
    BoardMicInput* GetMicInput() override { return &mic_input_; }
    BoardAdcInput* GetBcgAdc()   override { return &bcg_adc_; }

private:
    SleepPillowMicInput mic_input_;
    SleepPillowAdcInput bcg_adc_;
};

// 全局单例实例（静态构造时自动注册到Board::GetInstance()）
static SleepPillowBoard g_board;

// 强制链接入口：被 app_main 引用后，本目标文件才会从 libboard.a 中
// 被提取出来，g_board 才会存在；同时显式注册一次，避免依赖静态
// 构造函数的执行时机。
void BoardEnsureRegistered() {
    Board::SetInstance(&g_board);
}
