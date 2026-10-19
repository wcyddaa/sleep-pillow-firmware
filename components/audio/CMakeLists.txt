# ============================================================
# audio 组件构建文件
# ============================================================

idf_component_register(
    SRCS
        "codecs/es8311_codec.cc"
        "codecs/es7210_codec.cc"
        "wake_word/esp_sr_wake.cc"
    INCLUDE_DIRS
        "include"
    REQUIRES
        "driver"
        "esp_timer"
)
