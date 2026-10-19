idf_component_register(
    SRCS "bcg_adc.cc"
    INCLUDE_DIRS "include"
    # ★ esp_adc 必须是公共依赖(REQUIRES)：bcg_adc.h 里声明了
    #   adc_oneshot_unit_handle_t 等成员变量，属于公开接口的一部分。
    #   放在 PRIV_REQUIRES 里时，main 等使用方 include bcg_adc.h 会找不到
    #   esp_adc/adc_oneshot.h → 编译失败。
    REQUIRES app_config motion_infer esp_adc
    PRIV_REQUIRES driver
)
