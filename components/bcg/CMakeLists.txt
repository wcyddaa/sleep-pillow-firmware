idf_component_register(
    SRCS "bcg_adc.cc"
    INCLUDE_DIRS "include"
    # app_config/motion_infer 是公共依赖(REQUIRES)：bcg_adc.h 里
    # 使用了 EdgeMode 等类型，属于公开接口的一部分。
    # 底层 ADC 操作与引脚封装在板级抽象层，本组件只面向 BoardAdcInput
    # 接口 → board.h 仅在 .cc 中引用，放 PRIV_REQUIRES。
    REQUIRES app_config motion_infer
    PRIV_REQUIRES board
)
