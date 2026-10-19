idf_component_register(SRCS "inmp441_mic.cc"
                       INCLUDE_DIRS "include"
                       # 底层 I2S 操作与引脚封装在板级抽象层，本组件只面向
                       # BoardMicInput 接口 → 只依赖 board（board.h 仅在 .cc 中引用，
                       # 放 PRIV_REQUIRES）。log/heap 为 IDF 公共依赖，无需声明。
                       PRIV_REQUIRES board)
