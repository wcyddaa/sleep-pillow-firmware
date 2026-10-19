# motion_infer 组件 — BCG 体动检测（多尺度 std 阈值 + 可选 BiGRU-FCN int8）
idf_component_register(
    SRCS
        "motion_infer.cc"
    INCLUDE_DIRS
        "include"
    REQUIRES
        models
)
