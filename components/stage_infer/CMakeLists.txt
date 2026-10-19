# stage_infer 组件 — 睡眠分期（EdgeBiGatedNet int8）
idf_component_register(
    SRCS
        "stage_infer.cc"
    INCLUDE_DIRS
        "include"
    REQUIRES
        models
)
