# ============================================================
# snore_infer 组件 — 鼾声检测推理（64 维手工前端 + int8 MLP + 事件聚合）
# 模型权重与推理算子来自 components/models
# ============================================================
idf_component_register(
    SRCS
        "snore_infer.cc"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_timer
        models
)
