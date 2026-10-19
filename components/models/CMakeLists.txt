# models 组件：三个端侧模型的 int8 权重 + 端侧推理算子与网络前向
# 生成方式：python components/models/tools/export_onnx_to_c.py
idf_component_register(
    SRCS
        "src/nn_ops.cc"
        "src/model_nets.cc"
        "models/model_snore.cc"
        "models/model_snore_fe.cc"
        "models/model_stage.cc"
        "models/model_motion.cc"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_timer
)
