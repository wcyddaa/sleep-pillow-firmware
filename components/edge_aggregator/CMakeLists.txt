idf_component_register(
    SRCS "edge_aggregator.cc"
    INCLUDE_DIRS "include"
    REQUIRES app_config snore_infer stage_infer bcg cloud offline_queue freertos
    PRIV_REQUIRES esp_timer
)
