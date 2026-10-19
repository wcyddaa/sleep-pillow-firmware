idf_component_register(SRCS "main.cpp" "app_main.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES board mic network utils cloud
                                app_config snore_infer stage_infer motion_infer
                                models edge_aggregator bcg)
