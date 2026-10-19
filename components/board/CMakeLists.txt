idf_component_register(SRCS "board_registry.cc"
                           "wifi_board.cc"
                           "boards/board_sleep_pillow_v1.cc"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_wifi esp_event)