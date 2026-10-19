idf_component_register(SRCS "board_registry.cc"
                           "wifi_board.cc"
                           "boards/board_esp32s3.cc"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_wifi esp_event)