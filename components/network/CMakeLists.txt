idf_component_register(SRCS "wifi_manager.cc"
                           "ble_provision.cc"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_event esp_netif nvs_flash bt mbedtls json
                                app_config)
