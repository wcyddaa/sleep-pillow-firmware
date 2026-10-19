idf_component_register(SRCS "cloud_client.cc"
                       INCLUDE_DIRS "include"
                       REQUIRES mqtt mbedtls esp_event esp_netif)
