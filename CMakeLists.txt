idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES driver nvs_flash esp_timer
                       INCLUDE_DIRS ".")