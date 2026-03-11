idf_component_register(SRCS "esp_ds18b20.c"
                    INCLUDE_DIRS "include"
                    REQUIRES log
                    PRIV_REQUIRES esp32-ds18b20 esp32-owb)

