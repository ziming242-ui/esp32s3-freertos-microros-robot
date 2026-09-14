idf_component_register(
    SRCS "robot_encoder_probe.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
    PRIV_REQUIRES esp_driver_pcnt freertos log
)
