idf_component_register(
    SRCS
        "robot_chassis_diag.c"
        "robot_chassis_diag_core.c"
    INCLUDE_DIRS "include"
    REQUIRES bdc_motor esp_adc esp_driver_pcnt esp_driver_uart esp_timer freertos robot_imu
)
