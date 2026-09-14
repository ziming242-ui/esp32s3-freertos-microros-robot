idf_component_register(
    SRCS
        "robot_imu.c"
        "robot_imu_policy.c"
        "robot_imu_icm42670p_core.c"
        "robot_imu_icm42670p_backend.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_timer freertos
)
