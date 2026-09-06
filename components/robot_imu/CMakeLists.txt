idf_component_register(
    SRCS
        "robot_imu.c"
        "robot_imu_policy.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer freertos
)
