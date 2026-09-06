idf_component_register(
    SRCS
        "robot_drive.c"
        "robot_drive_core.c"
        "robot_drive_kinematics.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_common
    PRIV_REQUIRES esp_timer freertos
)
