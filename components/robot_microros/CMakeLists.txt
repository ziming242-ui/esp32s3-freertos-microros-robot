idf_component_register(
    SRCS "robot_microros.c"
    INCLUDE_DIRS "include"
    REQUIRES micro_ros_espidf_component esp_timer robot_drive robot_imu
)
