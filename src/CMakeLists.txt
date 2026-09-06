idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES robot_wifi robot_microros robot_drive robot_imu
)
