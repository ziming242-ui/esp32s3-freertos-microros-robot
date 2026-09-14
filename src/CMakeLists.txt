set(robot_app_requires esp_driver_gpio)
if(CONFIG_ROBOT_APP_MODE_CHASSIS_DIAG)
    list(APPEND robot_app_requires robot_chassis_diag)
elseif(CONFIG_ROBOT_APP_MODE_ENCODER_PROBE)
    list(APPEND robot_app_requires robot_encoder_probe)
elseif(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    list(APPEND robot_app_requires
        robot_chassis_diag robot_imu robot_microros robot_wifi)
else()
    list(APPEND robot_app_requires
        robot_drive robot_imu robot_microros robot_wifi)
endif()

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES ${robot_app_requires}
)
