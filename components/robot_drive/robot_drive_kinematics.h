#pragma once

#include "robot_drive_core.h"

robot_drive_core_status_t robot_drive_kinematics_compute(
    const robot_drive_core_input_t *input,
    float turn_geometry_m,
    robot_drive_core_output_t *output);
