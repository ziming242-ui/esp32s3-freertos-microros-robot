#include "robot_drive_kinematics.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

robot_drive_core_status_t robot_drive_kinematics_compute(
    const robot_drive_core_input_t *input,
    float turn_geometry_m,
    robot_drive_core_output_t *output)
{
    if ((input == NULL) || (output == NULL)) {
        return ROBOT_DRIVE_CORE_INVALID_ARGUMENT;
    }
    if (!isfinite(input->linear_x_mps) ||
        !isfinite(input->angular_z_radps)) {
        return ROBOT_DRIVE_CORE_INVALID_COMMAND;
    }
    if (!isfinite(turn_geometry_m) || (turn_geometry_m <= 0.0f)) {
        return ROBOT_DRIVE_CORE_INVALID_CONFIG;
    }

    const double left = (double)input->linear_x_mps -
                        (double)input->angular_z_radps *
                        (double)turn_geometry_m;
    const double right = (double)input->linear_x_mps +
                         (double)input->angular_z_radps *
                         (double)turn_geometry_m;
    if (!isfinite(left) || !isfinite(right) ||
        (fabs(left) > FLT_MAX) || (fabs(right) > FLT_MAX)) {
        return ROBOT_DRIVE_CORE_NUMERIC_ERROR;
    }

    output->target_mps[ROBOT_DRIVE_WHEEL_LF] = (float)left;
    output->target_mps[ROBOT_DRIVE_WHEEL_LR] = (float)left;
    output->target_mps[ROBOT_DRIVE_WHEEL_RF] = (float)right;
    output->target_mps[ROBOT_DRIVE_WHEEL_RR] = (float)right;
    return ROBOT_DRIVE_CORE_OK;
}
