#include "robot_drive_core.h"

#include <math.h>
#include <stddef.h>

#include "robot_drive_kinematics.h"

void robot_drive_core_zero_output(robot_drive_core_output_t *output)
{
    if (output == NULL) {
        return;
    }
    for (int wheel = 0; wheel < ROBOT_DRIVE_WHEEL_COUNT; ++wheel) {
        output->target_mps[wheel] = 0.0f;
    }
    output->limited = false;
}

robot_drive_core_status_t robot_drive_core_compute_targets(
    const robot_drive_core_config_t *config,
    const robot_drive_core_input_t *input,
    robot_drive_core_output_t *output)
{
    if (output == NULL) {
        return ROBOT_DRIVE_CORE_INVALID_ARGUMENT;
    }
    robot_drive_core_zero_output(output);

    if ((config == NULL) || (input == NULL)) {
        return ROBOT_DRIVE_CORE_INVALID_ARGUMENT;
    }
    if (!isfinite(config->turn_geometry_m) ||
        (config->turn_geometry_m <= 0.0f) ||
        !isfinite(config->max_target_mps) ||
        (config->max_target_mps <= 0.0f)) {
        return ROBOT_DRIVE_CORE_INVALID_CONFIG;
    }

    const robot_drive_core_status_t status = robot_drive_kinematics_compute(
        input, config->turn_geometry_m, output);
    if (status != ROBOT_DRIVE_CORE_OK) {
        robot_drive_core_zero_output(output);
        return status;
    }

    float peak = 0.0f;
    for (int wheel = 0; wheel < ROBOT_DRIVE_WHEEL_COUNT; ++wheel) {
        peak = fmaxf(peak, fabsf(output->target_mps[wheel]));
    }
    if (!isfinite(peak)) {
        robot_drive_core_zero_output(output);
        return ROBOT_DRIVE_CORE_NUMERIC_ERROR;
    }

    if (peak > config->max_target_mps) {
        const float scale = config->max_target_mps / peak;
        if (!isfinite(scale) || (scale <= 0.0f)) {
            robot_drive_core_zero_output(output);
            return ROBOT_DRIVE_CORE_NUMERIC_ERROR;
        }
        for (int wheel = 0; wheel < ROBOT_DRIVE_WHEEL_COUNT; ++wheel) {
            output->target_mps[wheel] *= scale;
        }
        output->limited = true;
    }

    return ROBOT_DRIVE_CORE_OK;
}
