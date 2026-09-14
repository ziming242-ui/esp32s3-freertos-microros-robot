#include "robot_chassis_diag_core.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static void zero_control(robot_chassis_core_t *core)
{
    memset(core->target_counts, 0, sizeof(core->target_counts));
    memset(core->output, 0, sizeof(core->output));
    memset(core->integral, 0, sizeof(core->integral));
    memset(core->stall_count, 0, sizeof(core->stall_count));
}

static bool config_valid(const robot_chassis_core_config_t *config)
{
    return (config != NULL) && isfinite(config->kp) && (config->kp >= 0.0f) &&
           isfinite(config->ki_per_step) && (config->ki_per_step >= 0.0f) &&
           isfinite(config->integral_limit) && (config->integral_limit > 0.0f) &&
           (config->max_output > 0) && (config->max_target_counts > 0) &&
           (config->command_timeout_ms > 0U) && (config->stall_steps > 0U);
}

bool robot_chassis_core_init(robot_chassis_core_t *core,
                             const robot_chassis_core_config_t *config)
{
    if ((core == NULL) || !config_valid(config)) {
        return false;
    }
    memset(core, 0, sizeof(*core));
    core->config = *config;
    core->state = ROBOT_CHASSIS_STATE_DISARMED;
    core->stop_reason = ROBOT_CHASSIS_STOP_STARTUP;
    return true;
}

bool robot_chassis_core_update_gains(robot_chassis_core_t *core,
                                     float kp,
                                     float ki_per_step)
{
    if ((core == NULL) || (core->state != ROBOT_CHASSIS_STATE_DISARMED) ||
        !isfinite(kp) || (kp < 0.0f) ||
        !isfinite(ki_per_step) || (ki_per_step < 0.0f)) {
        return false;
    }
    zero_control(core);
    core->config.kp = kp;
    core->config.ki_per_step = ki_per_step;
    core->zero_seen = false;
    core->stop_reason = ROBOT_CHASSIS_STOP_EXPLICIT;
    return true;
}

bool robot_chassis_core_twist_to_targets(
    float linear_x_mps,
    float angular_z_radps,
    float turn_geometry_m,
    float counts_per_metre_per_step,
    int max_target_counts,
    int target_counts[ROBOT_CHASSIS_WHEEL_COUNT],
    bool *limited)
{
    if ((target_counts == NULL) || (limited == NULL) ||
        !isfinite(linear_x_mps) || !isfinite(angular_z_radps) ||
        !isfinite(turn_geometry_m) || (turn_geometry_m <= 0.0f) ||
        !isfinite(counts_per_metre_per_step) ||
        (counts_per_metre_per_step <= 0.0f) ||
        (max_target_counts <= 0)) {
        return false;
    }

    float left_counts =
        (linear_x_mps - angular_z_radps * turn_geometry_m) *
        counts_per_metre_per_step;
    float right_counts =
        (linear_x_mps + angular_z_radps * turn_geometry_m) *
        counts_per_metre_per_step;
    const float peak = fmaxf(fabsf(left_counts), fabsf(right_counts));
    *limited = peak > (float)max_target_counts;
    if (*limited) {
        const float scale = (float)max_target_counts / peak;
        left_counts *= scale;
        right_counts *= scale;
    }

    const int left = (int)lroundf(left_counts);
    const int right = (int)lroundf(right_counts);
    target_counts[0] = left;
    target_counts[1] = left;
    target_counts[2] = right;
    target_counts[3] = right;
    return true;
}

int robot_chassis_safe_count_delta(int current_count,
                                   int previous_count,
                                   int count_limit)
{
    const int raw_delta = current_count - previous_count;
    if (count_limit <= 0) {
        return raw_delta;
    }

    /* PCNT restarts at zero after reaching either configured boundary. The
     * real per-sample movement is therefore the shortest of the direct path
     * and the two paths crossing a boundary. */
    int best_delta = raw_delta;
    const int positive_wrap_delta = raw_delta + count_limit;
    const int negative_wrap_delta = raw_delta - count_limit;
    if (abs(positive_wrap_delta) < abs(best_delta)) {
        best_delta = positive_wrap_delta;
    }
    if (abs(negative_wrap_delta) < abs(best_delta)) {
        best_delta = negative_wrap_delta;
    }
    return best_delta;
}

bool robot_chassis_odom_update(
    robot_chassis_odom_t *odom,
    const int count_delta[ROBOT_CHASSIS_WHEEL_COUNT],
    float dt_s,
    float metres_per_count,
    float turn_geometry_m)
{
    if ((odom == NULL) || (count_delta == NULL) ||
        !isfinite(dt_s) || (dt_s <= 0.0f) ||
        !isfinite(metres_per_count) || (metres_per_count <= 0.0f) ||
        !isfinite(turn_geometry_m) || (turn_geometry_m <= 0.0f) ||
        !isfinite(odom->x_m) || !isfinite(odom->y_m) ||
        !isfinite(odom->yaw_rad)) {
        return false;
    }

    const float left_counts =
        0.5f * ((float)count_delta[0] + (float)count_delta[1]);
    const float right_counts =
        0.5f * ((float)count_delta[2] + (float)count_delta[3]);
    const float left_m = left_counts * metres_per_count;
    const float right_m = right_counts * metres_per_count;
    const float distance_m = 0.5f * (left_m + right_m);
    const float delta_yaw_rad =
        (right_m - left_m) / (2.0f * turn_geometry_m);
    const float midpoint_yaw_rad = odom->yaw_rad + 0.5f * delta_yaw_rad;

    odom->x_m += distance_m * cosf(midpoint_yaw_rad);
    odom->y_m += distance_m * sinf(midpoint_yaw_rad);
    odom->yaw_rad += delta_yaw_rad;
    odom->linear_x_mps = distance_m / dt_s;
    odom->angular_z_radps = delta_yaw_rad / dt_s;
    return true;
}

void robot_chassis_core_accept_zero(robot_chassis_core_t *core,
                                    uint32_t now_ms)
{
    if (core == NULL) {
        return;
    }
    zero_control(core);
    core->last_command_ms = now_ms;
    core->zero_seen = true;
    if (core->state != ROBOT_CHASSIS_STATE_FAULT_LATCHED) {
        core->state = ROBOT_CHASSIS_STATE_DISARMED;
        core->stop_reason = ROBOT_CHASSIS_STOP_EXPLICIT;
    }
}

bool robot_chassis_core_arm(robot_chassis_core_t *core, uint32_t now_ms)
{
    if ((core == NULL) || (core->state != ROBOT_CHASSIS_STATE_DISARMED) ||
        !core->zero_seen) {
        return false;
    }
    zero_control(core);
    core->state = ROBOT_CHASSIS_STATE_ARMED;
    core->stop_reason = ROBOT_CHASSIS_STOP_NONE;
    core->last_command_ms = now_ms;
    return true;
}

bool robot_chassis_core_submit(robot_chassis_core_t *core,
                               const int target_counts[ROBOT_CHASSIS_WHEEL_COUNT],
                               uint32_t now_ms)
{
    if ((core == NULL) || (target_counts == NULL) ||
        (core->state != ROBOT_CHASSIS_STATE_ARMED)) {
        return false;
    }

    bool any_nonzero = false;
    for (int i = 0; i < ROBOT_CHASSIS_WHEEL_COUNT; ++i) {
        if (abs(target_counts[i]) > core->config.max_target_counts) {
            robot_chassis_core_stop(core, ROBOT_CHASSIS_STOP_INVALID_TARGET);
            return false;
        }
        any_nonzero = any_nonzero || (target_counts[i] != 0);
    }

    memcpy(core->target_counts, target_counts, sizeof(core->target_counts));
    core->last_command_ms = now_ms;
    core->stop_reason = any_nonzero ? ROBOT_CHASSIS_STOP_NONE
                                    : ROBOT_CHASSIS_STOP_EXPLICIT;
    if (!any_nonzero) {
        memset(core->output, 0, sizeof(core->output));
        memset(core->integral, 0, sizeof(core->integral));
        memset(core->stall_count, 0, sizeof(core->stall_count));
    }
    return true;
}

void robot_chassis_core_stop(robot_chassis_core_t *core,
                             robot_chassis_stop_reason_t reason)
{
    if (core == NULL) {
        return;
    }
    zero_control(core);
    core->stop_reason = reason;
    if ((reason == ROBOT_CHASSIS_STOP_INVALID_TARGET) ||
        (reason == ROBOT_CHASSIS_STOP_ENCODER_STALL)) {
        core->state = ROBOT_CHASSIS_STATE_FAULT_LATCHED;
        core->zero_seen = false;
    } else {
        core->state = ROBOT_CHASSIS_STATE_DISARMED;
        if (reason == ROBOT_CHASSIS_STOP_TIMEOUT) {
            core->zero_seen = false;
        }
    }
}

bool robot_chassis_core_clear_fault(robot_chassis_core_t *core)
{
    if ((core == NULL) ||
        (core->state != ROBOT_CHASSIS_STATE_FAULT_LATCHED) ||
        !core->zero_seen) {
        return false;
    }
    zero_control(core);
    core->state = ROBOT_CHASSIS_STATE_DISARMED;
    core->stop_reason = ROBOT_CHASSIS_STOP_EXPLICIT;
    return true;
}

void robot_chassis_core_step(robot_chassis_core_t *core,
                             const int count_delta[ROBOT_CHASSIS_WHEEL_COUNT],
                             uint32_t now_ms)
{
    if ((core == NULL) || (count_delta == NULL)) {
        return;
    }
    if (core->state != ROBOT_CHASSIS_STATE_ARMED) {
        memset(core->output, 0, sizeof(core->output));
        return;
    }

    const uint32_t age_ms = now_ms - core->last_command_ms;
    if (age_ms > core->config.command_timeout_ms) {
        robot_chassis_core_stop(core, ROBOT_CHASSIS_STOP_TIMEOUT);
        return;
    }

    for (int i = 0; i < ROBOT_CHASSIS_WHEEL_COUNT; ++i) {
        const int target = core->target_counts[i];
        if (target == 0) {
            core->output[i] = 0;
            core->integral[i] = 0.0f;
            core->stall_count[i] = 0;
            continue;
        }

        if (count_delta[i] == 0) {
            if (core->stall_count[i] < UINT16_MAX) {
                ++core->stall_count[i];
            }
        } else {
            core->stall_count[i] = 0;
        }
        if (core->stall_count[i] >= core->config.stall_steps) {
            robot_chassis_core_stop(core, ROBOT_CHASSIS_STOP_ENCODER_STALL);
            return;
        }

        const float error = (float)(target - count_delta[i]);
        core->integral[i] += error;
        if (core->integral[i] > core->config.integral_limit) {
            core->integral[i] = core->config.integral_limit;
        } else if (core->integral[i] < -core->config.integral_limit) {
            core->integral[i] = -core->config.integral_limit;
        }
        float command = core->config.kp * error +
                        core->config.ki_per_step * core->integral[i];
        if (command > (float)core->config.max_output) {
            command = (float)core->config.max_output;
        } else if (command < (float)-core->config.max_output) {
            command = (float)-core->config.max_output;
        }
        core->output[i] = (int)lroundf(command);
    }
}

const char *robot_chassis_state_string(robot_chassis_state_t state)
{
    switch (state) {
    case ROBOT_CHASSIS_STATE_DISARMED: return "DISARMED";
    case ROBOT_CHASSIS_STATE_ARMED: return "ARMED";
    case ROBOT_CHASSIS_STATE_FAULT_LATCHED: return "FAULT_LATCHED";
    default: return "UNKNOWN";
    }
}

const char *robot_chassis_stop_reason_string(robot_chassis_stop_reason_t reason)
{
    switch (reason) {
    case ROBOT_CHASSIS_STOP_STARTUP: return "startup";
    case ROBOT_CHASSIS_STOP_NONE: return "none";
    case ROBOT_CHASSIS_STOP_EXPLICIT: return "explicit";
    case ROBOT_CHASSIS_STOP_TIMEOUT: return "timeout";
    case ROBOT_CHASSIS_STOP_INVALID_TARGET: return "invalid-target";
    case ROBOT_CHASSIS_STOP_ENCODER_STALL: return "encoder-stall";
    default: return "unknown";
    }
}
