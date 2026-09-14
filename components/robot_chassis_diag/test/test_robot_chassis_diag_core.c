#include "robot_chassis_diag_core.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int checks;
static int failures;

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        ++failures; \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
    } \
} while (0)

static robot_chassis_core_t make_core(uint16_t stall_steps)
{
    robot_chassis_core_t core;
    const robot_chassis_core_config_t config = {
        .kp = 5.0f,
        .ki_per_step = 0.25f,
        .integral_limit = 40.0f,
        .max_output = 20,
        .max_target_counts = 3,
        .command_timeout_ms = 500,
        .stall_steps = stall_steps,
    };
    CHECK(robot_chassis_core_init(&core, &config));
    return core;
}

int main(void)
{
    const int pcnt_limit = 30000;
    const float metres_per_count = 0.1508f / 1040.0f;
    const float turn_geometry_m = 0.115f;
    /* Positive crossing: 29999 -> 0 is one forward count, not -29999. */
    CHECK(robot_chassis_safe_count_delta(0, 29999, pcnt_limit) == 1);
    /* Negative crossing: -29999 -> 0 is one reverse count, not +29999. */
    CHECK(robot_chassis_safe_count_delta(0, -29999, pcnt_limit) == -1);
    /* Ordinary samples remain a plain signed difference. */
    CHECK(robot_chassis_safe_count_delta(110, 100, pcnt_limit) == 10);
    CHECK(robot_chassis_safe_count_delta(10000, 20000, pcnt_limit) == -10000);

    robot_chassis_odom_t odom = {0};
    const int stationary_counts[4] = {0, 0, 0, 0};
    CHECK(robot_chassis_odom_update(
        &odom, stationary_counts, 0.01f, metres_per_count,
        turn_geometry_m));
    CHECK(fabsf(odom.x_m) < 1.0e-7f && fabsf(odom.y_m) < 1.0e-7f &&
          fabsf(odom.yaw_rad) < 1.0e-7f);
    CHECK(fabsf(odom.linear_x_mps) < 1.0e-7f &&
          fabsf(odom.angular_z_radps) < 1.0e-7f);

    const int straight_counts[4] = {10, 10, 10, 10};
    CHECK(robot_chassis_odom_update(
        &odom, straight_counts, 0.01f, metres_per_count,
        turn_geometry_m));
    const float straight_x_m = odom.x_m;
    CHECK(fabsf(straight_x_m - 10.0f * metres_per_count) < 1.0e-7f);
    CHECK(fabsf(odom.y_m) < 1.0e-7f && fabsf(odom.yaw_rad) < 1.0e-7f);
    CHECK(odom.linear_x_mps > 0.0f &&
          fabsf(odom.angular_z_radps) < 1.0e-7f);

    const int rotate_counts[4] = {-10, -10, 10, 10};
    CHECK(robot_chassis_odom_update(
        &odom, rotate_counts, 0.01f, metres_per_count,
        turn_geometry_m));
    CHECK(fabsf(odom.x_m - straight_x_m) < 1.0e-7f &&
          fabsf(odom.y_m) < 1.0e-7f);
    CHECK(odom.yaw_rad > 0.0f && odom.angular_z_radps > 0.0f);
    CHECK(fabsf(odom.linear_x_mps) < 1.0e-7f);

    robot_chassis_odom_t wrapped_odom = {0};
    int wrapped_counts[4];
    for (int i = 0; i < 4; ++i) {
        wrapped_counts[i] = robot_chassis_safe_count_delta(
            0, 29999, pcnt_limit);
    }
    CHECK(robot_chassis_odom_update(
        &wrapped_odom, wrapped_counts, 0.01f, metres_per_count,
        turn_geometry_m));
    CHECK(fabsf(wrapped_odom.x_m - metres_per_count) < 1.0e-7f);
    CHECK(fabsf(wrapped_odom.yaw_rad) < 1.0e-7f);

    robot_chassis_core_t core = make_core(4);
    const int zero[4] = {0, 0, 0, 0};
    const int forward_m1[4] = {2, 0, 0, 0};
    int delta[4] = {0, 0, 0, 0};

    CHECK(core.state == ROBOT_CHASSIS_STATE_DISARMED);
    CHECK(core.stop_reason == ROBOT_CHASSIS_STOP_STARTUP);
    CHECK(!robot_chassis_core_arm(&core, 0));
    CHECK(!robot_chassis_core_submit(&core, forward_m1, 0));

    robot_chassis_core_accept_zero(&core, 10);
    CHECK(core.zero_seen);
    CHECK(robot_chassis_core_arm(&core, 20));
    CHECK(core.state == ROBOT_CHASSIS_STATE_ARMED);
    CHECK(robot_chassis_core_submit(&core, forward_m1, 30));
    robot_chassis_core_step(&core, delta, 40);
    CHECK(core.output[0] > 0);
    CHECK(core.output[0] <= 20);
    CHECK(core.output[1] == 0 && core.output[2] == 0 && core.output[3] == 0);

    delta[0] = 1;
    robot_chassis_core_step(&core, delta, 50);
    CHECK(core.output[0] > 0);
    delta[0] = 3;
    robot_chassis_core_step(&core, delta, 60);
    CHECK(core.output[0] < 0);

    robot_chassis_core_step(&core, zero, 531);
    CHECK(core.stop_reason == ROBOT_CHASSIS_STOP_TIMEOUT);
    CHECK(core.state == ROBOT_CHASSIS_STATE_DISARMED);
    CHECK(!core.zero_seen);
    CHECK(core.output[0] == 0);
    CHECK(!robot_chassis_core_submit(&core, forward_m1, 540));
    const float pose_before_stopped_step = wrapped_odom.x_m;
    CHECK(robot_chassis_odom_update(
        &wrapped_odom, zero, 0.01f, metres_per_count, turn_geometry_m));
    CHECK(fabsf(wrapped_odom.x_m - pose_before_stopped_step) < 1.0e-7f);
    CHECK(!robot_chassis_core_arm(&core, 541));
    robot_chassis_core_accept_zero(&core, 550);
    CHECK(robot_chassis_core_arm(&core, 560));
    CHECK(robot_chassis_core_submit(&core, forward_m1, 570));
    robot_chassis_core_step(&core, delta, 571);
    CHECK(core.stop_reason == ROBOT_CHASSIS_STOP_NONE);

    const int invalid[4] = {4, 0, 0, 0};
    CHECK(!robot_chassis_core_submit(&core, invalid, 580));
    CHECK(core.state == ROBOT_CHASSIS_STATE_FAULT_LATCHED);
    CHECK(core.stop_reason == ROBOT_CHASSIS_STOP_INVALID_TARGET);
    CHECK(!robot_chassis_core_arm(&core, 590));
    CHECK(!robot_chassis_core_clear_fault(&core));
    robot_chassis_core_accept_zero(&core, 600);
    CHECK(robot_chassis_core_clear_fault(&core));
    CHECK(core.state == ROBOT_CHASSIS_STATE_DISARMED);
    CHECK(robot_chassis_core_arm(&core, 610));

    CHECK(robot_chassis_core_submit(&core, forward_m1, 620));
    for (uint32_t t = 630; t <= 660; t += 10) {
        robot_chassis_core_step(&core, zero, t);
    }
    CHECK(core.state == ROBOT_CHASSIS_STATE_FAULT_LATCHED);
    CHECK(core.stop_reason == ROBOT_CHASSIS_STOP_ENCODER_STALL);
    CHECK(core.output[0] == 0);

    robot_chassis_core_accept_zero(&core, 670);
    CHECK(robot_chassis_core_clear_fault(&core));
    CHECK(robot_chassis_core_arm(&core, 680));
    CHECK(robot_chassis_core_submit(&core, zero, 690));
    CHECK(core.stop_reason == ROBOT_CHASSIS_STOP_EXPLICIT);
    CHECK(core.state == ROBOT_CHASSIS_STATE_ARMED);
    robot_chassis_core_accept_zero(&core, 700);
    CHECK(core.state == ROBOT_CHASSIS_STATE_DISARMED);
    CHECK(core.zero_seen);
    robot_chassis_core_stop(&core, ROBOT_CHASSIS_STOP_EXPLICIT);
    CHECK(core.state == ROBOT_CHASSIS_STATE_DISARMED);
    CHECK(strcmp(robot_chassis_state_string(core.state), "DISARMED") == 0);
    CHECK(strcmp(robot_chassis_stop_reason_string(core.stop_reason), "explicit") == 0);

    robot_chassis_core_t bounded_high_power;
    const robot_chassis_core_config_t high_power_config = {
        .kp = 5.0f,
        .ki_per_step = 0.25f,
        .integral_limit = 40.0f,
        .max_output = 100,
        .max_target_counts = 20,
        .command_timeout_ms = 500,
        .stall_steps = 100,
    };
    const int high_target[4] = {20, 0, 0, 0};
    CHECK(robot_chassis_core_init(&bounded_high_power, &high_power_config));
    CHECK(!robot_chassis_core_update_gains(&bounded_high_power, NAN, 0.25f));
    CHECK(robot_chassis_core_update_gains(&bounded_high_power, 3.0f, 0.10f));
    CHECK(bounded_high_power.config.kp == 3.0f);
    CHECK(bounded_high_power.config.ki_per_step == 0.10f);
    CHECK(!bounded_high_power.zero_seen);
    CHECK(!robot_chassis_core_arm(&bounded_high_power, 710));
    robot_chassis_core_accept_zero(&bounded_high_power, 720);
    CHECK(robot_chassis_core_arm(&bounded_high_power, 730));
    CHECK(!robot_chassis_core_update_gains(&bounded_high_power, 5.0f, 0.25f));
    CHECK(robot_chassis_core_submit(&bounded_high_power, high_target, 740));
    robot_chassis_core_step(&bounded_high_power, zero, 750);
    CHECK(bounded_high_power.output[0] > 0);
    CHECK(bounded_high_power.output[0] <= 100);
    CHECK(bounded_high_power.output[1] == 0 &&
          bounded_high_power.output[2] == 0 &&
          bounded_high_power.output[3] == 0);

    int twist_targets[4] = {0};
    bool limited = false;
    const float counts_per_metre_per_step = 1040.0f * 0.01f / 0.1508f;
    CHECK(robot_chassis_core_twist_to_targets(
        0.20f, 0.0f, 0.115f, counts_per_metre_per_step, 20,
        twist_targets, &limited));
    CHECK(!limited);
    CHECK(twist_targets[0] == 14 && twist_targets[1] == 14 &&
          twist_targets[2] == 14 && twist_targets[3] == 14);

    CHECK(robot_chassis_core_twist_to_targets(
        0.0f, 1.0f, 0.115f, counts_per_metre_per_step, 20,
        twist_targets, &limited));
    CHECK(!limited);
    CHECK(twist_targets[0] == -8 && twist_targets[1] == -8 &&
          twist_targets[2] == 8 && twist_targets[3] == 8);

    CHECK(robot_chassis_core_twist_to_targets(
        1.0f, 0.0f, 0.115f, counts_per_metre_per_step, 20,
        twist_targets, &limited));
    CHECK(limited);
    CHECK(twist_targets[0] == 20 && twist_targets[1] == 20 &&
          twist_targets[2] == 20 && twist_targets[3] == 20);
    CHECK(!robot_chassis_core_twist_to_targets(
        NAN, 0.0f, 0.115f, counts_per_metre_per_step, 20,
        twist_targets, &limited));

    printf("robot_chassis_diag_core: %d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
