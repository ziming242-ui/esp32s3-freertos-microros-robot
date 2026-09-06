#include "robot_drive_core.h"

#include <math.h>
#include <stdio.h>

static unsigned s_checks;
static unsigned s_failures;

#define CHECK(condition)                                                     \
    do {                                                                     \
        ++s_checks;                                                          \
        if (!(condition)) {                                                  \
            ++s_failures;                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,       \
                    #condition);                                             \
        }                                                                    \
    } while (0)

static void check_near(float actual, float expected, float tolerance)
{
    CHECK(isfinite(actual));
    CHECK(fabsf(actual - expected) <= tolerance);
}

static void check_zeroed(const robot_drive_core_output_t *output)
{
    for (int wheel = 0; wheel < ROBOT_DRIVE_WHEEL_COUNT; ++wheel) {
        check_near(output->target_mps[wheel], 0.0f, 0.0f);
    }
    CHECK(!output->limited);
}

static robot_drive_core_config_t default_config(void)
{
    const robot_drive_core_config_t config = {
        .turn_geometry_m = 0.115f,
        .max_target_mps = 1.0f,
    };
    return config;
}

static void test_straight(void)
{
    const robot_drive_core_config_t config = default_config();
    const robot_drive_core_input_t input = {
        .linear_x_mps = 0.4f,
        .angular_z_radps = 0.0f,
    };
    robot_drive_core_output_t output;

    CHECK(robot_drive_core_compute_targets(&config, &input, &output) ==
          ROBOT_DRIVE_CORE_OK);
    for (int wheel = 0; wheel < ROBOT_DRIVE_WHEEL_COUNT; ++wheel) {
        check_near(output.target_mps[wheel], 0.4f, 1.0e-6f);
    }
    CHECK(!output.limited);
}

static void test_turning(void)
{
    const robot_drive_core_config_t config = default_config();
    const robot_drive_core_input_t input = {
        .linear_x_mps = 0.3f,
        .angular_z_radps = 0.8f,
    };
    robot_drive_core_output_t output;

    CHECK(robot_drive_core_compute_targets(&config, &input, &output) ==
          ROBOT_DRIVE_CORE_OK);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_LF], 0.208f, 1.0e-6f);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_LR], 0.208f, 1.0e-6f);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_RF], 0.392f, 1.0e-6f);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_RR], 0.392f, 1.0e-6f);
    CHECK(!output.limited);
}

static void test_ratio_preserving_limit(void)
{
    const robot_drive_core_config_t config = {
        .turn_geometry_m = 0.25f,
        .max_target_mps = 0.5f,
    };
    const robot_drive_core_input_t input = {
        .linear_x_mps = 1.0f,
        .angular_z_radps = 2.0f,
    };
    robot_drive_core_output_t output;

    CHECK(robot_drive_core_compute_targets(&config, &input, &output) ==
          ROBOT_DRIVE_CORE_OK);
    CHECK(output.limited);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_LF], 1.0f / 6.0f,
               1.0e-6f);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_LR], 1.0f / 6.0f,
               1.0e-6f);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_RF], 0.5f, 1.0e-6f);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_RR], 0.5f, 1.0e-6f);
    check_near(output.target_mps[ROBOT_DRIVE_WHEEL_RF] /
                   output.target_mps[ROBOT_DRIVE_WHEEL_LF],
               3.0f, 1.0e-5f);
}

static void test_nan_command_is_zeroed(void)
{
    const robot_drive_core_config_t config = default_config();
    const robot_drive_core_input_t input = {
        .linear_x_mps = NAN,
        .angular_z_radps = 0.0f,
    };
    robot_drive_core_output_t output = {
        .target_mps = {1.0f, 2.0f, 3.0f, 4.0f},
        .limited = true,
    };

    CHECK(robot_drive_core_compute_targets(&config, &input, &output) ==
          ROBOT_DRIVE_CORE_INVALID_COMMAND);
    check_zeroed(&output);
}

static void test_invalid_config_is_zeroed(void)
{
    robot_drive_core_config_t config = default_config();
    const robot_drive_core_input_t input = {
        .linear_x_mps = 0.4f,
        .angular_z_radps = 0.0f,
    };
    robot_drive_core_output_t output = {
        .target_mps = {1.0f, 2.0f, 3.0f, 4.0f},
        .limited = true,
    };

    config.turn_geometry_m = 0.0f;
    CHECK(robot_drive_core_compute_targets(&config, &input, &output) ==
          ROBOT_DRIVE_CORE_INVALID_CONFIG);
    check_zeroed(&output);

    output.target_mps[0] = 9.0f;
    output.limited = true;
    config = default_config();
    config.max_target_mps = -1.0f;
    CHECK(robot_drive_core_compute_targets(&config, &input, &output) ==
          ROBOT_DRIVE_CORE_INVALID_CONFIG);
    check_zeroed(&output);
}

int main(void)
{
    test_straight();
    test_turning();
    test_ratio_preserving_limit();
    test_nan_command_is_zeroed();
    test_invalid_config_is_zeroed();

    printf("[PC simulation] robot drive core: %u checks, %u failures.\n",
           s_checks, s_failures);
    return s_failures == 0U ? 0 : 1;
}
