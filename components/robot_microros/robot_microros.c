#include "robot_microros.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "geometry_msgs/msg/twist.h"
#include "micro_ros_utilities/string_utilities.h"
#include "nav_msgs/msg/odometry.h"
#include "rcl/context.h"
#include "rcl/error_handling.h"
#include "rcl/publisher.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/time_sync.h"
#include "rmw_microros/timing.h"
#include "robot_chassis_diag.h"
#include "robot_drive.h"
#include "robot_imu.h"
#include "sdkconfig.h"
#include "sensor_msgs/msg/imu.h"
#include "std_msgs/msg/int32.h"

static const char *TAG = "robot_microros";

static rcl_publisher_t s_heartbeat_publisher;
static rcl_publisher_t s_imu_publisher;
static rcl_publisher_t s_odom_publisher;
static rcl_subscription_t s_command_subscription;
static rcl_subscription_t s_cmd_vel_subscription;
static std_msgs__msg__Int32 s_heartbeat_msg;
static std_msgs__msg__Int32 s_command_msg;
static geometry_msgs__msg__Twist s_cmd_vel_msg;
static sensor_msgs__msg__Imu s_imu_msg;
static nav_msgs__msg__Odometry s_odom_msg;

static bool s_time_synced;
static int64_t s_next_time_sync_at_us;
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
static bool s_has_published_odom_sample;
static int64_t s_last_published_odom_sampled_at_us;
static int64_t s_last_published_odom_stamp_ns;
#endif
static bool s_has_published_imu_sample;
static uint32_t s_last_published_imu_sequence;
static int64_t s_last_published_imu_stamp_ns;
static uint32_t s_imu_coalesced_events_since_log;
static int64_t s_next_imu_coalesced_log_at_us;
static robot_imu_sample_state_t s_last_imu_state =
    ROBOT_IMU_SAMPLE_ARGUMENT_INVALID;

#define ROBOT_COMMAND_TIMEOUT_MS 500
#define ROBOT_AGENT_SESSION_PING_PERIOD_MS 1000
#define ROBOT_AGENT_SESSION_PING_TIMEOUT_MS 100
#define ROBOT_AGENT_SESSION_PING_FAILURE_LIMIT 3U
#define ROBOT_AGENT_DISCOVERY_PING_TIMEOUT_MS 250
#define ROBOT_AGENT_DISCOVERY_RETRY_MS 500
#define ROBOT_AGENT_WAIT_LOG_PERIOD_MS 5000
#define ROBOT_RCL_ERROR_LOG_PERIOD_MS 1000
#define ROBOT_RELIABLE_PUBLISH_TIMEOUT_MS 100

typedef struct {
    int32_t value;
    int64_t received_at_ms;
} robot_command_cache_t;

/* Length one gives motion commands "latest value" semantics. */
static QueueHandle_t s_command_queue;

typedef struct {
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t heartbeat_timer;
    rcl_timer_t imu_timer;
    rclc_executor_t executor;
    bool support_initialized;
    bool node_initialized;
    bool heartbeat_publisher_initialized;
    bool imu_publisher_initialized;
    bool odom_publisher_initialized;
    bool command_subscription_initialized;
    bool cmd_vel_subscription_initialized;
    bool heartbeat_timer_initialized;
    bool imu_timer_initialized;
    bool executor_initialized;
} robot_microros_entities_t;

static int64_t s_next_rcl_error_log_at_us;

static bool request_motion_stop(robot_drive_stop_reason_t reason)
{
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    (void)reason;
    return robot_chassis_diag_remote_stop();
#else
    return robot_drive_request_stop(reason);
#endif
}

static bool submit_motion_command(float linear_x_mps,
                                  float angular_z_radps)
{
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    return robot_chassis_diag_submit_cmd_vel(
        linear_x_mps, angular_z_radps);
#else
    return robot_drive_submit_cmd_vel(linear_x_mps, angular_z_radps);
#endif
}

static void log_rcl_runtime_failure(const char *operation, rcl_ret_t rc)
{
    const int64_t now_us = esp_timer_get_time();
    if (now_us >= s_next_rcl_error_log_at_us) {
        ESP_LOGW(TAG, "%s failed: rc=%d", operation, (int)rc);
        s_next_rcl_error_log_at_us =
            now_us + (int64_t)ROBOT_RCL_ERROR_LOG_PERIOD_MS * 1000;
    }
    rcl_reset_error();
}

static bool rcl_init_step_succeeded(const char *operation, rcl_ret_t rc)
{
    if (rc == RCL_RET_OK) {
        return true;
    }

    ESP_LOGE(TAG, "%s failed: rc=%d; detail=%s",
             operation,
             (int)rc,
             rcl_get_error_string().str);
    rcl_reset_error();
    return false;
}

static void log_rcl_cleanup_failure(const char *operation, rcl_ret_t rc)
{
    if (rc == RCL_RET_OK) {
        return;
    }

    ESP_LOGW(TAG, "cleanup %s failed: rc=%d", operation, (int)rc);
    rcl_reset_error();
}

static void reset_entity_handles(robot_microros_entities_t *entities)
{
    *entities = (robot_microros_entities_t){0};
    entities->node = rcl_get_zero_initialized_node();
    entities->heartbeat_timer = rcl_get_zero_initialized_timer();
    entities->imu_timer = rcl_get_zero_initialized_timer();
    entities->executor = rclc_executor_get_zero_initialized_executor();
    s_heartbeat_publisher = rcl_get_zero_initialized_publisher();
    s_imu_publisher = rcl_get_zero_initialized_publisher();
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    s_command_subscription = rcl_get_zero_initialized_subscription();
    s_cmd_vel_subscription = rcl_get_zero_initialized_subscription();
}

#if !defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
static void heartbeat_timer_callback(rcl_timer_t *timer,
                                     int64_t last_call_time)
{
    (void)last_call_time;

    if (timer == NULL) {
        return;
    }

    const rcl_ret_t rc = rcl_publish(
        &s_heartbeat_publisher, &s_heartbeat_msg, NULL);
    if (rc == RCL_RET_OK) {
        ESP_LOGI(TAG, "published heartbeat=%ld",
                 (long)s_heartbeat_msg.data);
        s_heartbeat_msg.data++;
    } else {
        log_rcl_runtime_failure("heartbeat publish", rc);
    }
}
#endif

static bool supported_twist_is_valid(const geometry_msgs__msg__Twist *command)
{
    if (command == NULL) {
        return false;
    }

    const bool all_finite =
        isfinite(command->linear.x) && isfinite(command->linear.y) &&
        isfinite(command->linear.z) && isfinite(command->angular.x) &&
        isfinite(command->angular.y) && isfinite(command->angular.z);
    const double unsupported_epsilon = 1.0e-6;
    const bool unsupported_axes_are_zero =
        fabs(command->linear.y) <= unsupported_epsilon &&
        fabs(command->linear.z) <= unsupported_epsilon &&
        fabs(command->angular.x) <= unsupported_epsilon &&
        fabs(command->angular.y) <= unsupported_epsilon;
    return all_finite && unsupported_axes_are_zero;
}

static void cmd_vel_subscription_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *command =
        (const geometry_msgs__msg__Twist *)msg_in;
    if (!supported_twist_is_valid(command)) {
        (void)request_motion_stop(ROBOT_DRIVE_STOP_INVALID_COMMAND);
        ESP_LOGW(TAG, "rejected invalid or unsupported /cmd_vel");
        return;
    }

    if (!submit_motion_command(
            (float)command->linear.x, (float)command->angular.z)) {
        (void)request_motion_stop(ROBOT_DRIVE_STOP_EXTERNAL_REQUEST);
        ESP_LOGW(TAG, "failed to submit /cmd_vel to motion backend");
    }
}

static void log_imu_state_transition(robot_imu_sample_state_t state)
{
    if (state == s_last_imu_state) {
        return;
    }
    if (state == ROBOT_IMU_SAMPLE_FRESH) {
        ESP_LOGI(TAG, "IMU sample state: fresh");
    } else {
        ESP_LOGW(TAG, "IMU sample state: %s; publish skipped",
                 robot_imu_sample_state_name(state));
    }
    s_last_imu_state = state;
}

#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
static void publish_odom_snapshot(void)
{
    robot_chassis_odom_snapshot_t snapshot;
    if (!robot_chassis_diag_get_odom_snapshot(&snapshot) ||
        !s_time_synced || !rmw_uros_epoch_synchronized()) {
        return;
    }
    if (s_has_published_odom_sample &&
        snapshot.sampled_at_us <= s_last_published_odom_sampled_at_us) {
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    int64_t sample_age_us = now_us - snapshot.sampled_at_us;
    if (sample_age_us < 0) {
        sample_age_us = 0;
    }
    const int64_t epoch_now_ns = rmw_uros_epoch_nanos();
    const int64_t sampled_at_ns = epoch_now_ns - sample_age_us * 1000;
    if ((epoch_now_ns <= 0) || (sampled_at_ns <= 0)) {
        ESP_LOGW(TAG, "invalid synchronized timestamp; odometry publish skipped");
        return;
    }
    if (s_has_published_odom_sample &&
        sampled_at_ns <= s_last_published_odom_stamp_ns) {
        ESP_LOGW(TAG,
                 "non-monotonic odometry timestamp skipped: "
                 "previous=%lld current=%lld",
                 (long long)s_last_published_odom_stamp_ns,
                 (long long)sampled_at_ns);
        return;
    }

    s_odom_msg.header.stamp.sec =
        (int32_t)(sampled_at_ns / 1000000000LL);
    s_odom_msg.header.stamp.nanosec =
        (uint32_t)(sampled_at_ns % 1000000000LL);
    s_odom_msg.pose.pose.position.x = snapshot.x_m;
    s_odom_msg.pose.pose.position.y = snapshot.y_m;
    s_odom_msg.pose.pose.position.z = 0.0;
    const double half_yaw = 0.5 * (double)snapshot.yaw_rad;
    s_odom_msg.pose.pose.orientation.x = 0.0;
    s_odom_msg.pose.pose.orientation.y = 0.0;
    s_odom_msg.pose.pose.orientation.z = sin(half_yaw);
    s_odom_msg.pose.pose.orientation.w = cos(half_yaw);
    s_odom_msg.twist.twist.linear.x = snapshot.linear_x_mps;
    s_odom_msg.twist.twist.linear.y = 0.0;
    s_odom_msg.twist.twist.linear.z = 0.0;
    s_odom_msg.twist.twist.angular.x = 0.0;
    s_odom_msg.twist.twist.angular.y = 0.0;
    s_odom_msg.twist.twist.angular.z = snapshot.angular_z_radps;

    const rcl_ret_t rc = rcl_publish(&s_odom_publisher, &s_odom_msg, NULL);
    if (rc == RCL_RET_OK) {
        s_last_published_odom_sampled_at_us = snapshot.sampled_at_us;
        s_last_published_odom_stamp_ns = sampled_at_ns;
        s_has_published_odom_sample = true;
    } else {
        log_rcl_runtime_failure("odometry publish", rc);
    }
}
#endif

static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    publish_odom_snapshot();
#endif

    if (!robot_imu_is_enabled()) {
        return;
    }

    robot_imu_sample_t sample;
    const bool has_sample = robot_imu_get_latest(&sample);
    const int64_t now_us = esp_timer_get_time();
    const robot_imu_sample_state_t state = robot_imu_classify_sample(
        has_sample,
        has_sample ? &sample : NULL,
        now_us,
        robot_imu_stale_timeout_us());
    log_imu_state_transition(state);
    if (state != ROBOT_IMU_SAMPLE_FRESH) {
        return;
    }

    if (s_has_published_imu_sample &&
        sample.sample_sequence == s_last_published_imu_sequence) {
        return;
    }

    if (!s_time_synced || !rmw_uros_epoch_synchronized()) {
        return;
    }

    const int64_t epoch_now_ns = rmw_uros_epoch_nanos();
    const int64_t sample_age_us = now_us - sample.sampled_at_us;
    const int64_t sampled_at_ns = epoch_now_ns - sample_age_us * 1000;
    if (epoch_now_ns <= 0 || sampled_at_ns <= 0) {
        ESP_LOGW(TAG, "invalid synchronized timestamp; IMU publish skipped");
        return;
    }
    if (s_has_published_imu_sample &&
        sampled_at_ns <= s_last_published_imu_stamp_ns) {
        ESP_LOGW(TAG,
                 "non-monotonic IMU timestamp skipped: previous=%lld current=%lld",
                 (long long)s_last_published_imu_stamp_ns,
                 (long long)sampled_at_ns);
        return;
    }

    s_imu_msg.header.stamp.sec = (int32_t)(sampled_at_ns / 1000000000LL);
    s_imu_msg.header.stamp.nanosec =
        (uint32_t)(sampled_at_ns % 1000000000LL);
    s_imu_msg.angular_velocity.x = sample.angular_velocity_rad_s[0];
    s_imu_msg.angular_velocity.y = sample.angular_velocity_rad_s[1];
    s_imu_msg.angular_velocity.z = sample.angular_velocity_rad_s[2];
    s_imu_msg.linear_acceleration.x = sample.linear_acceleration_mps2[0];
    s_imu_msg.linear_acceleration.y = sample.linear_acceleration_mps2[1];
    s_imu_msg.linear_acceleration.z = sample.linear_acceleration_mps2[2];

    const rcl_ret_t rc = rcl_publish(&s_imu_publisher, &s_imu_msg, NULL);
    if (rc == RCL_RET_OK) {
        if (s_has_published_imu_sample) {
            uint32_t expected_sequence = s_last_published_imu_sequence + 1U;
            if (expected_sequence == 0U) {
                expected_sequence = 1U;
            }
            if (sample.sample_sequence != expected_sequence) {
                s_imu_coalesced_events_since_log++;
                if (now_us >= s_next_imu_coalesced_log_at_us) {
                    ESP_LOGI(TAG,
                             "IMU producer samples coalesced before ROS publish: "
                             "events=%lu "
                             "previous=%lu current=%lu",
                             (unsigned long)s_imu_coalesced_events_since_log,
                             (unsigned long)s_last_published_imu_sequence,
                             (unsigned long)sample.sample_sequence);
                    s_imu_coalesced_events_since_log = 0;
                    s_next_imu_coalesced_log_at_us = now_us + 1000000LL;
                }
            }
        }
        s_last_published_imu_sequence = sample.sample_sequence;
        s_last_published_imu_stamp_ns = sampled_at_ns;
        s_has_published_imu_sample = true;
    } else {
        log_rcl_runtime_failure("IMU publish", rc);
    }
}

static void refresh_time_sync_if_due(void)
{
    const int64_t now_us = esp_timer_get_time();
    if (now_us < s_next_time_sync_at_us) {
        return;
    }

    const rmw_ret_t rc = rmw_uros_sync_session(1000);
    s_time_synced = (rc == RMW_RET_OK) && rmw_uros_epoch_synchronized();
    if (s_time_synced) {
        s_next_time_sync_at_us = now_us +
            (int64_t)CONFIG_ROBOT_MICROROS_TIME_SYNC_PERIOD_MS * 1000;
        ESP_LOGI(TAG, "Agent time synchronized");
    } else {
        s_next_time_sync_at_us = now_us + 5000000LL;
        log_rcl_runtime_failure("Agent time synchronization", (rcl_ret_t)rc);
    }
}

static void command_subscription_callback(const void *msg_in)
{
    if (msg_in == NULL) {
        ESP_LOGW(TAG, "command callback received NULL message");
        return;
    }

    const std_msgs__msg__Int32 *command =
        (const std_msgs__msg__Int32 *)msg_in;

#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    bool remote_ok = false;
    if (command->data == 0) {
        remote_ok = robot_chassis_diag_remote_zero();
    } else if (command->data == 1) {
        remote_ok = robot_chassis_diag_remote_arm();
    } else {
        (void)robot_chassis_diag_remote_stop();
        ESP_LOGW(TAG,
                 "rejected remote chassis command=%ld; use 0=ZERO or 1=ARM",
                 (long)command->data);
        return;
    }
    if (!remote_ok) {
        (void)robot_chassis_diag_remote_stop();
        ESP_LOGE(TAG, "failed to queue remote chassis command=%ld",
                 (long)command->data);
        return;
    }
#endif

    const robot_command_cache_t latest = {
        .value = command->data,
        .received_at_ms = esp_timer_get_time() / 1000,
    };

    if (xQueueOverwrite(s_command_queue, &latest) != pdPASS) {
        ESP_LOGE(TAG, "failed to update latest-command cache");
        return;
    }

    ESP_LOGI(TAG, "received command=%ld; latest cache overwritten",
             (long)command->data);
}

static void check_command_timeout(void)
{
    robot_command_cache_t latest;
    if (xQueuePeek(s_command_queue, &latest, 0) != pdPASS) {
        return;
    }

    const int64_t age_ms = esp_timer_get_time() / 1000 - latest.received_at_ms;
    if (age_ms > ROBOT_COMMAND_TIMEOUT_MS) {
        ESP_LOGW(TAG,
                 "test command expired: age=%lld ms; cache cleared",
                 (long long)age_ms);
        xQueueReset(s_command_queue);
    }
}

static void destroy_ros_entities(robot_microros_entities_t *entities)
{
    if (entities->support_initialized) {
        rmw_context_t *rmw_context =
            rcl_context_get_rmw_context(&entities->support.context);
        if (rmw_context != NULL) {
            const rmw_ret_t rc =
                rmw_uros_set_context_entity_destroy_session_timeout(
                    rmw_context, 0);
            if (rc != RMW_RET_OK) {
                ESP_LOGW(TAG,
                         "failed to make entity cleanup non-blocking: rc=%d",
                         (int)rc);
            }
        }
    }

    /* Executor must release its references before the entities it serves. */
    if (entities->executor_initialized) {
        log_rcl_cleanup_failure(
            "executor", rclc_executor_fini(&entities->executor));
    }
    if (entities->imu_timer_initialized) {
        log_rcl_cleanup_failure(
            "IMU timer", rcl_timer_fini(&entities->imu_timer));
    }
    if (entities->heartbeat_timer_initialized) {
        log_rcl_cleanup_failure(
            "heartbeat timer", rcl_timer_fini(&entities->heartbeat_timer));
    }
    if (entities->cmd_vel_subscription_initialized) {
        log_rcl_cleanup_failure(
            "/cmd_vel subscription",
            rcl_subscription_fini(
                &s_cmd_vel_subscription, &entities->node));
    }
    if (entities->command_subscription_initialized) {
        log_rcl_cleanup_failure(
            "/robot/command subscription",
            rcl_subscription_fini(
                &s_command_subscription, &entities->node));
    }
    if (entities->imu_publisher_initialized) {
        log_rcl_cleanup_failure(
            "/imu publisher",
            rcl_publisher_fini(&s_imu_publisher, &entities->node));
    }
    if (entities->odom_publisher_initialized) {
        log_rcl_cleanup_failure(
            "/odom_raw publisher",
            rcl_publisher_fini(&s_odom_publisher, &entities->node));
    }
    if (entities->heartbeat_publisher_initialized) {
        log_rcl_cleanup_failure(
            "/robot/heartbeat publisher",
            rcl_publisher_fini(
                &s_heartbeat_publisher, &entities->node));
    }
    if (entities->node_initialized) {
        log_rcl_cleanup_failure("node", rcl_node_fini(&entities->node));
    }
    if (entities->support_initialized) {
        log_rcl_cleanup_failure(
            "support", rclc_support_fini(&entities->support));
    }

    reset_entity_handles(entities);
}

static bool configure_publisher_timeout(const char *name,
                                        rcl_publisher_t *publisher)
{
    rmw_publisher_t *rmw_publisher =
        rcl_publisher_get_rmw_handle(publisher);
    if (rmw_publisher == NULL) {
        ESP_LOGE(TAG, "%s RMW handle is unavailable", name);
        rcl_reset_error();
        return false;
    }

    const rmw_ret_t rc = rmw_uros_set_publisher_session_timeout(
        rmw_publisher, ROBOT_RELIABLE_PUBLISH_TIMEOUT_MS);
    if (rc != RMW_RET_OK) {
        ESP_LOGE(TAG, "%s publish timeout setup failed: rc=%d",
                 name, (int)rc);
        rcl_reset_error();
        return false;
    }
    return true;
}

static bool create_ros_entities(robot_microros_entities_t *entities,
                                rcl_init_options_t *init_options,
                                rcl_allocator_t *allocator)
{
    reset_entity_handles(entities);

    rcl_ret_t rc = rclc_support_init_with_options(
        &entities->support, 0, NULL, init_options, allocator);
    if (rc != RCL_RET_OK) {
        /* rclc may have created the context before a later clock failure. */
        entities->support_initialized =
            rcl_context_is_valid(&entities->support.context);
        (void)rcl_init_step_succeeded("rclc_support_init_with_options", rc);
        goto fail;
    }
    entities->support_initialized = true;

    rc = rclc_node_init_default(
        &entities->node, "esp32s3_robot", "", &entities->support);
    if (!rcl_init_step_succeeded("rclc_node_init_default", rc)) {
        goto fail;
    }
    entities->node_initialized = true;

#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    rc = rclc_publisher_init_default(
        &s_odom_publisher,
        &entities->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom_raw");
    if (!rcl_init_step_succeeded("odometry publisher init", rc)) {
        goto fail;
    }
    entities->odom_publisher_initialized = true;
    if (!configure_publisher_timeout("/odom_raw", &s_odom_publisher)) {
        goto fail;
    }
#else
    rc = rclc_publisher_init_default(
        &s_heartbeat_publisher,
        &entities->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/robot/heartbeat");
    if (!rcl_init_step_succeeded("heartbeat publisher init", rc)) {
        goto fail;
    }
    entities->heartbeat_publisher_initialized = true;
    if (!configure_publisher_timeout(
            "/robot/heartbeat", &s_heartbeat_publisher)) {
        goto fail;
    }
#endif

    rc = rclc_publisher_init_best_effort(
        &s_imu_publisher,
        &entities->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu");
    if (!rcl_init_step_succeeded("IMU publisher init", rc)) {
        goto fail;
    }
    entities->imu_publisher_initialized = true;

    rc = rclc_subscription_init_default(
        &s_command_subscription,
        &entities->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/robot/command");
    if (!rcl_init_step_succeeded("command subscription init", rc)) {
        goto fail;
    }
    entities->command_subscription_initialized = true;

    rc = rclc_subscription_init_default(
        &s_cmd_vel_subscription,
        &entities->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");
    if (!rcl_init_step_succeeded("cmd_vel subscription init", rc)) {
        goto fail;
    }
    entities->cmd_vel_subscription_initialized = true;

#if !defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    rc = rclc_timer_init_default(
        &entities->heartbeat_timer,
        &entities->support,
        RCL_MS_TO_NS(CONFIG_ROBOT_MICROROS_PUBLISH_PERIOD_MS),
        heartbeat_timer_callback);
    if (!rcl_init_step_succeeded("heartbeat timer init", rc)) {
        goto fail;
    }
    entities->heartbeat_timer_initialized = true;
#endif

    rc = rclc_timer_init_default(
        &entities->imu_timer,
        &entities->support,
        RCL_MS_TO_NS(CONFIG_ROBOT_MICROROS_IMU_PUBLISH_PERIOD_MS),
        imu_timer_callback);
    if (!rcl_init_step_succeeded("IMU timer init", rc)) {
        goto fail;
    }
    entities->imu_timer_initialized = true;

    const size_t executor_handles =
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
        3;
#else
        4;
#endif
    rc = rclc_executor_init(
        &entities->executor, &entities->support.context,
        executor_handles, allocator);
    if (!rcl_init_step_succeeded("executor init", rc)) {
        goto fail;
    }
    entities->executor_initialized = true;

#if !defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    rc = rclc_executor_add_timer(
        &entities->executor, &entities->heartbeat_timer);
    if (!rcl_init_step_succeeded("executor add heartbeat timer", rc)) {
        goto fail;
    }
#endif
    rc = rclc_executor_add_timer(&entities->executor, &entities->imu_timer);
    if (!rcl_init_step_succeeded("executor add IMU timer", rc)) {
        goto fail;
    }
    rc = rclc_executor_add_subscription(
        &entities->executor,
        &s_command_subscription,
        &s_command_msg,
        command_subscription_callback,
        ON_NEW_DATA);
    if (!rcl_init_step_succeeded("executor add command subscription", rc)) {
        goto fail;
    }
    rc = rclc_executor_add_subscription(
        &entities->executor,
        &s_cmd_vel_subscription,
        &s_cmd_vel_msg,
        cmd_vel_subscription_callback,
        ON_NEW_DATA);
    if (!rcl_init_step_succeeded("executor add cmd_vel subscription", rc)) {
        goto fail;
    }

    return true;

fail:
    destroy_ros_entities(entities);
    return false;
}

static bool initialize_ros_messages(void)
{
    s_heartbeat_msg.data = 0;
    s_command_msg.data = 0;

    if (!geometry_msgs__msg__Twist__init(&s_cmd_vel_msg)) {
        ESP_LOGE(TAG, "failed to initialize geometry_msgs/Twist");
        return false;
    }
    if (!sensor_msgs__msg__Imu__init(&s_imu_msg)) {
        ESP_LOGE(TAG, "failed to initialize sensor_msgs/Imu");
        geometry_msgs__msg__Twist__fini(&s_cmd_vel_msg);
        return false;
    }

#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    if (!nav_msgs__msg__Odometry__init(&s_odom_msg)) {
        ESP_LOGE(TAG, "failed to initialize nav_msgs/Odometry");
        sensor_msgs__msg__Imu__fini(&s_imu_msg);
        geometry_msgs__msg__Twist__fini(&s_cmd_vel_msg);
        return false;
    }
    s_odom_msg.header.frame_id = micro_ros_string_utilities_set(
        s_odom_msg.header.frame_id, "odom");
    s_odom_msg.child_frame_id = micro_ros_string_utilities_set(
        s_odom_msg.child_frame_id, "base_footprint");
    if ((s_odom_msg.header.frame_id.data == NULL) ||
        (s_odom_msg.child_frame_id.data == NULL)) {
        ESP_LOGE(TAG, "failed to allocate odometry frame strings");
        nav_msgs__msg__Odometry__fini(&s_odom_msg);
        sensor_msgs__msg__Imu__fini(&s_imu_msg);
        geometry_msgs__msg__Twist__fini(&s_cmd_vel_msg);
        return false;
    }
#endif

    s_imu_msg.header.frame_id = micro_ros_string_utilities_set(
        s_imu_msg.header.frame_id, robot_imu_frame_id());
    if (s_imu_msg.header.frame_id.data == NULL) {
        ESP_LOGE(TAG, "failed to allocate IMU frame_id");
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
        nav_msgs__msg__Odometry__fini(&s_odom_msg);
#endif
        sensor_msgs__msg__Imu__fini(&s_imu_msg);
        geometry_msgs__msg__Twist__fini(&s_cmd_vel_msg);
        return false;
    }
    s_imu_msg.orientation.x = 0.0;
    s_imu_msg.orientation.y = 0.0;
    s_imu_msg.orientation.z = 0.0;
    s_imu_msg.orientation.w = 1.0;
    s_imu_msg.orientation_covariance[0] = -1.0;
    return true;
}

static void wait_for_agent(rmw_init_options_t *rmw_options)
{
    int64_t next_wait_log_at_us = 0;
    while (true) {
        const rmw_ret_t rc = rmw_uros_ping_agent_options(
            ROBOT_AGENT_DISCOVERY_PING_TIMEOUT_MS, 1, rmw_options);
        if (rc == RMW_RET_OK) {
            ESP_LOGI(TAG, "Agent reachable at %s:%s",
                     CONFIG_ROBOT_MICROROS_AGENT_IP,
                     CONFIG_ROBOT_MICROROS_AGENT_PORT);
            return;
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_wait_log_at_us) {
            ESP_LOGW(TAG, "waiting for Agent at %s:%s",
                     CONFIG_ROBOT_MICROROS_AGENT_IP,
                     CONFIG_ROBOT_MICROROS_AGENT_PORT);
            next_wait_log_at_us = now_us +
                (int64_t)ROBOT_AGENT_WAIT_LOG_PERIOD_MS * 1000;
        }
        rcl_reset_error();
        vTaskDelay(pdMS_TO_TICKS(ROBOT_AGENT_DISCOVERY_RETRY_MS));
    }
}

static void prepare_new_ros_session(void)
{
    s_time_synced = false;
    s_next_time_sync_at_us = 0;
    s_imu_coalesced_events_since_log = 0;
    s_next_imu_coalesced_log_at_us = 0;
    s_last_imu_state = ROBOT_IMU_SAMPLE_ARGUMENT_INVALID;
    s_next_rcl_error_log_at_us = 0;
    xQueueReset(s_command_queue);
}

static void run_ros_session(robot_microros_entities_t *entities,
                            uint32_t session_generation)
{
    uint8_t consecutive_ping_failures = 0;
    int64_t next_agent_ping_at_us = esp_timer_get_time() +
        (int64_t)ROBOT_AGENT_SESSION_PING_PERIOD_MS * 1000;

    refresh_time_sync_if_due();
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
    ESP_LOGI(TAG,
             "ROS ready: generation=%lu "
             "pub=/odom_raw,/imu sub=/robot/command,/cmd_vel; "
             "odom_frame=odom child_frame=base_footprint "
             "IMU backend=%s frame=%s",
             (unsigned long)session_generation,
             robot_imu_backend_name(),
             robot_imu_frame_id());
#else
    ESP_LOGI(TAG,
             "ROS ready: generation=%lu "
             "pub=/robot/heartbeat,/imu sub=/robot/command,/cmd_vel; "
             "IMU backend=%s frame=%s",
             (unsigned long)session_generation,
             robot_imu_backend_name(),
             robot_imu_frame_id());
#endif

    while (true) {
        const rcl_ret_t spin_rc = rclc_executor_spin_some(
            &entities->executor, RCL_MS_TO_NS(5));
        if (spin_rc != RCL_RET_OK && spin_rc != RCL_RET_TIMEOUT) {
            log_rcl_runtime_failure("executor spin", spin_rc);
        }
        check_command_timeout();
        refresh_time_sync_if_due();

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_agent_ping_at_us) {
            const rmw_ret_t ping_rc = rmw_uros_ping_agent(
                ROBOT_AGENT_SESSION_PING_TIMEOUT_MS, 1);
            next_agent_ping_at_us = now_us +
                (int64_t)ROBOT_AGENT_SESSION_PING_PERIOD_MS * 1000;

            if (ping_rc == RMW_RET_OK) {
                if (consecutive_ping_failures > 0) {
                    ESP_LOGI(TAG,
                             "Agent session ping recovered after %u failure(s)",
                             (unsigned)consecutive_ping_failures);
                }
                consecutive_ping_failures = 0;
            } else {
                consecutive_ping_failures++;
                ESP_LOGW(TAG,
                         "Agent session ping failed: rc=%d consecutive=%u/%u",
                         (int)ping_rc,
                         (unsigned)consecutive_ping_failures,
                         (unsigned)ROBOT_AGENT_SESSION_PING_FAILURE_LIMIT);
                rcl_reset_error();
                if (consecutive_ping_failures >=
                    ROBOT_AGENT_SESSION_PING_FAILURE_LIMIT) {
                    return;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void micro_ros_task(void *arg)
{
    (void)arg;

    bool messages_initialized = false;
    bool init_options_initialized = false;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();
    robot_microros_entities_t entities;
    reset_entity_handles(&entities);

    s_command_queue = xQueueCreate(1, sizeof(robot_command_cache_t));
    if (s_command_queue == NULL) {
        ESP_LOGE(TAG, "failed to create latest-command queue");
        goto fatal;
    }

    messages_initialized = initialize_ros_messages();
    if (!messages_initialized) {
        goto fatal;
    }

    /* A1: configure the ROS domain and Agent UDP endpoint. */
    rcl_ret_t rcl_rc = rcl_init_options_init(&init_options, allocator);
    if (!rcl_init_step_succeeded("rcl_init_options_init", rcl_rc)) {
        goto fatal;
    }
    init_options_initialized = true;

    rcl_rc = rcl_init_options_set_domain_id(
        &init_options, CONFIG_ROBOT_MICROROS_DOMAIN_ID);
    if (!rcl_init_step_succeeded("rcl_init_options_set_domain_id", rcl_rc)) {
        goto fatal;
    }

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    if (rmw_options == NULL) {
        ESP_LOGE(TAG, "failed to get RMW init options");
        goto fatal;
    }
    const rmw_ret_t rmw_rc = rmw_uros_options_set_udp_address(
        CONFIG_ROBOT_MICROROS_AGENT_IP,
        CONFIG_ROBOT_MICROROS_AGENT_PORT,
        rmw_options);
    if (rmw_rc != RMW_RET_OK) {
        ESP_LOGE(TAG, "failed to set Agent UDP endpoint: rc=%d", (int)rmw_rc);
        rcl_reset_error();
        goto fatal;
    }

    uint32_t session_generation = 0;
    while (true) {
        /* A2: discover Agent without creating a participant or node. */
        wait_for_agent(rmw_options);

        /* A3-A5: exactly one complete entity set exists per live session. */
        ESP_LOGI(TAG, "creating ROS entities");
        if (!create_ros_entities(&entities, &init_options, &allocator)) {
            ESP_LOGW(TAG,
                     "ROS entity creation failed; returning to Agent discovery");
            vTaskDelay(pdMS_TO_TICKS(ROBOT_AGENT_DISCOVERY_RETRY_MS));
            continue;
        }

        session_generation++;
        prepare_new_ros_session();
        run_ros_session(&entities, session_generation);

        ESP_LOGE(TAG,
                 "Agent disconnected after %u consecutive session ping "
                 "failures; clearing command state and rebuilding",
                 (unsigned)ROBOT_AGENT_SESSION_PING_FAILURE_LIMIT);
        xQueueReset(s_command_queue);
        (void)request_motion_stop(ROBOT_DRIVE_STOP_EXTERNAL_REQUEST);
        s_time_synced = false;
        destroy_ros_entities(&entities);
        ESP_LOGI(TAG, "ROS entities cleaned; returning to Agent discovery");
    }

fatal:
    (void)request_motion_stop(ROBOT_DRIVE_STOP_EXTERNAL_REQUEST);
    destroy_ros_entities(&entities);
    if (init_options_initialized) {
        log_rcl_cleanup_failure(
            "init options", rcl_init_options_fini(&init_options));
    }
    if (messages_initialized) {
#if defined(CONFIG_ROBOT_APP_MODE_MICROROS_CHASSIS)
        nav_msgs__msg__Odometry__fini(&s_odom_msg);
#endif
        sensor_msgs__msg__Imu__fini(&s_imu_msg);
        geometry_msgs__msg__Twist__fini(&s_cmd_vel_msg);
    }
    if (s_command_queue != NULL) {
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
    }
    ESP_LOGE(TAG, "micro-ROS task stopped after unrecoverable local setup error");
    vTaskDelete(NULL);
}

esp_err_t robot_microros_start(void)
{
    const BaseType_t created = xTaskCreate(
        micro_ros_task,
        "micro_ros_task",
        CONFIG_ROBOT_MICROROS_TASK_STACK,
        NULL,
        CONFIG_ROBOT_MICROROS_TASK_PRIORITY,
        NULL);

    return (created == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}
