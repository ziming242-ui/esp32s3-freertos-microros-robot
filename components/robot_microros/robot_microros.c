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
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/time_sync.h"
#include "robot_drive.h"
#include "robot_imu.h"
#include "sensor_msgs/msg/imu.h"
#include "std_msgs/msg/int32.h"

static const char *TAG = "robot_microros";

static rcl_publisher_t s_heartbeat_publisher;
static rcl_publisher_t s_imu_publisher;
static rcl_subscription_t s_command_subscription;
static rcl_subscription_t s_cmd_vel_subscription;
static std_msgs__msg__Int32 s_heartbeat_msg;
static std_msgs__msg__Int32 s_command_msg;
static geometry_msgs__msg__Twist s_cmd_vel_msg;
static sensor_msgs__msg__Imu s_imu_msg;

static bool s_time_synced;
static int64_t s_next_time_sync_at_us;
static bool s_has_published_imu_sample;
static uint32_t s_last_published_imu_sequence;
static robot_imu_sample_state_t s_last_imu_state =
    ROBOT_IMU_SAMPLE_ARGUMENT_INVALID;

#define ROBOT_COMMAND_TIMEOUT_MS 500

typedef struct {
    int32_t value;
    int64_t received_at_ms;
} robot_command_cache_t;

/* Length one gives motion commands "latest value" semantics. */
static QueueHandle_t s_command_queue;

#define RCL_CHECK_OR_DELETE_TASK(call)                                      \
    do {                                                                    \
        const rcl_ret_t rc = (call);                                        \
        if (rc != RCL_RET_OK) {                                             \
            ESP_LOGE(TAG, "%s failed: rc=%d", #call, (int)rc);            \
            vTaskDelete(NULL);                                              \
        }                                                                   \
    } while (0)

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
        ESP_LOGW(TAG, "heartbeat publish failed: rc=%d", (int)rc);
    }
}

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
        (void)robot_drive_request_stop(ROBOT_DRIVE_STOP_INVALID_COMMAND);
        ESP_LOGW(TAG, "rejected invalid or unsupported /cmd_vel");
        return;
    }

    if (!robot_drive_submit_cmd_vel(
            (float)command->linear.x, (float)command->angular.z)) {
        ESP_LOGW(TAG, "failed to submit /cmd_vel to drive dry-run");
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

static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL || !robot_imu_is_enabled()) {
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
                ESP_LOGW(TAG,
                         "IMU sequence jump: previous=%lu current=%lu",
                         (unsigned long)s_last_published_imu_sequence,
                         (unsigned long)sample.sample_sequence);
            }
        }
        s_last_published_imu_sequence = sample.sample_sequence;
        s_has_published_imu_sample = true;
    } else {
        ESP_LOGW(TAG, "IMU publish failed: rc=%d", (int)rc);
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
        ESP_LOGW(TAG, "Agent time synchronization failed: rc=%d", (int)rc);
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

static void micro_ros_task(void *arg)
{
    (void)arg;

    s_command_queue = xQueueCreate(1, sizeof(robot_command_cache_t));
    if (s_command_queue == NULL) {
        ESP_LOGE(TAG, "failed to create latest-command queue");
        vTaskDelete(NULL);
        return;
    }

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();

    /* A1: configure the ROS domain and Agent UDP endpoint. */
    RCL_CHECK_OR_DELETE_TASK(rcl_init_options_init(&init_options, allocator));
    RCL_CHECK_OR_DELETE_TASK(rcl_init_options_set_domain_id(
        &init_options, CONFIG_ROBOT_MICROROS_DOMAIN_ID));

    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    RCL_CHECK_OR_DELETE_TASK(rmw_uros_options_set_udp_address(
        CONFIG_ROBOT_MICROROS_AGENT_IP,
        CONFIG_ROBOT_MICROROS_AGENT_PORT,
        rmw_options));

    /* A2: wait until the configured Agent answers an XRCE ping. */
    while (rmw_uros_ping_agent_options(1000, 1, rmw_options) != RMW_RET_OK) {
        ESP_LOGW(TAG, "waiting for Agent at %s:%s",
                 CONFIG_ROBOT_MICROROS_AGENT_IP,
                 CONFIG_ROBOT_MICROROS_AGENT_PORT);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* A3: create the Client Session and ROS 2 entities through the Agent. */
    RCL_CHECK_OR_DELETE_TASK(rclc_support_init_with_options(
        &support, 0, NULL, &init_options, &allocator));

    rcl_node_t node = rcl_get_zero_initialized_node();
    RCL_CHECK_OR_DELETE_TASK(rclc_node_init_default(
        &node, "esp32s3_robot", "", &support));

    s_heartbeat_publisher = rcl_get_zero_initialized_publisher();
    RCL_CHECK_OR_DELETE_TASK(rclc_publisher_init_default(
        &s_heartbeat_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/robot/heartbeat"));

    s_imu_publisher = rcl_get_zero_initialized_publisher();
    RCL_CHECK_OR_DELETE_TASK(rclc_publisher_init_default(
        &s_imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu"));

    /* A4: create the ROS 2 -> ESP32 command subscription. */
    s_command_subscription = rcl_get_zero_initialized_subscription();
    RCL_CHECK_OR_DELETE_TASK(rclc_subscription_init_default(
        &s_command_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/robot/command"));

    s_cmd_vel_subscription = rcl_get_zero_initialized_subscription();
    RCL_CHECK_OR_DELETE_TASK(rclc_subscription_init_default(
        &s_cmd_vel_subscription,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    /* A5: let one Executor serve both directions of the communication path. */
    rcl_timer_t heartbeat_timer = rcl_get_zero_initialized_timer();
    RCL_CHECK_OR_DELETE_TASK(rclc_timer_init_default(
        &heartbeat_timer,
        &support,
        RCL_MS_TO_NS(CONFIG_ROBOT_MICROROS_PUBLISH_PERIOD_MS),
        heartbeat_timer_callback));

    rcl_timer_t imu_timer = rcl_get_zero_initialized_timer();
    RCL_CHECK_OR_DELETE_TASK(rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(CONFIG_ROBOT_MICROROS_IMU_PUBLISH_PERIOD_MS),
        imu_timer_callback));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCL_CHECK_OR_DELETE_TASK(rclc_executor_init(
        &executor, &support.context, 4, &allocator));
    RCL_CHECK_OR_DELETE_TASK(
        rclc_executor_add_timer(&executor, &heartbeat_timer));
    RCL_CHECK_OR_DELETE_TASK(rclc_executor_add_timer(&executor, &imu_timer));
    RCL_CHECK_OR_DELETE_TASK(rclc_executor_add_subscription(
        &executor,
        &s_command_subscription,
        &s_command_msg,
        command_subscription_callback,
        ON_NEW_DATA));
    RCL_CHECK_OR_DELETE_TASK(rclc_executor_add_subscription(
        &executor,
        &s_cmd_vel_subscription,
        &s_cmd_vel_msg,
        cmd_vel_subscription_callback,
        ON_NEW_DATA));

    s_heartbeat_msg.data = 0;
    s_command_msg.data = 0;
    if (!geometry_msgs__msg__Twist__init(&s_cmd_vel_msg)) {
        ESP_LOGE(TAG, "failed to initialize geometry_msgs/Twist");
        vTaskDelete(NULL);
        return;
    }
    if (!sensor_msgs__msg__Imu__init(&s_imu_msg)) {
        ESP_LOGE(TAG, "failed to initialize sensor_msgs/Imu");
        vTaskDelete(NULL);
        return;
    }
    s_imu_msg.header.frame_id = micro_ros_string_utilities_set(
        s_imu_msg.header.frame_id, "imu_frame");
    if (s_imu_msg.header.frame_id.data == NULL) {
        ESP_LOGE(TAG, "failed to allocate IMU frame_id");
        vTaskDelete(NULL);
        return;
    }
    s_imu_msg.orientation.x = 0.0;
    s_imu_msg.orientation.y = 0.0;
    s_imu_msg.orientation.z = 0.0;
    s_imu_msg.orientation.w = 1.0;
    s_imu_msg.orientation_covariance[0] = -1.0;

    refresh_time_sync_if_due();
    ESP_LOGI(TAG,
             "ROS ready: pub=/robot/heartbeat,/imu sub=/robot/command,/cmd_vel; "
             "IMU backend=%s",
             robot_imu_backend_name());

    while (true) {
        const rcl_ret_t rc = rclc_executor_spin_some(
            &executor, RCL_MS_TO_NS(5));
        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
            ESP_LOGW(TAG, "executor spin failed: rc=%d", (int)rc);
        }
        check_command_timeout();
        refresh_time_sync_if_due();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
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
