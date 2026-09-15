# ESP32-S3 FreeRTOS micro-ROS Robot

ESP32-S3 firmware for a ROS 2 / micro-ROS differential-drive robot control chain.
The repository focuses on MCU-side real-time tasks, command handling, IMU data
flow, communication recovery, and safe control interfaces.

## Current scope

| Area | Implementation | Current status |
| --- | --- | --- |
| Communication | Wi-Fi/UDP micro-ROS, Agent session probing and recovery, `/robot/heartbeat`, `/robot/command`, rclc Executor | Runtime-tested communication path; credentials and Agent address remain local configuration |
| Drive control | Command validation, differential-drive target calculation, latest-command snapshot and timeout handling | Software dry-run; PWM, encoder feedback, PID, `/odom_raw` and physical stopping remain separate work |
| Chassis diagnostics | 10 ms task, MCPWM/PCNT interfaces, PI calculation, hard limits, default disable and `stop_all()` | Available for controlled bring-up; motor power remains disabled by default |
| IMU | ICM42670P I2C backend, WHO_AM_I/configuration checks, raw-to-SI conversion, freshness handling and `/imu` packaging | I2C batch check is recorded; formal image keeps the physical backend disabled until integration checks are complete |

## Data paths

```text
/cmd_vel -> validation -> differential-drive targets -> latest command snapshot
         -> 10 ms control task -> guarded motor interface

ICM42670P -> I2C0 -> raw samples -> SI-unit sample -> freshness check -> /imu
ROS 2 -> micro-ROS Agent -> rclc callback -> command snapshot
```

The command callback only updates the latest snapshot and timestamp. The local
control task owns timeout handling and the safe-stop path. IMU orientation is not
estimated in firmware; consumers should not treat placeholder covariance as an
orientation solution.

## Directory guide

- `components/robot_microros`: ROS 2 / micro-ROS node, publishers, subscribers and Executor
- `components/robot_drive`: command validation, differential targets and timeout logic
- `components/robot_chassis_diag`: 10 ms chassis diagnostics, MCPWM, PCNT, PI and safe state machine
- `components/robot_encoder_probe`: read-only four-channel PCNT diagnostics
- `components/robot_imu`: IMU policy, ICM42670P backend and conversion core
- `docs/verification-2026-09-12.md`: latest hardware-module check summary and known limits
- `tools/README_chassis_diag_test.md`: safe chassis diagnostic procedure

## Build and checks

```powershell
pwsh .\tools\test-host-core.ps1
pwsh .\tools\build-firmware.ps1
```

Configure Wi-Fi credentials and the micro-ROS Agent address locally. Generated
`sdkconfig*` files and build output are intentionally not committed. The checked-in
defaults contain no credentials or private network address.

## Known limits

The current public scope does not include validated motor motion, closed-loop PID,
physical `/odom_raw`, LiDAR, TF/EKF/SLAM/AMCL/Nav2, or whole-vehicle navigation.
Passing host checks or building the firmware does not replace those integration
checks.
