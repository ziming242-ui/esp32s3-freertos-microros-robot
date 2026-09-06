# ESP32-S3 FreeRTOS micro-ROS Robot

This repository is the firmware baseline for the ESP32-S3 differential-drive
robot learning route. It keeps software evidence separate from physical robot
evidence.

## Implemented scope

| Module | Firmware path | Current boundary |
|---|---|---|
| 01 communication | `components/robot_wifi`, `components/robot_microros` | Wi-Fi/UDP micro-ROS, `/robot/heartbeat`, and `/robot/command` are retained. Runtime Session reconstruction is not implemented. |
| 02 drive | `components/robot_drive` | `/cmd_vel` becomes a four-wheel target snapshot with validation, proportional limiting, and a 500 ms timeout. Dry-run only: no PWM, encoder, PID, or physical stop. |
| 03 IMU | `components/robot_imu` plus `/imu` publisher | Coherent SI-unit samples, acquisition timestamp, sequence, stale-data rejection, and an optional synthetic source. No physical IMU driver is selected yet. |

The default IMU backend is disabled. Synthetic samples must be enabled
explicitly and are never sensor evidence.
The `/imu` publisher entity is created after Agent connection even when the
backend is disabled, but no samples are published in that configuration.

The future physical IMU backend owns raw-count conversion and physical-sensor
axis mapping. `robot_imu_submit_si_sample()` accepts values already expressed
in SI units and in the right-handed `imu_frame`; the ROS layer only packages
them and does not convert axes merely by changing `frame_id`.

## Data paths

```text
/cmd_vel -> validate -> differential-drive targets -> latest dry-run snapshot

IMU producer -> coherent latest sample -> freshness check -> sensor_msgs/Imu
             -> sample-time timestamp -> micro-ROS Agent -> ROS 2 /imu
```

`sensor_msgs/Imu.orientation` is not estimated in this firmware. The message
sets `orientation_covariance[0] = -1`, which tells consumers to ignore the
placeholder orientation.

Gyroscope and accelerometer covariance arrays remain zero (unknown). Samples
include gravity. Duplicate sample sequences are not republished after a
successful publish. A sequence jump between publications can result from the
latest-value queue or deliberate downsampling; it does not measure ROS network
packet loss. The current producer API requires one producer task.

## Checkout and dependencies

```powershell
git clone --recurse-submodules https://github.com/ziming242-ui/esp32s3-freertos-microros-robot.git
cd esp32s3-freertos-microros-robot
pwsh .\tools\setup-dependencies.ps1
```

`setup-dependencies.ps1` applies one small compatibility patch to the pinned
micro-ROS ESP-IDF component. Generated micro-ROS headers and libraries remain
inside that dependency and are not committed here.
The setup script intentionally leaves the submodule with a local `libmicroros.mk`
modification matching the checked-in patch. This is expected after setup.

## Local configuration

Wi-Fi credentials and generated `sdkconfig*` files are intentionally local.
Configure these values with ESP-IDF menuconfig:

- `Robot Network Configuration`: SSID and password.
- `Robot micro-ROS Configuration`: Agent address, port, timing, and task size.
- `Robot drive dry-run configuration`: provisional geometry, target limit, and timeout.
- `Robot IMU Configuration`: disabled or synthetic backend and stale timeout.

The checked-in `sdkconfig.defaults` contains only portable defaults.
The build helper explicitly selects `esp32s3`. A fresh configuration has empty
Wi-Fi credentials and cannot connect until configured. The WSL helper currently
expects Ubuntu-22.04 and the ESP-IDF installation managed by local PlatformIO;
it does not install that environment on a fresh computer. `platformio.ini`
retains the earlier board configuration; this integration is verified with the
WSL helper, not with `pio run`.

## Verification

Run the platform-independent Module 02/03 core tests:

```powershell
pwsh .\tools\test-host-core.ps1
```

Build the ESP32-S3 firmware through the existing WSL ESP-IDF environment:

```powershell
pwsh .\tools\build-firmware.ps1
```

A passing host test or firmware build is software evidence only. It does not
prove motor movement, encoder feedback, physical stopping, IMU communication,
or valid sensor measurements.

See [verification evidence](docs/verification-2026-09-06.md) for results and
remaining checks, and [CONTRIBUTING.md](CONTRIBUTING.md) for the branch/PR flow.
02 learner acceptance (10 formal questions plus the comprehensive question)
remains pending regardless of automated test results.

Known integration limits: Agent Session reconstruction is not implemented;
network/Agent recovery requires a separate runtime test and recovery change.
Agent time synchronization currently runs on the executor task and may block
callback service for up to one second. The independent drive task still applies
its local dry-run timeout. Current micro-ROS resource limits are 2 publishers
and 2 subscriptions, both fully allocated; adding another topic requires a
resource-budget change. LiDAR, millimeter-wave radar, fusion and navigation are
outside this implementation.
