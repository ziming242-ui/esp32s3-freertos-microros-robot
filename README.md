# ESP32-S3 FreeRTOS micro-ROS Robot

ESP32-S3 底盘固件与传感器验证工程，面向“ROS 2 上位机 + MCU 实时任务”的实验室配送机器人控制链。

## 当前公开范围

| 模块 | 公开内容 | 证据边界 |
| --- | --- | --- |
| micro-ROS 通信 | Wi-Fi/UDP、Agent、`/robot/heartbeat`、`/robot/command`、rclc Executor | 通信链与命令接收可复现；不把 ping 当作 Session 或实体证据 |
| 底盘控制 | `robot_drive` 最新命令快照、超时保护；`robot_chassis_diag` 的 10 ms 控制任务、MCPWM/PCNT、硬限幅、默认禁能和 `stop_all()` | 四路 PCNT 手转属于 `[模块实测]`；不宣称电机闭环、PID、里程计或整车运动已通过 |
| ICM42670P | I2C0 GPIO39/40、地址 `0x68`、WHO_AM_I、配置回读、DRDY 轮询、raw→SI 换算和 stale 测试入口 | 真实 I2C 与 16/16 六轴批量采样属于 `[模块实测]`；安装轴映射、正式 ROS 频率和断开自动恢复仍单独标注 |
| 主机测试 | drive / IMU 纯 C 核心测试和 PowerShell 构建入口 | `[PC模拟]` 或构建成功不等于整车硬件证明 |

## 关键数据链

```text
ROS 2 → micro-ROS Agent → rclc callback → latest command queue
      → 10 ms chassis task → guarded MCPWM output → PCNT feedback

ICM42670P → I2C0 → raw int16 → SI-unit sample → freshness check → /imu
```

订阅回调只更新命令快照和时间戳；本地控制任务独立执行，命令超时后进入安全停止路径。编码器诊断固件默认不启动电机动力，便于先验证轮序、正反向计数和静止稳定性。

## 目录入口

- `components/robot_microros`：ROS 2 / micro-ROS 节点、发布、订阅与 Executor
- `components/robot_drive`：命令校验、差速目标和超时 dry-run
- `components/robot_chassis_diag`：10 ms 底盘诊断、MCPWM、PCNT、PI 和安全状态机
- `components/robot_encoder_probe`：只读四路 PCNT 诊断
- `components/robot_imu`：IMU 策略、ICM42670P 后端和纯 C 换算核心
- `docs/verification-2026-09-12.md`：最新验证摘要与证据边界
- `tools/README_chassis_diag_test.md`：底盘诊断命令与安全前置条件

## 构建与测试

```powershell
pwsh .\tools\test-host-core.ps1
pwsh .\tools\build-firmware.ps1
```

首次使用前配置本地 Wi-Fi、Agent 地址和端口；凭据与生成的 `sdkconfig` 不提交。完整 micro-ROS 依赖通过仓库脚本准备。

## 明确不宣称

本仓库不把 PC 模拟、编码器手转或 IMU 模块采样包装成完整底盘闭环；不宣称激光雷达、EKF、TF、SLAM、AMCL、Nav2 或整车导航已经完成。
