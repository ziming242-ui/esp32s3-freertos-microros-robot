# 01/02/03 集成验证记录

日期：2026-09-06。路线：`esp32s3-differential-drive`。
范围：自有固件首次纳入 Git；01 通信接口、02 dry-run、03 IMU 软件数据管线。

## 运行前预测

用户确认：默认 IMU disabled 时，主机测试和固件编译预期通过；
`/cmd_vel` 只生成 dry-run 四轮目标，`/imu` 不发布样本。
后两项需要开发板/Agent 运行才能获得实测证据，源码检查不能替代实测。

## 当前结果

| 验证项 | 命令/方法 | 实际结果 | 证据与边界 |
|---|---|---|---|
| 02 纯 C 核心 | `pwsh tools/test-host-core.ps1` | 62 checks, 0 failures | `[PC模拟]`：直行、转向、等比例限速、NaN、无效配置；未执行 FreeRTOS 超时任务 |
| 03 数据检查策略 | 同上 | `robot_imu_policy: PASS` | `[PC模拟]`：无样本、新鲜、100 ms 边界、过期、时钟倒退、NaN；未执行真实采集 |
| Synthetic 条件分支 | 从 compile_commands.json 复用 Xtensa 编译选项，额外定义 synthetic/10 ms，执行 `-fsyntax-only` | PASS | 单文件交叉编译语法检查；不是 synthetic 整固件链接或运行 |
| 固件默认配置 | `pwsh tools/build-firmware.ps1` | 正在验证 | 目标 ESP32-S3，IMU disabled；结果待本轮更新 |
| 补丁已应用检查 | `pwsh tools/setup-dependencies.ps1` | PASS，识别已应用补丁 | 检查本地依赖状态；干净克隆应用由 CI 复验 |

构建使用本机 ESP-IDF 6.0.1、WSL Ubuntu-22.04 和既有 micro-ROS 库缓存。
micro-ROS 子模块固定为 `4ddd8c26e721662319ed8af981cb7cdc9ae05382`。
这不是从零构建全部依赖的证据。

## 代码行为与待验证项

- 01 保留 `/robot/heartbeat` 与 `/robot/command`。后者为通信测试输入；
  超时只清空测试缓存，日志不代表物理停车。历史双向通信开发板实测不能
  自动升级为本次新增固件的实测。
- 02 接收 `/cmd_vel` 中的 `linear.x` 与 `angular.z`，检查有限值和不支持的轴，
  保存四轮目标及 dry-run 状态；500 ms 过期处理位于独立 FreeRTOS 任务。
  PWM、编码器、PID、实体电机和 `/odom_raw` ROS 2 发布均未接入。
- 03 默认禁用，synthetic 必须主动配置。数据先统一为 m/s² 与 rad/s，
  原始量换算、校准和实物坐标映射属于待接入的传感器后端。
  新鲜度使用采样时间；发布成功后记录序号，重复样本不重发。
  无姿态估计时 `orientation_covariance[0] = -1`。
- 新增 `/cmd_vel` 与 `/imu` 没有本次开发板、模块或整车实测证据。
  ICM42670P 实物驱动、基础校准和 I2C 仍待接入，不能写成 IMU 已跑通。
- Agent 断线后的 Session 重建尚未实现；同步时间可能阻塞 executor，需后续
  测量频率与抖动。序号跳变只观察生产者到发布者之间的跳样，不证明 DDS 丢包。
- 学习验收独立于 CI：02 正式测试与综合题未作答，03 也没有完成整模块验收。

厂家资料 `E:\MicroROS Robot` 不纳入仓库，保持只读。
