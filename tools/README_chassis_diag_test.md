# 底盘诊断串口测试脚本

`run-chassis-diag-test.py` 用于给当前 `CHASSIS_DIAG` 固件发送有界串口命令并保存原始日志。

- 默认仅打印计划，不打开串口；只有显式增加 `--execute` 才会执行。
- 允许单轮 M1～M4 或四轮 M0、非零目标 `-20..20 count/10 ms`、持续时间 `0.6..5.0 s`；四轮执行还必须显式增加 `--all-wheels-confirmed`。
- 每 100 ms 刷新一次 `RUN`，避免 500 ms 命令超时干扰预定测试。
- 无论正常结束还是异常退出，都尝试发送 `STOP -> ZERO -> CLEAR -> STATUS`。
- 判定使用本次测试前后的编码器累计计数差，避免历史计数造成假通过。
- `--expect motion` 要求所选编码器计数差非零且无停转故障；`--expect stall` 要求捕获停转故障且所选编码器计数差为 0。
- 可选 `--kp` 与 `--ki` 会在 `DISARMED` 状态更新 PI 参数；固件会强制再次执行 `ZERO -> ARM`。
- 固件 `STATUS` 同时输出电池 ADC、实际电池电压、PI 参数、目标/输出限幅和候选里程计。

示例（不执行）：

```powershell
python .\tools\run-chassis-diag-test.py --wheel 1 --target 2 --duration 1.5 --expect motion
```

实体动力测试前仍需人工确认车轮架空、周围无障碍且能立即断电；脚本保护不能替代物理安全措施。
