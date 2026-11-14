# MC02-0056：三模式许可策略软件证据

此记录是近期软件切片，不维护任务状态。

- WSL Debug／Release 的 `production_robot_control_policy` 各1/1通过；Release使用显式检查。
- 同实现和测试通过 GCC ASan／UBSan；生产策略以 Cortex-M7 硬浮点参数独立编译通过，无HAL／RTOS头依赖。
- 覆盖纯手动拒绝视觉接管、辅助人工击发、自主无人工来源、瞄准／导航分别过期、本地策略接管、裁判阶段过期、立即禁止／核心异常覆盖三模式、局部发射故障、云台反馈失效禁火、来源时间回绕、调试模式限制及错误保留输出。

主机构建位于 `/home/mc02/build/sentinel-debug`、`sentinel-release`；独立sanitizer程序和ARM对象位于 `/home/mc02/build/sentinel-policy-san`、`sentinel-policy-arm.o`。未接入安全管理和命令发布路径，未连接硬件，不能据此关闭整个MC02-0056。
