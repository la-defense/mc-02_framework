# MOTOR-01 + MOTOR-02：修复 LK 与 HT04 闭环反馈来源

本文记录实现和本机软件验收；任务状态只由[TODO](../../../.Doc/TODO.md)维护。本轮没有连接开发板或电机。

## 基线与改动

- 基线：`de64f91ac7ffe6ff2f4dbb29bbebd8cb8717abbd`；共用分支：`fix/motor-feedback-loops`。
- LK9025 角度环使用 `total_angle`；速度环使用独立的 `speed_PID`，并由 `speed_feedback_source` 选择内部速度或外部速度。
- HT04 速度环也改由 `speed_feedback_source` 选择反馈。角度和速度配置互不影响；角度、速度输入分别从 rad、rad/s 换成 PID 使用的 deg、deg/s。
- 两驱动保存初始化配置中的外部反馈和前馈指针，只要求实际参与分支的字段有效。启用时在初始化检查来源与指针，任务每轮再次检查来源、指针、有限值和前馈相加结果。
- 发生输入、PID、单位换算或累加错误时，停止当前串级计算、清除该实例三环历史并输出零。HT04 将零力矩写入既有协议字段；LK9025 清对应电机槽位。修正输入后下一周期可恢复。
- 公式、反馈时间来源、输出编码和前馈加入位置保持既有实现。LK 跨圈参考可信度、HT04 底层注册回退、跨任务共享数据一致性和物理单位标定仍由原 TODO 跟踪。

## 固定数值与调用方测试

LK 主机测试链接生产 `LK9025.c`、PID、Daemon、BSP CAN 注册和发送实现，仅替换 HAL、时间与日志。固定输入角度参考30、内部／外部角度10／20、内部／外部速度2／4、当前反馈100，三环比例系数2／3／1；四种反馈组合输出14、−46、8、−52，并核对 CAN ID `0x280` 与四个小端有符号槽位。另覆盖独立前馈、空指针和非法来源初始化拒绝、NaN、前馈溢出、PID 未初始化、失败清零和恢复。

HT04 测试通过任务创建替身捕获生产任务入口，再用有限轮次调用真实控制循环。它按独立双精度计算比较弧度换算和 12 位力矩字段，覆盖四种内外反馈组合、前馈单独及组合、必需指针缺失、非法来源、角度／速度 NaN、速度换算溢出和恢复；失败后检查 PID 历史已清零。该测试不调用真实 CAN、GPIO、等待或电机设备。

## 本机验证记录（2026-10-20）

| 验证 | 结果 |
|---|---|
| WSL Ubuntu 22.04／GCC 11.4 Debug 全量 CTest | 215/215 通过 |
| WSL Ubuntu 22.04／GCC 11.4 Release 全量 CTest | 215/215 通过；新增 `TEST_CHECK` 在 Release 生效 |
| Windows MinGW 11.5 Debug 支持子集 | 151/151 通过 |
| Windows MinGW 11.5 Release 支持子集 | 151/151 通过 |
| Windows 已有排除项 | `production_init_resource_failures` 使用 GNU `--wrap` 专属符号，MinGW 链接时缺少 `__real_malloc/calloc/free`；沿用支持子集排除，不影响新增电机测试 |
| WSL ASan／UBSan | LK 6项与 HT04 11项，共17/17通过；启用泄漏检查及遇错退出 |
| 固件矩阵 | `bench_safe`、`robot`、`rcdiag` × Debug／RelWithDebInfo 六种禁热配置全部构建通过 |
| ELF profile 隔离 | 六种产物均通过 `check_firmware_profile.py`；robot 保留电机接口，bench_safe 未链接执行器，rcdiag 身份检查通过 |
| 非法固件配置 | `bench_safe + heater ON`、`robot + remote diagnostics ON` 均被 CMake 拒绝 |
| 编译设置 | 六种 Ninja 命令均使用 Cortex-M7、`fpv5-d16`、hard-float 和 `-Wformat=2`；未启用 `-ffast-math`／`-Ofast` |
| 旧 Makefile | `make -n -B build/HT04.o` 与 `build/LK9025.o` 均保留自有代码格式检查选项；生产源码清单已包含两驱动，本轮未修改清单 |
| CMake 增量检查 | WSL 原生文件系统副本中，源码增加／删除／移动、嵌套目录、只移动头文件目录和排除模板场景均与干净配置一致；基线162条源码路径 |
| 代码差异检查 | `git diff --check` 通过；本地教学链接和 TODO 格式在提交前复查 |

### 固件容量

基线和本轮均使用 WSL ARM GCC 10.3.1、相同配置选项。Flash 按 `text + data`，静态 RAM 按 `data + bss` 统计。LK 控制代码在 robot 配置中实际保留；小型其他配置差异仅记录为链接区数值，不据此声称节省或性能变化。

| 配置 | 基线 Flash | 本轮 Flash | 变化 | 基线 RAM | 本轮 RAM | 变化 |
|---|---:|---:|---:|---:|---:|---:|
| bench_safe Debug | 185968 B | 185996 B | +28 B | 104332 B | 104332 B | 0 B |
| bench_safe RelWithDebInfo | 180260 B | 180288 B | +28 B | 104284 B | 104284 B | 0 B |
| robot Debug | 215500 B | 215992 B | +492 B | 113232 B | 113232 B | 0 B |
| robot RelWithDebInfo | 207896 B | 208332 B | +436 B | 113176 B | 113176 B | 0 B |
| rcdiag Debug | 188156 B | 188184 B | +28 B | 107500 B | 107500 B | 0 B |
| rcdiag RelWithDebInfo | 182348 B | 182376 B | +28 B | 107452 B | 107452 B | 0 B |

原始构建日志位于忽略目录 `build/motor-feedback/`；CMake 增量回归逐步日志保存在 WSL `/home/mc02/build/motor-feedback/cmake-incremental/`。本轮仅证明主机算法、编码和固件构建；不宣称验证真实电机响应、CAN 波形、物理量纲或控制整定。

## 教学入口

- [学习笔记23：Ozone 辅助 PID 调参](../../学习笔记/23-Ozone辅助PID调参.md)
- [LK9025 驱动说明](../../../modules/motor/LKmotor/LK_motor.md)
- [HT04 驱动说明](../../../modules/motor/HTmotor/HT04.md)
- [LK 生产闭环测试](../../../tests/host/lk_motor_bsp_can_test.c)
- [HT04 生产任务测试](../../../tests/host/ht_control_test.c)
