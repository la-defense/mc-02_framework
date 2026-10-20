# MOTOR-03：DM 力矩编码与初始化生命周期

本票据记录代码范围和验证证据；唯一任务状态仍由 [.Doc/TODO.md](../../../.Doc/TODO.md) 管理。本轮只做本机软件验证，没有连接开发板或电机。

## 基线与实现

- 基线：`64b9b9213711015e11445393f14a2cb3f6f44773`
- 分支：`fix/dm-torque-lifecycle`
- DM 仍只接受直接力矩模式。注册接口使用只读配置，最多4个实例；检查支持的三条 FDCAN 总线、标准 ID、方向、模式及同总线重复 ID。候选对象准备完成 Daemon 和 CAN 登记后才发布，失败时回收已申请资源。
- 注册成功后保持停止，不发送自动使能、置零或控制帧。控制任务只由 `DMMotorControlInit()` 显式创建；创建失败会回收已建任务，若无法确认回收则隔离模块。
- 使能和编码器置零使用单命令槽，由每实例任务各尝试发送一次。命令查询中的 `ACCEPTED` 仅表示本机 CAN 接受；使能接受后清除旧参考。Stop 清除参考并取消尚未提交的命令，已开始提交的帧不能撤回。
- 控制任务每轮读取一次参考和方向，先反向、再限幅到 `−18～18`，再量化到12位字段。停止控制帧的零力矩字段为 `0x7FF`，继续保留完整8字节帧。该协议数值没有据此解释成物理力矩单位。

## 主机验证

`tests/host/dm_motor_test.c` 链接生产 DM、FDCAN 注册和 Daemon，替换 HAL、RTOS、时间与日志。各场景独立进程、检查在 Release 下始终生效，超时设为10秒。

| 检查 | 结果 |
|---|---|
| WSL Ubuntu 22.04 / GCC 11.4 全量 Debug | 228/228 通过 |
| WSL Ubuntu 22.04 / GCC 11.4 全量 Release | 228/228 通过；新增 DM 检查使用 `TEST_CHECK` |
| DM 专项 Debug / Release | 各12/12 通过；包含查询不推进命令的检查 |
| DM ASan + UBSan + float-cast-overflow | 12/12 通过，启用泄漏检查 |
| 编码固定值 | `+9 → 0xBFF`、`−9 → 0x3FF`、`−18 → 0x000`、`+18 → 0xFFF`；零力矩完整帧 `7F FF 7F F0 00 00 07 FF` |
| 资源与任务失败 | 分配、Daemon/CAN 登记、任务创建及任务回收失败回退场景通过；成功后运行期无额外分配 |
| 编译告警 | DM 测试目标使用 `-Werror`；仅对生产 `bsp_can.c` 中已有的无符号数 `>= 0` 比较局部屏蔽 `-Wtype-limits` |
| CMake 增量发现 | WSL 原生文件系统隔离副本中，源码新增／删除／移动、头目录移动和模板排除检查通过；记录位于 `/home/mc02/build/dm-torque-cmake-regeneration/` |
| 非法固件配置 | `bench_safe + heater`、`robot + remote diagnostics`、`bench_safe + referee UI` 均被 CMake 拒绝 |

全量 Debug／Release 套件先完成；随后只在 DM 专项增加查询不推进命令、置零后不使能的断言，并在 Debug、Release 和消毒构建中重跑全部12个 DM 场景，均通过。其他主机测试源码没有变化。

## 固件矩阵与资源

工具链为 ARM GNU GCC 10.3.1、Cortex-M7、`fpv5-d16`、hard-float。六种禁热固件均构建并通过 `check_firmware_profile.py`；没有启用 fast-math。

| 配置 | Flash（text + data） | 静态 RAM（data + bss） |
|---|---:|---:|
| bench_safe Debug | 186,212 B | 104,396 B |
| bench_safe RelWithDebInfo | 180,496 B | 104,348 B |
| robot Debug | 216,208 B | 113,296 B |
| robot RelWithDebInfo | 208,540 B | 113,240 B |
| rcdiag Debug | 188,392 B | 107,564 B |
| rcdiag RelWithDebInfo | 182,584 B | 107,516 B |

以相同 GCC 对基线 `64b9b92` 和本分支构建 `robot Debug`，基线使用 `-ffile-prefix-map` 将隔离路径映射到相同源码路径后，Flash 和静态 RAM 均相同（216,208 B／113,296 B）。六种 map 中 DM 对象的 `.text`、`.data`、`.bss` 均未进入输出镜像：当前正式固件没有 DM 调用点，链接器将未引用节丢弃，因此本轮 DM 代码对这些固件的实际占用变化为0。该结果不表示 DM 可在真实电机上安全运行。

旧 Makefile 的 DM 单目标编译通过。首次尝试只因忽略目录中尚无嵌套输出目录而失败；创建构建目录后，`build/modules/motor/DMmotor/dmmotor.o` 编译成功。生产源码清单没有变化，也未执行 BUILD-02。

## 未完成项与边界

当前 Windows 环境未找到 MinGW、GCC、Clang 或 MSVC；现有 Windows CMake 缓存也将编译器记为 `NOTFOUND`，因此 Windows 支持子集尚未运行。TODO 中的 MOTOR-03 保持开放，待有可用 Windows 编译工具链后补跑该子集。该限制不影响已完成的 WSL 与 ARM 构建结果。

没有验证 CAN 波形、电机执行结果、型号／固件兼容性或物理力矩单位。完整 MIT 模式及力矩量纲仍由 MC02-0073 跟踪；其他未启用驱动初始化仍由 MC02-0051 跟踪。

## 教学入口

- [学习笔记23：电机参考值、DM 力矩处理和启停顺序](../../学习笔记/23-Ozone辅助PID调参.md#dm-只在显式使能后发送力矩motor-03)
- [DM 主机回归](../../../tests/host/dm_motor_test.c)
- [DM 公开接口](../../../modules/motor/DMmotor/dmmotor.h)
