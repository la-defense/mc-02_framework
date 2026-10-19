# MOTOR-04 + MOTOR-05：LK 注册回退与按总线分组发送

## 基线与范围

- 基线：`d38f5369904632e0e989da6e6ca9252be1cc22bb`；分支：`fix/lk-registration-routing`。
- 每条 H723 CAN 总线最多登记4个 LK9025，总计最多12个。`can_init_config.tx_id` 在初始化配置中表示电机编号1～4，不是最终发送 ID。
- 本任务只处理注册、资源回退和帧路由。角度／速度控制环的既有反馈及 PID 选择问题由 MOTOR-01 跟踪；方向扩展、原生闭环和设备参数管理也不在本轮范围内。

## 实现

- `LKMotorInit(const Motor_Init_Config_s *config)` 在资源申请前检查电机类型、支持的 CAN 句柄、编号、同总线重复项、总线容量和所有已启用 PID 环；不会改写调用者配置。
- 每个 LK 对象仅分配一次。资源按 LK 对象、Daemon、CAN 的顺序准备；Daemon 失败释放候选对象，CAN 失败先注销 Daemon 再释放候选对象。CAN 成功后才发布实例和总线分组，不再进行可能失败的资源申请。
- 实例编号和物理总线序号独立保存。CAN 接收 ID 为 `0x140 + 编号`，每个实例的真实发送 ID 与 HAL 帧头均为 `0x280`。
- 控制循环先将每条总线的候选帧清零，再把编号1～4映射到固定两字节槽并显式按小端写入。停止、非有限结果、超出有符号16位表示范围或方向处理后越界均输出零；数值失败时清 PID 历史。无法表示时不做电流饱和猜测。
- 各已登记总线独立提交一次。某总线提交失败不影响其他总线，失败帧不缓存，下一周期重新计算。`CANTransmit()` 的本机结果仍由既有 CAN 统计记录。

## 验证记录

| 验证 | 结果 |
|---|---|
| WSL 主机 Debug 全量 CTest | 169/169 通过 |
| WSL 主机 Release 全量 CTest | 169/169 通过 |
| Windows Debug 支持子集 | 105/105 通过 |
| Windows Release 支持子集 | 105/105 通过 |
| WSL ASan／UBSan | 注册、Daemon 分配失败、CAN 回退、稀疏路由及生产 BSP CAN 回退／发送共6个场景通过；关闭 LeakSanitizer 是因为成功注册对象按设计保留到进程结束，失败回退另有分配／释放计数断言 |
| 固件矩阵 | bench_safe、robot、rcdiag × Debug、RelWithDebInfo 六种禁热配置构建通过；六个 ELF profile 隔离检查通过 |
| 非法配置 | bench_safe 启用加热、robot 启用遥控诊断均被 CMake 拒绝 |
| CMake 增量发现 | 在隔离 Linux 副本验证新增／删除／移动源文件、多层目录、只移动头文件目录及排除模板；与全新配置的源文件和包含目录集合一致 |

六种固件均使用原有 Cortex-M7、`fpv5-d16`、hard-float 配置；检查六份 Ninja 命令未启用 `-Ofast` 或 `-ffast-math`。

Windows 的 `production_init_resource_failures` 未纳入支持子集：MinGW 链接该既有测试时找不到 `__real_malloc`、`__real_calloc`、`__real_free`。这不影响本轮新增 LK 测试；其余105项在两个 Windows 构建类型中通过。

Flash 按 `text + data`、静态 RAM 按 `data + bss` 统计。基线与本轮固件均使用 WSL GCC 10.3.1、相同源码路径和同一构建选项；当前工程只在 robot 配置保留 LK 控制模块。尺寸如下：

| 配置 | 基线 Flash | 本轮 Flash | 变化 | 基线 RAM | 本轮 RAM | 变化 |
|---|---:|---:|---:|---:|---:|---:|
| bench_safe Debug | 174596 B | 174596 B | 0 B | 104316 B | 104316 B | 0 B |
| bench_safe RelWithDebInfo | 172288 B | 172288 B | 0 B | 104268 B | 104268 B | 0 B |
| robot Debug | 203928 B | 204120 B | +192 B | 113176 B | 113216 B | +40 B |
| robot RelWithDebInfo | 199796 B | 199916 B | +120 B | 113120 B | 113160 B | +40 B |
| rcdiag Debug | 176776 B | 176776 B | 0 B | 107484 B | 107484 B | 0 B |
| rcdiag RelWithDebInfo | 174376 B | 174376 B | 0 B | 107436 B | 107436 B | 0 B |

主机回归在 [LK 注册与路由测试](../../../tests/host/lk_motor_registration_test.c) 和 [生产 BSP CAN 集成测试](../../../tests/host/lk_motor_bsp_can_test.c) 中；测试只替换必要的 CAN／HAL、时间和日志入口，不连接电机。固件基线与本轮尺寸构建、CMake 增量日志保存在本机 WSL 的 `build/lk-registration-*` 目录。

## 遗留边界

- 没有连接开发板或真实 LK 电机；本轮不验证 CAN 电气波形、总线 ACK、电机响应或物理吞吐。
- 成功登记的实例没有运行期注销接口，注册必须在接收派发、Daemon 与控制任务启动前串行完成。
- MOTOR-01 仍记录的角度环／速度环路径问题没有改变；新增零输出保护只处理非有限值和无法编码为有符号16位的数值结果。

教学入口：[学习笔记03：LK 电机注册与跨总线组帧](../../学习笔记/03-非阻塞通信与CAN收发技术细节.md#12-lk-电机注册与跨总线组帧motor-0405)、[LK 电机模块说明](../../../modules/motor/LKmotor/LK_motor.md)。
