# MC-02 可移植性能模板调研

- 状态：调研稿，已由用户审核（2026-11-08）；对应 MC02-0130–0137，任务状态仍只在 TODO 维护。
- 基线：提交 `09db2f4`（`feat/h723-staged-startup`）；该基线早于 MC02-0087 完成。文中的构建、内存和性能数字均为 2026-11-06 的历史记录，不能代替当前基线。开始 MC02-0130 前，按匹配 Build ID 的固件重新测量；MC02-0087 票据记录的构建数据可作版本对照。
- 证据等级：L1（Level 1，第一级）编译/链接/主机测试；L2（Level 2，第二级）模拟器启动冒烟；L3（Level 3，第三级）H723 裸板测量；L4（Level 4，第四级）整车/执行器。
- 用途：为 MC02-0130–0137 的拆分提供依据；任务状态只在 TODO 维护。

## 0. 摘要

这份调研保留了 2026-11-06 对工程的观察，作为待复核的候选问题，不代表当前分支的实测结论：

1. 旧报告称 AXI SRAM1 的使用率接近满载，并提出检查 AXI SRAM2/DTCM 映射；当前内存边界和可用空间须按 STM32H723 官方资料、最新 linker 与对应构建产物复核，不能沿用旧百分比。
2. 旧代码检查未发现 I-Cache（Instruction Cache，指令缓存）或 D-Cache（Data Cache，数据缓存）启用入口；迁移前需对当前启动代码和平台后端重新核对。
3. INS／电机任务使用 `osDelay(1)` 等待一个 RTOS tick。实际周期还受调用相位、执行时间和调度影响，不能直接按“执行时间 + 1 ms（毫秒）”推算频率；先修正运行时计数器回绕并实测，再确定性能基线。
4. 旧基线观察到 FreeRTOS（Real-Time Operating System，实时操作系统）堆、newlib（C 标准库实现）堆和动态任务创建；需按 MC02-0087 后的活动源码重新盘点。
5. 浮点热点和类型转换收益尚未在匹配当前配置的代码与目标板上测量；F407 的能力和编译选项在平台接入时按器件资料确认。
6. 该基线的 CMake／链接配置只覆盖 H723；当前平台与板卡接口已有变化，跨板扩展仍需以最新构建入口核对。

方案方向：以**板级 profile（配置档）**描述平台能力，以**内存区域描述**统一 H723/F407 的内存布局，以**可选 Cache/MPU（Memory Protection Unit，内存保护单元）**适配有无缓存的芯片，以**固定周期 + 静态任务**提高确定性，最后才评估受限 C++。

## 1. 目标、范围与非目标

### 1.1 目标

- 短期：H723 与 STM32F407 C 型开发板可共用同一套业务/模块接口；F407 先做到描述 + compile-only（只编译）入口，不搬完整 HAL（Hardware Abstraction Layer，硬件抽象层）。
- 中期：覆盖低性能 MCU（Microcontroller Unit，微控制器），通过板级 profile 裁剪功能、降低周期、使用 `float`（单精度浮点）或定点。
- 长期：跨厂商主控；业务不直接依赖 STM32 地址、寄存器或 Cache 假设。
- 高性能平台尽量启用 Cache、TCM（Tightly Coupled Memory，紧耦合内存）、双精度 FPU；低性能平台明确降级，不假装具备。

### 1.2 非目标

- 本轮不重写 C++。
- 本轮不实现 CORDIC（COordinate Rotation DIgital Computer，坐标旋转数字计算）/FMAC（Filter Math Accelerator，滤波数学加速器）驱动。
- 本轮不重新整定 PID（Proportional-Integral-Derivative，比例-积分-微分）参数。
- 本轮不生成 robot 配置固件；车型确认门槛未满足。
- 不把模拟器结果当作 Cache/时序验收。

## 2. 历史测量与复测要求

### 2.1 测量口径

下列数据来自 `bench_safe Debug` 构建产物（`build/wsl/bench-debug/Basic_Framework_MC02_bench_safe.elf`，2026-11-06 19:02）。该产物没有与当时的提交做 Build ID 对照，也早于 MC02-0087 完成；数字仅保存为历史观察。MC02-0130 须用同一固件 SHA／Build ID 重新测量，不能据此直接决定内存放置或性能预算。

### 2.2 段大小

| 段/区域 | 大小 | 位置 | 说明 |
|---|---:|---|---|
| `.text` | 211,360 B | Flash | 代码 |
| `.rodata` | 29,940 B | Flash | 常量、字库、表 |
| `.data` | 892 B | AXI SRAM1 | 有初值全局变量 |
| `.bss` | 74,264 B | AXI SRAM1 | 零初始化全局/静态数据 |
| `._user_heap_stack` | 40,960 B | AXI SRAM1 | 链接脚本预留：主堆 20 KB + 主栈 20 KB |
| `.crash_record` | 512 B | AXI SRAM1 起点 | 复位后保留的崩溃记录 |
| `.log_workspace` | 14,720 B | DTCM（Data Tightly Coupled Memory，数据紧耦合内存） | 日志工作区 |
| `.can_workspace` | 16,992 B | RAM_D2 | CAN（Controller Area Network，控制器局域网）固定队列 |
| `.ws2812_dma` | 128 B | RAM_D3 | WS2812 DMA（Direct Memory Access，直接内存访问）缓冲 |

按这份旧 ELF 的链接布局计算，主 RAM 相关段合计 `116,628 B`；这个数对应 2026-11-06 的旧构建和旧链接布局。带 UI 哨兵的 99.80%/99.77% 也来自当时的记录，二者均不代表 MC02-0087 完成后的当前固件。重新测量时以匹配 Build ID 的 map／ELF 和当前 [MC02-0087 记录](../tickets/MC02-0087.md) 为依据。

### 2.3 主要内存占用者

| 符号/对象 | 大小 | 说明 |
|---|---:|---|
| `ucHeap` | 25,600 B | FreeRTOS 堆，动态任务栈和 TCB（Task Control Block，任务控制块）从这里分配 |
| `ports` | 6,656 B | UART（Universal Asynchronous Receiver/Transmitter，通用异步收发器）/端口实例池 |
| `rx_lookup` | 6,144 B | CAN 接收查找表 |
| `attitude_filter_workspace` | 4,096 B | 姿态滤波工作区 |
| `usart_rxq_pool` | 3,072 B | UART 接收队列池 |
| `UserRxBufferHS` / `UserTxBufferHS` | 2,048 B × 2 | USB（Universal Serial Bus，通用串行总线）HS 缓冲 |
| `log_workspace` | 10,516 B | 日志工作区主体 |

占用来自长期静态缓冲、两套堆、20 KB 主栈预留和 UI 缓冲的叠加。链接脚本把 20 KB 主堆和 20 KB 主栈算进占用，实际高水位可能低于预留值，但新功能仍会被链接期预留挡住。

### 2.4 CPU 与任务（历史观察，待复测）

[学习笔记 02](../../学习笔记/02-CPU占用与实时性入门.md) 记录的基线（USB 供电、无电机、LCD（Liquid Crystal Display，液晶显示）常开）：

| 任务 | CPU 占用 | 说明 |
|---|---:|---|
| `motortask` | 48.2% | 1 kHz 电机控制，含被它承接的中断时间 |
| `instask` | 35.7% | 1 kHz 姿态解算 |
| `lcdtask` | 8.9% | 1 kHz 采样 + 5 Hz 刷屏，实际被饿到约 2.2 Hz |
| IDLE | 5.9% | 剩余余量 |

[platform_os_freertos.c](../../../bsp/os/platform_os_freertos.c) 的 `PlatformTaskDelayMs()` 调用 `osDelay()`；INS/电机任务在每轮结束后等待一个 tick。等待时间取决于调用时的 tick 相位，执行耗时和抢占也会改变实际周期，因此不能用 `T = 执行时间 W + 1 ms` 推算频率。下列数字是旧文档按该假设算出的估计值，不是测量结果，现已撤销其验收用途：

- 电机任务 `f=48.2%` → `W≈0.93 ms` → `T≈1.93 ms`，约 **518 Hz**；
- INS 任务 `f=35.7%` → `W≈0.55 ms` → `T≈1.55 ms`，约 **643 Hz**。

此外，当前 FreeRTOS 运行时间源直接返回 `DWT->CYCCNT / 1000`，DWT 源仍约每 8.94 秒回绕；除法不会延长回绕周期。跨回绕的任务时间差可能错误，详见 [MC02-0142](../../../.Doc/TODO.md)。修复并验证该来源之前，这些历史 CPU 占比不能作为任务频率或容量依据。Phase 0 使用短时 DWT 探针及已验证的周期观测重新测量。

任务栈（1 字 = 4 B）：INS 1024 字 = 4 KB、电机 512 字 = 2 KB、Daemon 512 字 = 2 KB、Robot 1024 字 = 4 KB、UI 512 字 = 2 KB、LCD 1024 字 = 4 KB、UART 发送 512 字 = 2 KB。

### 2.5 Cache、MPU、堆、任务创建

- 仓库中没有 `SCB_EnableICache()` / `SCB_EnableDCache()`；只在 [bsp_adc.c](../../../bsp/adc/bsp_adc.c#L148)、[bsp_flash.c](../../../bsp/flash/bsp_flash.c#L123) 等少数路径做 defensive（防御性）Cache 维护。
- `configENABLE_MPU` 为 0（[FreeRTOSConfig.h](../../../Core/Inc/FreeRTOSConfig.h#L60)）。
- FreeRTOS 堆 25,600 B（[FreeRTOSConfig.h](../../../Core/Inc/FreeRTOSConfig.h#L71)）；链接脚本另预留 20 KB 主堆。
- `configSUPPORT_STATIC_ALLOCATION` 为 1，但平台层的 `osThreadDef_t` 没有填 `buffer/controlblock`，所以任务仍走动态 `xTaskCreate()`；[bsp_tools.c](../../../bsp/bsp_tools.c#L58)、[usart_tx_task.c](../../../bsp/usart/usart_tx_task.c#L43)、XM/DM 电机任务也直接调用动态创建。

### 2.6 平台与构建

- [CMakeLists.txt](../../../CMakeLists.txt#L193) 硬编码 `-mcpu=cortex-m7 ... -mfpu=fpv5-d16` 和 H723 链接脚本。
- [mc02_configure_vehicle.cmake](../../../cmake/mc02_configure_vehicle.cmake#L4) 已有 `MC02_PLATFORM`、`MC02_BOARD`、`MC02_RTOS` 和生成的 `mc02_build_config.h`；但 [mc02_vehicle_selection.cmake](../../../cmake/mc02_vehicle_selection.cmake#L10) 只允许 `stm32h723/mc02/freertos`。
- 因此 F407 目前不能靠配置切换，必须改根 CMake 和身份校验。

### 2.7 测量缺口

- 没有每任务周期/jitter（周期抖动）计数器，只有执行时间探针；
- 没有栈高水位汇总和堆高水位；
- 没有匹配 Build ID 的基线表；
- 没有 Cache 开/关对照；
- 没有 F407 参考平台。

## 3. 平台能力模型

### 3.1 形式

采用“**板级 profile 编译期决定 + 启动期硬性检查 + 手动覆盖**”：

- 编译期：平台/板级 CMake 文件声明 CPU、FPU、Cache、内存区域、CAN/UART 数量、可选功能；CMake 选择源码并生成 `mc02_platform_config.h`。
- 启动期：只检查安全/启动必须成立的条件（时钟、时基、必需外设、内存边界）；不满足则拒绝或明确降级，并写入初始化诊断。
- 手动覆盖：允许 `MC02_*` 构建选项覆盖默认值；构建清单记录覆盖项，启动日志打印实际生效值。

### 3.2 文件布局建议

```text
cmake/platforms/stm32h723.cmake     # CPU/FPU/Cache、内存区域、外设数量
cmake/platforms/stm32f407.cmake     # Cortex-M4F、单精度 FPU、无 Cache
cmake/boards/mc02.cmake             # H723 板：AXI1/AXI2/DTCM/D2/D3、USB/CAN 映射
cmake/boards/rm-c.cmake             # F407 C 型板：SRAM/CCM、CAN/UART 映射
application/config/mc02_platform_config.h.in  # 生成头模板
```

`MC02_PLATFORM`/`MC02_BOARD` 继续由 `mc02_configure_vehicle.cmake` 读取；`mc02_validate_vehicle_identity()` 改为查表，不再写死一个组合。

### 3.3 功能分级

| 等级 | 必须保留 | 可裁剪 |
|---|---|---|
| 最低 | 安全、看门狗、电机控制、IMU（Inertial Measurement Unit，惯性测量单元）、参数存储、最低限度通信 | LCD/UI、视觉、普通日志、CORDIC/FMAC、多电机/多总线扩展 |
| 标准 | 最低 + 遥控/裁判 + 姿态 | LCD/UI、视觉、CORDIC/FMAC |
| 高性能 | 全部 | 无（按需启用 Cache、双精度、加速器） |

具体到硬件：

- 只有 1 路 CAN：板级文件声明 `CAN_BUS_COUNT=1`；业务只注册 CAN1，CAN2/3 相关源码不编译，运行期引用返回“不支持”。
- 没有 CORDIC/FMAC：数学接口选择 CMSIS-DSP（Cortex Microcontroller Software Interface Standard - Digital Signal Processing，Cortex 微控制器软件接口标准数字信号处理库）后端；业务不散落 `#ifdef`。
- 没有 Cache：`PlatformCacheInit()` 为空操作；内存描述里 `cacheable` 为 false；DMA 不需要维护。
- RAM 小：功能 profile 关闭 LCD/UI/视觉，静态池容量由板级文件给出，链接期 `ASSERT` 检查。

### 3.4 F407 C 型参考

官方资料与 RT-Thread BSP 可参考 [RoboMaster 开发板 C 型](https://www.robomaster.com/zh-CN/products/components/general/development-board-type-c)、[RT-Thread STM32F407 RoboMaster C BSP](https://github.com/rt-thread/rt-thread/blob/99428a1e7f7447955aa860f7c969273a12095b8f/bsp/stm32/stm32f407-robomaster-c/README.md)；仓库历史说明也指出原框架基于 F407 C 型（[README.md](../../../README.md#L306)）。

能力表（以 ST 数据手册核对后定稿）：

| 项 | F407 C 型 | H723 MC-02 |
|---|---|---|
| 内核 | Cortex-M4F，168 MHz | Cortex-M7，480 MHz |
| FPU | 单精度 | 单/双精度 |
| Cache | 无 | I/D-Cache 各 16 KB |
| Flash | 1 MB | 768 KB 应用区 + 2×128 KB 参数区 |
| SRAM | 128 KB + 64 KB CCM（Core Coupled Memory，内核耦合内存） | 128 KB 固定 AXI SRAM + 64 KB 固定 ITCM + 192 KB 可按 64 KB 粒度分配给 ITCM 或 AXI SRAM + 128 KB DTCM + 32 KB D2 SRAM + 16 KB D3 SRAM + 4 KB 备份 SRAM |
| DMA 限制 | CCM 不可 DMA | DTCM 不可 DMA，D3 只适合 BDMA（Basic DMA，基本 DMA） |
| 默认控制周期 | 500 Hz | 先测 WCET，再定 1 kHz |

## 4. 内存模型

### 4.1 区域属性

| 字段 | 含义 | 用法 |
|---|---|---|
| `name` | 区域名 | `DTCM`、`AXI1`、`AXI2`、`D2`、`D3`、`CCM`、`SRAM` |
| `base` / `size` | 起始地址、容量 | linker 和生成头共用 |
| `kind` | TCM/SRAM/D2/D3/CCM/Flash/外部 | 决定默认策略 |
| `cacheable` | CPU Cache 是否缓存 | DMA 区必须 false 或由 MPU 覆盖 |
| `dma_masters` | DMA1/DMA2/BDMA/USB 等主机掩码 | 不能只用一个布尔值 |
| `executable` | 是否能取指 | Flash/ITCM 为 true，数据区为 false |
| `stack_ok` / `heap_ok` | 是否允许放栈/堆 | DTCM 适合栈，D3 不适合 |
| `default` | 普通 `.data/.bss` 默认区 | 第一版选 AXI2 |
| `access` | 读/写/执行权限 | 参数区只读 |

### 4.2 H723 第一版布局

| 区域 | 用途 | 预算 |
|---|---|---|
| 固定 AXI SRAM 128 KB | 候选用途为崩溃记录及 DMA 缓冲；实际放置先核对各 DMA 主设备可达性和 Cache 策略 | 软预算待复测后确定 |
| 可共享 D1 SRAM 192 KB | 可按 64 KB 粒度分配给 ITCM 或 AXI SRAM；候选用途包括普通数据或指令，须结合启动配置与链接布局验证 | 软预算待复测后确定 |
| DTCM 128 KB | 日志工作区、主栈、FreeRTOS 静态任务栈/TCB | 软预算 70% |
| RAM_D2 32 KB | CAN 工作区 + 预留 | 现状 17 KB |
| RAM_D3 16 KB | WS2812 DMA | 128 B |
| ITCM 64 KB | 暂不使用；候选热代码见 4.5 | 0 |
| Flash 768 KB | 代码/常量 | 现状约 233 KB |

STM32H723 固定提供 128 KB AXI SRAM、64 KB ITCM 和 128 KB DTCM；另有 192 KB D1 SRAM 可在 ITCM 与 AXI SRAM 之间按 64 KB 粒度共享、32 KB D2 SRAM、16 KB D3 SRAM及 4 KB 备份 SRAM。该容量关系见 [ST RM0468](https://www.st.com/resource/en/reference_manual/rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) 与 [STM32H723VG 数据手册](https://www.st.com/resource/en/datasheet/stm32h723vg.pdf)。共享区的实际分配和 DMA 可达性仍需结合启动寄存器、板级用途和当前 linker 核对。

### 4.3 linker 与符号

- 每板保留静态 linker，不自动生成；CMake 用链接符号核对区域大小。
- 新增 `RAM_AXI2`，不把 `RAM` 简单扩成 320 KB；避免崩溃记录、DMA、普通数据混在一起。
- 崩溃记录仍在 AXI1 起点 512 B；`__mc02_main_ram_*` 改为“崩溃/DMA 区”语义，新增 `__mc02_data_ram_*` 给 AXI2；同步修改 [bsp_crash.c](../../../bsp/crash/bsp_crash.c#L50) 的区域表，避免 AXI2 栈被误判越界。
- `.bss` 放 AXI2 后，启动清零符号必须覆盖 AXI2；如果 `.data/.bss` 跨两个区，启动代码要分别复制/清零，不能只改一个 `_sbss/_ebss`。

### 4.4 堆与栈

- 只保留 newlib 堆（约 20 KB，放 AXI2），实现 `__malloc_lock`/`__malloc_unlock`；FreeRTOS 堆移除。
- 主栈和所有静态任务栈放 DTCM；任务栈尺寸沿用现有值，改完重新量高水位。
- 模块 `malloc` 暂保留，后续静态池再收；最终目标是零运行期分配。

### 4.5 ITCM 候选（暂不迁移）

| 候选 | 依据 | 备注 |
|---|---|---|
| `QuaternionEKF_InstanceUpdate` | 约 5.3 KB，1 kHz INS 主路径 | 收益可能最大 |
| INS/BMI088 采样处理 | 1 kHz | 与 EKF 一起评估 |
| `MotorControlTask`、`CANProcessRx`、PID | 1 kHz 电机/通信 | 控制确定性 |
| `FastTrigSinF32` / `arm_sin_f32` | 小函数、调用频繁 | 表仍在 Flash |
| 故障入口/安全关断 | 故障路径 | 追求“还能跑” |

I-Cache 启用后先测 WCET，再决定是否迁移；ITCM 的价值在确定性，不在平均吞吐。

### 4.6 预算机制

- 板级文件给每区软预算（建议 70%），超软预算 CI 警告；
- linker `ASSERT` 硬容量，超硬容量链接失败；
- CI 输出实际占用、最大符号、趋势；
- 入口脚本读取 `size -A` 或 map，生成 `memory-budget.json`/Markdown 表。

## 5. Cache 与 MPU

### 5.1 I-Cache 与 D-Cache

| | I-Cache | D-Cache |
|---|---|---|
| 缓存对象 | Flash 取来的指令 | SRAM 变量/数组 |
| 与 DMA 的关系 | 基本无关 | 直接相关：DMA 绕过 CPU 读写主存 |
| 风险 | 低 | 高：旧数据/脏数据 |
| 本工程顺序 | 先开，单独测 | 后开，先建 MPU/审计缓冲 |
| 默认 | 验证后默认开 | `MC02_ENABLE_DCACHE` 默认关，opt-in（选择启用） |

### 5.2 MPU 区域表

平台后端维护静态表，启动时一次配置：

| 区域 | 属性 | 作用 |
|---|---|---|
| Flash | 只读、cacheable、可执行 | 保护代码 |
| AXI SRAM1 | 读写、non-cacheable、可共享 | DMA/崩溃区 |
| AXI SRAM2 | 读写、cacheable、非共享 | 普通数据 |
| RAM_D2/D3 | 读写、non-cacheable | DMA |
| 外设寄存器 | Device、non-cacheable、不可执行 | 防止缓存寄存器 |
| 可选栈底/空指针区 | 不可访问 | 越界立即 HardFault（硬件错误） |

Cortex-M7 一般有 16 个 MPU region；驱动只声明属性需求，不直接写 MPU。`configENABLE_MPU` 要先改为 1，并在 `SCB_EnableDCache()` 之前完成表和使能。

### 5.3 DMA 策略

- 固定内部缓冲（ADC（Analog-to-Digital Converter，模数转换器）/USB/CAN/WS2812）放 non-cacheable 区。
- 调用者提供的 SPI（Serial Peripheral Interface，串行外设接口）/UART/I²C（Inter-Integrated Circuit，内部集成电路总线）缓冲通过平台 Cache API 做 32 字节对齐和 clean/invalidate；小数据可用内部 bounce buffer（中转缓冲）。
- 平台 API：`PlatformCacheCleanByAddr`、`PlatformCacheInvalidateByAddr`、`PlatformCacheCleanInvalidateByAddr`；主机后端空操作；驱动不直接碰 SCB（System Control Block，系统控制块）。
- 现有 [bsp_adc.c](../../../bsp/adc/bsp_adc.c#L26) 和 [bsp_flash.c](../../../bsp/flash/bsp_flash.c#L123) 已有 32 字节维护，可迁移到统一 API。

### 5.4 DMA 缓冲审计清单

| 路径 | 当前状态 | D-Cache 前要做什么 |
|---|---|---|
| ADC | 已 32 字节对齐 + clean/invalidate | 可继续用，或移到 non-cacheable 区 |
| Flash | 已 invalidate | 保留 |
| CAN | 工作区在 D2 | 确认 D2 非缓存；无需改动 |
| WS2812 | D3 + 已有 D-Cache 检查 | 确认 BDMA 可访问；无需改动 |
| USB CDC | `UserRxBufferHS/UserTxBufferHS` 在普通 `.bss` | 移到 AXI1 non-cacheable 或加维护 |
| UART 接收 | `recv_buff` 在实例内，DMA 写入 | 审计并移动/维护 |
| SPI/I²C | 调用者缓冲 | 平台 API + 对齐；必要时 bounce buffer |

### 5.5 开启顺序

1. 配 MPU 表（Flash/AXI1/AXI2/D2/D3/外设）；
2. `SCB_EnableICache()`，测一轮；
3. `SCB_InvalidateDCache()`；
4. `SCB_EnableDCache()`；
5. 启动 DMA 前确认缓冲属性；
6. 跑全 DMA 压力测试。

### 5.6 验证

- 裸板同时跑 USB/CAN/UART/SPI/I²C/ADC/WS2812 收发压力；
- 记录 CRC（Cyclic Redundancy Check，循环冗余校验）、丢帧、复位、任务 WCET；
- 不允许只测控制任务就宣称 D-Cache 可用；
- 模拟器不用于 Cache 结论。

## 6. 周期与静态分配

### 6.1 固定周期

- 新增 `PlatformTaskDelayUntilMs()`：以“上次唤醒 + 周期”为基准，平台校验 tick 分辨率；
- 周期不是整数 tick 时返回错误，不静默取整；
- 超时：跳过已错过的周期，不补跑；记录任务名、错过次数；连续超时交给 TaskMonitor 降级/告警；
- 保留 `PlatformTaskDelayMs()` 给非周期任务。

### 6.2 速率

| 任务 | H723 | F407 |
|---|---|---|
| INS | 1 kHz（先测 WCET） | 500 Hz |
| 电机 | 1 kHz（先测 WCET） | 500 Hz |
| Robot | 200 Hz | 100 Hz |
| Daemon | 100 Hz | 50 Hz |
| UART 发送 | 1 kHz | 1 kHz |
| LCD/UI | 低优先级，按需 | 关闭 |

### 6.3 静态任务

- 全部任务静态：核心任务、UART 发送、日志、Daemon 事件、`bsp_tools` 回调、XM/DM 任务；
- 回调池 8、XM 池 4、DM 池 4；容量不足启动报错；
- 每个任务准备 `StaticTask_t` 和栈数组，放 DTCM；
- 平台 API 增加静态缓冲字段，或提供 `PlatformTaskCreateStatic`；
- 移除 FreeRTOS 堆：`configSUPPORT_DYNAMIC_ALLOCATION=0`，不再编 `heap_4.c`；
- 保留 newlib 堆给模块初始化，后续静态池再收。

### 6.4 验收

- 同一固件固定周期前后：周期/jitter、WCET、IDLE、栈高水位；
- 超时注入：任务故意超期，确认跳拍、计数、监控降级；
- 控制行为只验证“时序不变差”，不重新整定 PID；
- F407 只做 compile-only/模拟器，不声称实时性。

## 7. 浮点、LTO 与编译参数

### 7.1 double 分布

| 位置 | 频率 | 处理 |
|---|---|---|
| `attitude_filter`、`vector3`、`sample_filter`、`motor_feedback` | 1 kHz | 改 `float` |
| RLS（Recursive Least Squares，递推最小二乘）/KF（Kalman Filter，卡尔曼滤波） | 低频/数值敏感 | 保留 `double` |
| `kalman_filter` 方差比较 | 1 kHz | 可保留，量小 |
| `math_utils`、LCD 格式化 | 低频 | 不改 |

### 7.2 精度验收

- 用同一输入对照 `double` 参考，记录最大相对/绝对误差；
- 基线后定阈值（建议先看 1e-4 量级是否影响行为）；
- 当前配置枚举的全部主机回归必须通过；
- F407 热路径禁 `double`，用 `-Wdouble-promotion` 提醒。

### 7.3 LTO

- 新增 opt-in `bench-lto`/`robot-lto` 预设；
- 测 Flash/RAM/WCET、Ozone（调试工具）/GDB（GNU Debugger，GNU 调试器）变量可见性、崩溃解析；
- 验证前默认仍 `-O2`；
- 不使用 `-ffast-math`（快速数学），避免破坏 NaN（Not a Number，非数）/`isfinite` 判断。

### 7.4 编译参数

| 参数 | H723 | F407 |
|---|---|---|
| `-mcpu` | `cortex-m7` | `cortex-m4` |
| `-mfpu` | `fpv5-d16` | `fpv4-sp-d16` |
| `-mfloat-abi` | `hard` | `hard` |
| 优化 | Debug `-Og`，Rel `-O2` | 同 |
| printf 浮点 | 可选 | 建议关，省 Flash |
| LTO | opt-in 预设 | opt-in 预设 |

参数由 `cmake/platforms/*.cmake` 定义，板级文件可覆盖；根 CMake 只读配置。

### 7.5 CORDIC/FMAC

- CORDIC：硬件三角函数/向量加速，适合 FOC 变换、角度换算；
- FMAC：硬件 FIR/IIR（Finite Impulse Response/Infinite Impulse Response，有限冲激响应/无限冲激响应）滤波加速；
- 两者都需要定点转换、精度配置、共享仲裁和忙/超时处理；
- 本轮只列为可选，后续用 MC02-0124/0125 评估；先量 float/Cache 后剩余 CPU。

## 8. C++ 评估

### 8.1 结论

不建议整体重写；性能任务稳定后，用受限 C++17 做单模块试点。理由是：

- 当前瓶颈是 Cache、内存布局、周期、double，不是语言；
- 当前测试、安全快照和崩溃记录是需要保留的验证资产；
- C++ 在接口安全上有真实价值：RAII（Resource Acquisition Is Initialization，资源获取即初始化）管理事务/锁、强单位类型、`constexpr` 配置、固定容器。

### 8.2 子集

- `-std=c++17`；
- `-fno-exceptions`（禁用异常）、`-fno-rtti`（禁用运行时类型信息）、`-fno-threadsafe-statics`、`-fno-use-cxa-atexit`；
- 禁用 `new/delete`；固定容器用 ETL（Embedded Template Library，嵌入式模板库）；
- HAL/ISR（Interrupt Service Routine，中断服务程序）/启动保持 C，边界 `extern "C"`；
- 不写全局运行期构造函数；只用 `constexpr`。

### 8.3 试点

- 模块：`bsp/param` 参数事务；
- 用 RAII 封装“暂存—校验—提交/回滚”，保持现有 C API 给调用者；
- 验收：当前主机回归全过并通过新增 C++ 行为用例；Flash/RAM 增量和任务耗时预算在实施前按匹配固件确定；无未经评估的全局构造。

### 8.4 参考

- [powerful_framework](https://gitee.com/hnuyuelurm/powerful_framework/)：同队 C++ 重构版，最直接的迁移样本；
- [Taproot](https://github.com/uw-advanced-robotics/taproot)：面向 RoboMaster 的 C++ 控制/代码生成框架；
- [LibXR](https://github.com/xrobot-org/libxr)：C++ 的 OS/外设兼容层；
- [rm_control](https://github.com/rm-controls/rm_control)：基于 ros-controls 的 C++ 硬件/仿真接口；
- [RoboRTS-Firmware](https://github.com/RoboMaster/RoboRTS-Firmware)：官方 C 固件，对照纯 C 方案；
- [XRobot topic 文档](https://xrobot-org.github.io/docs/basic_coding/middleware/message/message-topic)：消息语义参考。

## 9. 模拟器与证据

| 工具 | 能做什么 | 不能做什么 |
|---|---|---|
| Renode（开源仿真框架） | STM32H753 参考平台，跑未修改 ELF，启动/任务/UART/CAN 部分外设，headless CI | 非周期精确；Cache/Flash 等待/DMA 时序不可信 |
| QEMU（Quick Emulator，快速模拟器） | netduino2/plus2、olimex-stm32-h405、stm32vldiscovery 等 STM32 机器 | 无 H723 模型；外设覆盖有限 |
| Keil/IAR 模拟器 | 指令级、部分外设 | 不适合 CI，外设模型有限 |

证据等级：

- L1：编译、链接、主机测试；
- L2：Renode H753 / QEMU F4 启动冒烟（启动到任务创建、UART 输出/协议有响应）；
- L3：H723 裸板 + DWT + 任务统计（Cache/周期/WCET）；
- L4：整车/执行器，不在本轮。

模拟器可以作为 L2 证据，但不能替代 L3；CI 接入只证明“能启动”，不证明性能。

参考：[Renode STM32H753 参考平台](https://antmicro.com/blog/2026/07/stm32h7-renode-reference-platform)、[QEMU STM32 机器](https://qemu-project.gitlab.io/qemu/system/arm/stm32.html)、[Renode 官网](https://renode.io/)。

## 10. Phase 0 基线

- 配置：`bench_safe` Debug 与 RelWithDebInfo，同一提交/ELF；裸板、无执行器、禁热、禁火。
- 记录：
  - 各段/区域实际占用、`.bss` 前 20 大符号；
  - 每任务周期、jitter、WCET、IDLE；
  - 栈高水位、newlib/FreeRTOS 堆高水位；
  - Cache 关闭状态、MPU 状态。
- 工具：现有 `PlatformProbe`、任务统计、只读观测；不发送控制帧。
- 草拟门槛（基线后定稿）：主 RAM 余量 ≥30%、IDLE ≥15%、1 kHz 偏差 ≤5%、无测试回归。
- 安全：裸板 HIL（Hardware-in-the-Loop，硬件在环）边界不变；不连接电机/执行器；不烧写参数区。

## 11. 风险与未决

- 崩溃记录默认留 AXI1 起点并改符号；审核时可改。
- 192 KB D1 共享 SRAM 的 AXI／ITCM 分配以启动配置为准；每 64 KB 一个分配粒度，不能默认全部作为 AXI SRAM。
- D-Cache 的 USB/UART/SPI/I²C 缓冲审计尚未完成，D 默认关闭。
- F407 细节需以 ST 数据手册/官方 C 型板资料核对后写入。
- 当前 robot 配置被车型确认门槛挡住，Phase 0 只测 bench_safe。
- 模拟器只证明 L2。
- C++ 试点在性能任务之后，不进入本轮生产构建。
- 性能任务编号从 MC02-0130 开始；0129 已被文档审查任务占用。

## 12. 已登记任务范围与依赖

| 编号 | 标题 | 依赖 | 验收要点 | 证据等级 |
|---|---|---|---|---|
| MC02-0130 | 性能基线与可移植内存区域描述 | 0087 | 匹配 Build ID 的基线；核对官方内存图、DMA 可达性及 linker；平台/板级描述、生成头、符号核对；经验证后迁移段并设预算 | L1/L3 |
| MC02-0131 | I-Cache 启用与收益测量 | 0130 | I-Cache 开；同固件前后任务耗时/IDLE；DMA 不变；ITCM 候选表（不迁移）；测试/裸板 | L1/L3 |
| MC02-0132 | D-Cache/MPU 与 DMA 缓冲审计 | 0131 | 平台 MPU 表；固定缓冲 non-cacheable；Cache API；USB/CAN/UART/SPI/I²C/ADC/WS2812 压力；无 CRC/丢帧/复位；D 默认 off | L1/L3 |
| MC02-0133 | 验收已配置任务周期与超时行为 | 0055/0088/0130 | 复用既有固定周期接口；按车型配置测周期、抖动、执行耗时与超期行为，不新增重复调度接口或硬编码频率 | L1/L3 |
| MC02-0134 | 任务静态分配与堆使用审计 | 0133 | 仅覆盖已实现且启用的任务；容量按实际注册需求确定；明确 FreeRTOS/newlib 堆所有权、锁和失败处理 | L1/L3 |
| MC02-0135 | 1 kHz 热路径 float 化与 opt-in LTO | 0131/0133 | 四模块 float；double 参考误差；F407 单精度参数；LTO 预设；Flash/RAM/WCET；测试 | L1/L3 |
| MC02-0136 | F407 C 型平台描述与 compile-only/模拟器冒烟 | 0130/0134 | `stm32f407/rm-c` 平台/板级文件；F407 参数；compile-only；Renode/QEMU 启动冒烟；不改完整 HAL | L1/L2 |
| MC02-0137 | `bsp/param` 受限 C++17 试点 | 0134/0135 | 限定 C++17 子集；当前主机回归和 C++ 行为用例通过；实施前确定 Flash/RAM/耗时预算；无未经评估的全局构造 | L1/L3 |

依赖顺序：0130 → 0131 → 0132 → 0133 → 0134 → 0135/0136 → 0137。0136 可在 0130/0134 后并行；0137 最后。

## 13. 参考

### 本地

- [链接脚本](../../../STM32H723VGTx_FLASH.ld#L64)、[H723 内存映射头](../../../Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h723xx.h#L2059)
- [任务延时实现](../../../bsp/os/platform_os_freertos.c#L41)、[任务循环](../../../application/robot_task.c#L127)、[任务创建](../../../application/robot_task_create.c#L64)
- [FreeRTOS 配置](../../../Core/Inc/FreeRTOSConfig.h#L60)、[构建配置](../../../CMakeLists.txt#L193)
- [CPU 占用笔记](../../学习笔记/02-CPU占用与实时性入门.md)、[MC02-0095](../../../.Doc/TODO.md)、[MC02-0086](../../../.Doc/TODO.md)

### 外部

- ST：[AN4891 Cache](https://www.st.com.cn/resource/en/application_note/dm00306681.pdf)、[AN4838 MPU](https://www.st.com/resource/zh/application_note/an4838-introduction-to-memory-protection-unit-management-on-stm32-mcus-stmicroelectronics.pdf)
- CMSIS-Pack：[Packs 规范](https://open-cmsis-pack.github.io/Open-CMSIS-Pack-Spec/main/html/cp_Packs.html)
- Zephyr：[Kconfig 教程](https://github.com/MichaelTien8901/zephyr-guide-tutorial/blob/main/part2/03-kconfig.md)
- STM32H7 CORDIC：[培训资料](https://www.st.com/content/ccc/resource/training/technical/product_training/group1/67/7c/98/4f/37/38/4a/7f/STM32H7-Peripheral-Cordic_Coprocessor_CORDIC/files/STM32H7-Peripheral-Cordic_Coprocessor_CORDIC.pdf/_jcr_content/translations/en.STM32H7-Peripheral-Cordic_Coprocessor_CORDIC.pdf)、[G4 CORDIC 入门](https://www.st.com/resource/ja/application_note/dm00614795-getting-started-with-the-cordic-accelerator-using-stm32cubeg4-mcu-package-stmicroelectronics.pdf)、[DT0087](https://www.st.com/resource/en/design_tip/dt0087-coordinate-rotation-digital-computer-algorithm-cordic-test-and-performance-verification-stmicroelectronics.pdf)
- RoboMaster：[开发板 C 型](https://www.robomaster.com/zh-CN/products/components/general/development-board-type-c)、[RT-Thread C 型 BSP](https://github.com/rt-thread/rt-thread/blob/99428a1e7f7447955aa860f7c969273a12095b8f/bsp/stm32/stm32f407-robomaster-c/README.md)
- 开源框架：[powerful_framework](https://gitee.com/hnuyuelurm/powerful_framework/)、[Taproot](https://github.com/uw-advanced-robotics/taproot)、[LibXR](https://github.com/xrobot-org/libxr)、[rm_control](https://github.com/rm-controls/rm_control)、[RoboRTS-Firmware](https://github.com/RoboMaster/RoboRTS-Firmware)
