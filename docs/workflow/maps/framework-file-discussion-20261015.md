# MC-02 139 个文件的讨论记录

讨论汇总与登记选择日期：2026-10-15（Asia/Shanghai）。本文保留文件用途和讨论依据，不维护任务进度；唯一任务状态见 [TODO](../../../.Doc/TODO.md)，本轮实施范围和验收设计见[范围规格](../specs/robomaster-framework-evolution.md)。

用户最终批准 R01–R08、R13–R21、R23–R26、R28–R29 共 23 个主题；R09、R10、R11、R12、R22、R27 本次不新增。已有任务不因暂缓被删除。下方“讨论结论”是讨论快照，不代表每行都进入本轮实施；具体以范围规格和 TODO 为准。1–139 为文件讨论编号，R01–R29 为主题标签，都不是正式任务 ID。

## 最近确认及已覆盖的旧方案

- 2026-10-15 最后追加：R29 的 RGB 颜色／闪烁频率接入 safety 故障快照，并与蜂鸣器联动；以配置映射和优先级统一仲裁。
- 第139项：本轮适配 Fusion，作为配置可选的姿态后端，与现有 QuaternionEKF 共用姿态接口。之前“仅参考、以后接入”的建议已被此选择覆盖。
- 正式自主固件默认具有运行资格，是否启用按车型配置；无需逐场人工预授权。实际控制仍要求有效裁判阶段和全部核心条件。
- 运行故障复位包括 HardFault、栈溢出、断言等，统一允许按配置有限恢复：重启后完整初始化与核心检查通过才恢复。调试保持失能；当前启动的核心初始化失败锁存，超额恢复仍失能。
- 辅助模块失败可以降级，核心初始化／核心依赖失败不能继续控制；故障影响范围由统一安全管理按配置计算。
- USB 用于上下位机业务通信；调参和诊断另走独立串口。
- 哨兵移动命令可以降低自旋速度，优先实现平移。自旋策略是否采用同向偏置正弦由配置给出。
- 自动击发时机由视觉决定，下位机不额外设云台瞄准误差门槛；基础许可、核心反馈、裁判限制和发射机构就绪仍需检查。
- 发射保持请求射速，热量不足或机构未就绪时暂停供弹。卡弹恢复为配置间隔与有限次数的自动试运行。
- IMU 保留 gyro、acc、temperature 全部成功后交付整组样本，使用统一时间与有效性。未选择分量独立有效的公共接口。
- 公共滤波本轮包含二阶低通与陷波；姿态本轮包含基于安装几何和运动反馈的加速度补偿；功率本轮包含可配置 RLS 在线辨识。
- PID、KF 工作内存由调用模块提供。DR、步进、半舵本轮保留占位；选用未实现驱动不能伪成功。

## 已讨论部分的结构

```text
application/    车型配置、整车启动、命令仲裁、底盘／云台／发射业务
bsp/            CAN／UART／USB／SPI／I²C、GPIO／PWM、时基、ADC、Flash、日志与底层保护
modules/        安全管理、任务监控、消息、来源监控、电机、算法、BMI088 与姿态服务
```

139 个文件并不等于整个工程已全部讨论完。IMU 加热、其余传感器、上下位机协议、裁判、功率、电容、告警、显示等后续文件仍可继续逐项讨论；生成代码、厂商库与资料没有在这里虚增为已讨论业务项。

## 逐文件用途和讨论结论

### 1–29：应用层：配置、整车、命令与业务控制

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 1 | [application/config/config.h](../../../application/config/config.h) | 机器人类型与底盘、云台、发射等配置结构 | 以车型配置组织功能、硬件与参数；编译选择车型，每个兵种维护一套当前配置。 |
| 2 | [application/robot_def.h](../../../application/robot_def.h) | 整车模式、命令和反馈数据定义，以及历史板级与机械宏 | 统一类型、单位、参考系与配置来源，整理历史硬编码；首台哨兵采用单 MC-02 整车控制。 |
| 3 | [application/config/sentinel_robot/sentinel_config.c](../../../application/config/sentinel_robot/sentinel_config.c) | 哨兵机械、电机与控制参数实例 | 首台为四轮全向轮、底盘→大 yaw→小 yaw→pitch；板载 IMU 在 pitch 架，大 yaw 可连续多圈。轴系可配置扩展；一套发射机构，摩擦轮数量与电机型号可配置。 |
| 4 | [application/config/sentinel_robot/sentinel_config.h](../../../application/config/sentinel_robot/sentinel_config.h) | 哨兵配置初始化入口声明 | 每个兵种只保留一套当前配置，机械改版时更新。 |
| 5 | [application/robot_def.c](../../../application/robot_def.c) | 按车型选择并创建整车配置 | 选择尚未实现配置的车型时构建报错，补齐后才能生成固件。 |
| 6 | [application/robot.c](../../../application/robot.c) | 整车模块初始化及运行组织 | 按车型配置组合模块并校验依赖；辅助失败允许降级，核心初始化或核心依赖失败不能继续控制。 |
| 7 | [application/robot.h](../../../application/robot.h) | 整车对外接口 | 提供统一只读健康快照，复用已有状态来源。 |
| 8 | [application/robot_task.h](../../../application/robot_task.h) | RTOS 任务入口和周期调度 | 底盘、云台、发射使用独立任务；周期、优先级等按车型配置，启动读取、运行固定、修改后重启。 |
| 9 | [application/init_diagnostic.c](../../../application/init_diagnostic.c) | 初始化结果和故障诊断记录 | 保留有限 RAM 初始化事件：时间、模块、原因、严重程度，支持查询和导出。 |
| 10 | [application/init_diagnostic.h](../../../application/init_diagnostic.h) | 初始化诊断与失败状态接口 | 核心初始化失败在当前启动锁存；修正后重启并重新完成初始化。运行故障复位另按有限恢复策略。 |
| 11 | [application/cmd/robot_cmd.c](../../../application/cmd/robot_cmd.c) | 遥控、视觉、自主输入的命令仲裁 | 调试遥控掉线急停；正式自主允许无遥控。默认自主资格由车型配置决定，以有效裁判阶段和核心状态决定运行。视觉超时禁发射但继续自旋和本地索敌。 |
| 12 | [application/cmd/robot_cmd.h](../../../application/cmd/robot_cmd.h) | 命令模块接口与控制来源约定 | 明确纯手动、手动＋视觉辅助、全自主三种模式；辅助模式人工决定击发，视觉只辅助瞄准。 |
| 13 | [application/cmd/robot_cmd.md](../../../application/cmd/robot_cmd.md) | 遥控器、键鼠及命令模块接入说明 | 使能、失能和急停操作统一，其余操作按车型配置。 |
| 14 | [application/chassis/chassis.c](../../../application/chassis/chassis.c) | 底盘业务控制与供电约束 | 超级电容、额外底盘 IMU、里程计按车型配置；电容不可用时按普通供电限制功率。 |
| 15 | [application/chassis/chassis.h](../../../application/chassis/chassis.h) | 底盘命令和反馈接口 | 输出实际速度，可选本地里程计位姿；场地定位与导航由上位机负责。 |
| 16 | [application/chassis/chassis_motion.c](../../../application/chassis/chassis_motion.c) | 底盘运动换算、轮速限制与运动分配 | 平移、自旋分配策略可配置；哨兵收到移动指令时可减缓自旋，优先实现平移，并受轮速与功率限制。 |
| 17 | [application/chassis/chassis_motion.h](../../../application/chassis/chassis_motion.h) | 底盘运动计算与参考系接口 | 命令明确底盘、大 yaw、枪口或场地参考系；框架统一转换，车型声明允许范围，反馈缺失或过期撤销依赖命令。 |
| 18 | [application/chassis/chassis.md](../../../application/chassis/chassis.md) | 底盘模式和运动策略说明 | 自旋可选择带偏置的同向正弦转速；策略是否启用、幅值、周期、偏置、变化限制和移动降速均配置。 |
| 19 | [application/chassis/balance.h](../../../application/chassis/balance.h) | 当前为空的平衡／轮腿底盘占位 | 预留麦轮、舵轮、全向轮、轮腿、半舵五种类型；统一接口，并按配置选择源文件进入编译。轮腿具体实现范围仍需细化。 |
| 20 | [application/chassis/mecanum.h](../../../application/chassis/mecanum.h) | 当前为空的麦轮底盘占位 | 统一运动接口使用 m/s、rad/s 和前左上坐标；旧配置、电机单位和安装差异显式转换。 |
| 21 | [application/chassis/steering.h](../../../application/chassis/steering.h) | 当前为空的舵轮底盘占位 | 舵轮接统一类型接口；半舵构型尚未确定，本轮只预留文件。 |
| 22 | [application/gimbal/gimbal.c](../../../application/gimbal/gimbal.c) | 云台控制、姿态反馈和电机协同 | 三轴搜索：底盘自旋，大 yaw 补偿实际自旋并缓扫，小 yaw 左右扫、pitch 点头；锁定后持续协同回中。丢目标先局部重捕获后全局扫描；饱和时可按配置请求底盘降自旋，哨兵默认启用。 |
| 23 | [application/gimbal/gimbal.h](../../../application/gimbal/gimbal.h) | 云台命令、反馈和状态接口 | 自动击发时机由视觉端决定；下位机检查基础使能、核心反馈和裁判约束，不额外增加云台瞄准误差开火门槛。 |
| 24 | [application/gimbal/gimbal.md](../../../application/gimbal/gimbal.md) | 云台控制与人工操作说明 | 人工控制枪口整体 yaw、pitch；框架协调大小 yaw，让小 yaw 持续回中，结构与策略由配置声明。 |
| 25 | [application/shoot/shoot.c](../../../application/shoot/shoot.c) | 摩擦轮与拨弹控制、机构就绪和卡弹处理 | 所有启用摩擦轮反馈有效、达到目标且稳定后才供弹；未就绪的单发／三连发拒绝并需新请求。卡弹有限反转解堵，失败停用发射，底盘和云台继续。 |
| 26 | [application/shoot/shoot.h](../../../application/shoot/shoot.h) | 发射状态、故障与恢复接口 | 解堵失败后按配置间隔和有限次数自动试运行，正常后恢复；仍需相应许可，不能无限重试。 |
| 27 | [application/shoot/shoot.md](../../../application/shoot/shoot.md) | 发射行为、热量与调试说明 | 保持请求射速，仅在热量余量不足或机构未就绪时暂停供弹。 |
| 28 | [application/APP层应用编写指引.md](../../../application/APP层应用编写指引.md) | 应用模块编写和接入指南 | 本轮先支持裸板模块独立调试；主机模拟／回放作为后续扩展。 |
| 29 | [application/application.md](../../../application/application.md) | 应用层组织及整车模式说明 | 健康状态下切换模式保持使能、平滑接管；撤销旧控制来源和击发授权，不额外要求重新使能。 |

### 30–56：BSP：通信、资源、计时与采样

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 30 | [bsp/can/bsp_can.c](../../../bsp/can/bsp_can.c) | CAN 注册、收发及中断交付 | 增加按用途的发送调度：周期目标仅保留最新值，一次性命令有界队列与重试，优先级和期限可配置。 |
| 31 | [bsp/can/bsp_can.h](../../../bsp/can/bsp_can.h) | CAN 实例、帧和接口定义 | 本轮支持标准／扩展 ID 的经典 CAN，最多 8 字节；较长 CAN FD 帧以后按设备需求扩展。 |
| 32 | [bsp/can/bsp_can.md](../../../bsp/can/bsp_can.md) | CAN 接入、配置和故障恢复说明 | 总线恢复且必需反馈连续有效后，在当前运行资格有效时自动恢复；判据和等待时间配置，旧控制帧撤销。 |
| 33 | [bsp/usart/bsp_usart.c](../../../bsp/usart/bsp_usart.c) | 串口 DMA／中断收发封装 | 每串口有界发送缓冲和队列，提交时复制数据，完成或超时后释放并反馈状态。 |
| 34 | [bsp/usart/bsp_usart.h](../../../bsp/usart/bsp_usart.h) | 串口注册与收发接口 | BSP 交付不可变字节块，包含实际长度、原始接收时间和上下文；协议模块负责拼帧、拆帧与校验。 |
| 35 | [bsp/usart/bsp_usart.md](../../../bsp/usart/bsp_usart.md) | 串口接入与异常输入说明 | 短暂坏帧不刷新有效数据时间；上一合法快照在原期限内有效，过期撤销对应控制。 |
| 36 | [bsp/usb/bsp_usb.c](../../../bsp/usb/bsp_usb.c) | USB CDC 业务通信封装 | USB 当前用于上下位机业务通信；调参和诊断另外走独立串口。 |
| 37 | [bsp/usb/bsp_usb.h](../../../bsp/usb/bsp_usb.h) | USB 收发及链路接口 | 正式自主中 USB 失效时撤销旧移动、瞄准和开火命令，停止平移、禁发射，继续原地自旋与本地三轴索敌；恢复只使用新有效命令。 |
| 38 | [bsp/usb/bsp_usb.md](../../../bsp/usb/bsp_usb.md) | USB 接入与生成代码协调说明 | 补齐自定义代码位置、接入说明及 CubeMX 重新生成后的通信验证流程。 |
| 39 | [bsp/spi/bsp_spi.c](../../../bsp/spi/bsp_spi.c) | SPI 设备、片选及总线传输封装 | 加入有界事务队列，按配置优先级与期限调度，统一占用、完成、错误、超时释放与结果。 |
| 40 | [bsp/spi/bsp_spi.h](../../../bsp/spi/bsp_spi.h) | SPI 事务与缓冲区接口 | BSP 使用有界事务缓冲，复制发送数据，完成后交付只读接收结果；超时／取消后确认硬件已结束再复用。 |
| 41 | [bsp/spi/bsp_spi.md](../../../bsp/spi/bsp_spi.md) | SPI 工作模式与接入说明 | 仅配置明确允许、替代模式满足时限时，DMA 启动不可用可降级到中断模式；报告实际模式，否则按模块重要性处理。 |
| 42 | [bsp/iic/bsp_iic.c](../../../bsp/iic/bsp_iic.c) | I²C 注册与总线读写封装 | 加入统一有界事务调度与缓冲管理；连续多步协议操作不可插队，修复注册、容量、状态反馈和超时。 |
| 43 | [bsp/iic/bsp_iic.h](../../../bsp/iic/bsp_iic.h) | I²C 地址与读写接口 | 本轮支持 7 位设备地址、8／16 位寄存器地址；配置设备地址不预先左移，10 位设备地址后续扩展。 |
| 44 | [bsp/iic/bsp_iic.md](../../../bsp/iic/bsp_iic.md) | I²C 接入和恢复说明 | 加入配置限定次数与时限的自动恢复和诊断，重验设备状态，失败按受影响模块的重要性处理。 |
| 45 | [bsp/gpio/bsp_gpio.c](../../../bsp/gpio/bsp_gpio.c) | GPIO 注册、读写与 EXTI 分发 | 默认记录事件并通知任务；确有实时需求时，显式允许短中断回调。 |
| 46 | [bsp/gpio/bsp_gpio.h](../../../bsp/gpio/bsp_gpio.h) | GPIO／EXTI 实例与资源声明 | 加入统一板级资源声明与冲突校验，覆盖 GPIO、外设复用与 EXTI，并核对 CubeMX 和源码的一致性。 |
| 47 | [bsp/gpio/bsp_gpio.md](../../../bsp/gpio/bsp_gpio.md) | GPIO 接入和数字输入说明 | 加入通用数字输入模块：有效电平转换、可选去抖、状态变化事件，参数按配置。 |
| 48 | [bsp/pwm/bsp_pwm.c](../../../bsp/pwm/bsp_pwm.c) | 定时器 PWM 通道封装 | 共享同一定时器的通道保持同一固定波形周期，各自调占空比／脉宽；运行时变频用途独占定时器。 |
| 49 | [bsp/pwm/bsp_pwm.h](../../../bsp/pwm/bsp_pwm.h) | PWM 注册、启动与输出接口 | 注册仅检查、分配与配置资源；所有通道由功能模块显式启动。 |
| 50 | [bsp/pwm/bsp_pwm.md](../../../bsp/pwm/bsp_pwm.md) | PWM 接入和时序验收说明 | 加入只读时序诊断与裸板波形验收，区分配置值、寄存器推算值、仪器实测值，诊断复用独立串口。 |
| 51 | [bsp/dwt/bsp_dwt.c](../../../bsp/dwt/bsp_dwt.c) | DWT 时间与耗时测量 | 加入统一整数单调时基和期限接口，复用于新鲜度、超时和事件；保留短时低开销探针。 |
| 52 | [bsp/dwt/bsp_dwt.h](../../../bsp/dwt/bsp_dwt.h) | 计时与延时接口 | 区分有上限短忙等与让出式／分阶段等待，迁移长忙等，调整初始化阶段。 |
| 53 | [bsp/dwt/bsp_dwt.md](../../../bsp/dwt/bsp_dwt.md) | 计时和任务耗时观测说明 | 除基础统计外增加耗时分布和有限近期时序记录，按配置启用并验证采集开销。 |
| 54 | [bsp/adc/bsp_adc.c](../../../bsp/adc/bsp_adc.c) | ADC 采样、VIN 与模拟按键输入 | 扩展为配置选择模拟通道、采样参数和用途映射，统一发布有效样本快照。 |
| 55 | [bsp/adc/bsp_adc.h](../../../bsp/adc/bsp_adc.h) | ADC 原始样本与换算接口 | BSP 提供原始值和可配置线性换算，复杂曲线由传感器模块处理；采样与换算有效性分开。 |
| 56 | [bsp/adc/bsp_adc.md](../../../bsp/adc/bsp_adc.md) | 模拟量采样和标定说明 | 加入实测标定流程、系数持久化与查询／导出，记录量程内误差和通道／电路配置关联。 |

### 57–81：BSP：存储、日志、关断、恢复与事件

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 57 | [bsp/flash/bsp_flash.c](../../../bsp/flash/bsp_flash.c) | 当前旧内部 Flash 擦写封装 | 重写为 H723 底层 Flash 驱动，再由 bsp_param 复用。 |
| 58 | [bsp/flash/bsp_flash.h](../../../bsp/flash/bsp_flash.h) | 底层 Flash 寻址和操作接口 | 使用已声明分区＋偏移，统一范围、对齐、擦写粒度与链接布局校验；程序区默认只读。 |
| 59 | [bsp/flash/bsp_flash.md](../../../bsp/flash/bsp_flash.md) | 内部 Flash 使用与维护说明 | 所有底层擦写入口共用维护保护，缺少有效授权就拒绝；统一失能确认、总期限和结束结果。 |
| 60 | [bsp/param/bsp_param.c](../../../bsp/param/bsp_param.c) | 参数缓存、持久记录与加载保存 | 统一参数描述和基础校验，包含类型、单位、范围、默认值、版本和生效方式；模块补专用约束，失败不改旧值。 |
| 61 | [bsp/param/bsp_param.h](../../../bsp/param/bsp_param.h) | 参数读写与事务接口 | 加入参数组事务：整体校验、一次接受、失败保留旧组；区分暂存、生效与保存结果。 |
| 62 | [bsp/log/bsp_log.c](../../../bsp/log/bsp_log.c) | 日志提交、格式化与输出 | 加入有界异步日志，实时路径提交事件、日志任务格式化发送，普通文本有界复制接入。 |
| 63 | [bsp/log/bsp_log.h](../../../bsp/log/bsp_log.h) | 日志等级、事件和提交接口 | 为关键记录预留容量，重复告警聚合，普通日志按配置限速／丢弃；提交有界并统计损失。 |
| 64 | [bsp/log/bsp_log.md](../../../bsp/log/bsp_log.md) | 日志接入与比赛输出策略说明 | 正式固件默认关键故障和状态变化；普通日志按模块过滤，诊断串口可临时调整编译上限内等级。 |
| 65 | [bsp/watchdog/bsp_watchdog.c](../../../bsp/watchdog/bsp_watchdog.c) | 硬件看门狗配置与喂狗封装 | 启动、正常、维护窗口按车型配置，校验硬件范围与任务时限，反馈实际结果；正常喂狗由关键任务监控集中决定。 |
| 66 | [bsp/watchdog/bsp_watchdog.h](../../../bsp/watchdog/bsp_watchdog.h) | 看门狗与复位原因接口 | 调试复位后保持失能；正式自主可配置有限恢复，重新检查初始化、核心状态与有效裁判阶段。最终决定无需逐场人工预授权。 |
| 67 | [bsp/safety/bsp_safety.c](../../../bsp/safety/bsp_safety.c) | 异常情况下直接关断可控硬件输出 | 加入编译期板级静态关断清单及车型覆盖校验；异常路径仅执行直接寄存器操作。 |
| 68 | [bsp/safety/bsp_safety.h](../../../bsp/safety/bsp_safety.h) | 底层最终关断与安全入口 | 各运行故障复位统一按有限恢复处理，只要重启后初始化和全部核心检查通过；原因保留诊断。 |
| 69 | [bsp/safety/bsp_safety_error.c](../../../bsp/safety/bsp_safety_error.c) | Error_Handler 等致命错误适配 | 先关断、尽力记录类别／阶段／位置，再进入有限复位恢复；恢复超限保持失能，预期辅助失败用结果返回。 |
| 70 | [bsp/crash/bsp_crash.c](../../../bsp/crash/bsp_crash.c) | 异常现场捕获和 RAM 崩溃记录 | 本轮加入有限持久归档，异常时只写 RAM，重启后满足维护条件再归档；实际介质与分区尚需确认。 |
| 71 | [bsp/crash/bsp_crash.h](../../../bsp/crash/bsp_crash.h) | 崩溃记录格式与查询接口 | 关联统一构建标识、车型与配置版本；保存匹配 ELF／map 和解析说明，用原版本解析旧记录。 |
| 72 | [bsp/crash/bsp_crash_guard.c](../../../bsp/crash/bsp_crash_guard.c) | 异常记录过程的重入保护 | 整车跨复位限制连续恢复次数，包含二次异常；恢复控制后稳定运行达到配置时长自动清零。 |
| 73 | [bsp/crash/bsp_crash_guard.h](../../../bsp/crash/bsp_crash_guard.h) | 异常重入保护接口 | 统一健康快照报告恢复次数、剩余额度和稳定等待；清除／导出诊断记录不改变恢复计数。 |
| 74 | [bsp/crash/bsp_crash_frame.c](../../../bsp/crash/bsp_crash_frame.c) | 异常堆栈帧合法性检查和读取 | 可读内存边界统一由链接布局提供，并加入一致性验证，保持有界读取。 |
| 75 | [bsp/crash/bsp_crash_frame.h](../../../bsp/crash/bsp_crash_frame.h) | 堆栈解析结果与失败信息 | 增加具体失败原因和实际可读快照长度，随记录导出，读取失败不访问无效地址。 |
| 76 | [bsp/ws2812/ws2812.c](../../../bsp/ws2812/ws2812.c) | 板载 RGB 灯的 SPI 波形输出 | 本轮修现有 SPI 方案：资源校验、超时、有界异步刷新，核对灯珠与波形，解决与 ADC 按键的资源冲突。 |
| 77 | [bsp/ws2812/ws2812.h](../../../bsp/ws2812/ws2812.h) | RGB 输出提交接口 | 本轮仅板载单灯，补齐提交和发送结果；多灯以后扩展，颜色／闪烁／故障优先级归上层。 |
| 78 | [bsp/bsp_tools.c](../../../bsp/bsp_tools.c) | 模块回调任务与事件辅助设施 | 重构统一事件接口，支持绑定已有任务或按配置创建独立任务，收到事件才执行，容量和创建失败明确反馈。 |
| 79 | [bsp/bsp_tools.h](../../../bsp/bsp_tools.h) | 事件语义和任务绑定接口 | 按来源区分有序事件、计数事件和可合并状态，明确数据归属、原始时间和溢出结果。 |
| 80 | [bsp/bsp_init.h](../../../bsp/bsp_init.h) | BSP 初始化组织入口 | 按车型配置和依赖分阶段初始化；先关断、诊断、时基、看门狗，再在合适阶段处理 IRQ／DMA／长等待，逐项反馈。 |
| 81 | [bsp/bsp.md](../../../bsp/bsp.md) | BSP 层设计及接入指南 | 本轮统一软件 CRC 接口与校验向量；异常路径独立软件实现，硬件加速后续按耗时需要加入。 |

### 82–95：通用服务与电机调度

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 82 | [modules/safety/robot_safety.c](../../../modules/safety/robot_safety.c) | 整车安全状态与运行许可计算 | 统一整车授权和底盘、云台、发射等功能许可；按车型配置故障影响范围与依赖，复用模块状态，核心初始化失败全局失能。 |
| 83 | [modules/safety/robot_safety.h](../../../modules/safety/robot_safety.h) | 安全状态、请求和许可接口 | 单一管理者发布一致许可快照，保留急停与关键输出禁止立即生效的通道。 |
| 84 | [modules/task_monitor/task_monitor.c](../../../modules/task_monitor/task_monitor.c) | 任务活性、期限与看门狗喂狗依据 | 覆盖所有配置启用任务，区分周期／事件与核心／辅助；正常事件等待不算卡死，故障按依赖处理。 |
| 85 | [modules/task_monitor/task_monitor.h](../../../modules/task_monitor/task_monitor.h) | 任务注册、进度和等待接口 | 增加开始／完成进度、等待原因和总期限，避免反复续期掩盖卡死；设备离线与任务不推进分开判断。 |
| 86 | [modules/message_center/message_center.c](../../../modules/message_center/message_center.c) | 模块间消息发布、订阅和存储 | 支持最新快照与有界事件 FIFO，分别声明容量、期限、溢出和消费规则，过期命令不执行。 |
| 87 | [modules/message_center/message_center.h](../../../modules/message_center/message_center.h) | 话题注册和消息接口 | 统一话题类型、大小、语义、发布订阅关系；提供类型约束 C 接口和启动关系校验，运行期固定注册。 |
| 88 | [modules/message_center/message_freshness.h](../../../modules/message_center/message_freshness.h) | 消息时间和新鲜度判断 | 区分源数据时间与发布时间，读取或转发不延长原有效期；跨设备时钟由协议适配。 |
| 89 | [modules/message_center/message_center.md](../../../modules/message_center/message_center.md) | 消息接入和排查说明 | 加入只读话题关系与有界统计查询：发布、覆盖、溢出、过期、积压，经独立串口导出。 |
| 90 | [modules/daemon/daemon.c](../../../modules/daemon/daemon.c) | 设备／通信来源的在线与超时监控 | 使用统一单调时基和绝对期限，区分等待首帧、已验证在线、离线；只有合法数据刷新期限，启动宽限不代替有效数据。 |
| 91 | [modules/daemon/daemon.h](../../../modules/daemon/daemon.h) | 来源监控注册和状态查询 | 分别报告通信在线、关键数据有效和恢复验证状态，统一安全管理计算功能许可。 |
| 92 | [modules/daemon/daemon.md](../../../modules/daemon/daemon.md) | 来源监控、离线处理和恢复说明 | 离线回调只发布有界事件，恢复由对应模块异步执行，按配置授权、次数和时限约束。 |
| 93 | [modules/motor/motor_def.h](../../../modules/motor/motor_def.h) | 公共电机配置、控制量和反馈定义 | 统一跨型号接口和能力声明，输出轴位置／速度用 rad、rad/s；减速比与方向配置，力矩／电流／电压明确区分。 |
| 94 | [modules/motor/motor_task.c](../../../modules/motor/motor_task.c) | 电机接收、控制和发送任务组织 | 电机关键路径与串口／USB 其他协议分任务处理，按车型配置周期、优先级和预算，保留唯一消费者与原始接收时间。 |
| 95 | [modules/motor/motor_task.h](../../../modules/motor/motor_task.h) | 电机任务调度接口 | 计算与发送周期分别配置，校验设备能力、反馈新鲜度、总线带宽和端到端时限，发送最新有效结果。 |

### 96–115：电机和执行器驱动

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 96 | [modules/motor/DJImotor/dji_motor.c](../../../modules/motor/DJImotor/dji_motor.c) | DJI 电机反馈解析、闭环与分组发送 | 首帧建立编码器基准；间断后多圈位置不可信时禁止依赖它的控制，重建可信参考后恢复，位置与速度有效性分别表达。 |
| 97 | [modules/motor/DJImotor/dji_motor.h](../../../modules/motor/DJImotor/dji_motor.h) | DJI 实例、外部反馈与前馈接口 | 外部反馈／前馈绑定只读快照，含单位、参考系、采样时间、有效性；控制取一致数据，切换与过期统一校验。 |
| 98 | [modules/motor/DJImotor/dji_motor.md](../../../modules/motor/DJImotor/dji_motor.md) | DJI 电机接入与滤波说明 | 反馈滤波按实例配置截止频率／时间常数，使用实际采样间隔，允许关闭，保留原始数据并验证延迟。 |
| 99 | [modules/motor/DMmotor/dmmotor.c](../../../modules/motor/DMmotor/dmmotor.c) | 达妙电机命令和反馈适配 | 注册后保持失能，由模块显式使能；写零点是独立标定操作。 |
| 100 | [modules/motor/DMmotor/dmmotor.h](../../../modules/motor/DMmotor/dmmotor.h) | 达妙模式、量程与控制接口 | 本轮支持完整 MIT、位置＋速度、速度三种基础模式；启动选择，量程／能力匹配型号与固件，其他模式后续扩展。 |
| 101 | [modules/motor/LKmotor/LK9025.c](../../../modules/motor/LKmotor/LK9025.c) | LK 电机反馈和命令发送 | 支持单电机和按 CAN 总线分组的多电机发送，由配置选择并匹配设备协议；同一电机不混用两种发送模式。 |
| 102 | [modules/motor/LKmotor/LK9025.h](../../../modules/motor/LKmotor/LK9025.h) | LK 控制与反馈接口 | 支持 MCU 闭环输出电流目标和驱动器原生速度／位置闭环，按配置选择；同一控制环只有一个计算者。 |
| 103 | [modules/motor/LKmotor/LK_motor.md](../../../modules/motor/LKmotor/LK_motor.md) | LK 协议与接入说明 | 加入设备端参数只读核对和显式维护写入，相关功能失能，区分临时／持久保存并读回实际结果。 |
| 104 | [modules/motor/HTmotor/HT04.c](../../../modules/motor/HTmotor/HT04.c) | 海泰 MIT 风格电机驱动 | 与达妙共用参数化 MIT 编解码核心；型号量程、帧格式、特殊命令和物理量差异保留在驱动。 |
| 105 | [modules/motor/HTmotor/HT04.h](../../../modules/motor/HTmotor/HT04.h) | 海泰模式和控制接口 | 同时支持 MCU 闭环及完整 MIT，按实例配置，明确量纲／范围与控制环计算归属。 |
| 106 | [modules/motor/HTmotor/HT04.md](../../../modules/motor/HTmotor/HT04.md) | 海泰接入及关节标定说明 | 加入可复用、可配置、有界的机械回零流程，人工维护触发，成功建立可信参考，失败上报。 |
| 107 | [modules/motor/DRmotor/drmotor.c](../../../modules/motor/DRmotor/drmotor.c) | 未实现的 DR 电机占位 | 本轮只保留占位，配置选用明确报错；厂商与完整型号未确定。 |
| 108 | [modules/motor/DRmotor/drmotor.h](../../../modules/motor/DRmotor/drmotor.h) | DR 占位接口 | 本轮保留最小占位，具体设备接入时再整理新驱动模板。 |
| 109 | [modules/motor/XMmotor/xmmotor.c](../../../modules/motor/XMmotor/xmmotor.c) | 当前不完整的 XM 电机骨架 | 本轮补齐 H723 经典 CAN 扩展帧、反馈解析与统一控制接口，并验证，不允许骨架伪注册成功。 |
| 110 | [modules/motor/XMmotor/xmmotor.h](../../../modules/motor/XMmotor/xmmotor.h) | XM 电机协议目标和配置接口 | 首个目标为小米 CyberGear，具体型号／固件版本与协议量程待确认。 |
| 111 | [modules/motor/servo_motor/servo_motor.c](../../../modules/motor/servo_motor/servo_motor.c) | 舵机命令输出与协议处理 | 本轮支持 PWM 位置型、PWM 连续旋转型与确认协议的总线舵机；能力明确，无标定不虚构物理转速。 |
| 112 | [modules/motor/servo_motor/servo_motor.h](../../../modules/motor/servo_motor/servo_motor.h) | 舵机动作和状态接口 | 提供带请求编号的有界动作及接受／执行／完成／失败状态；输出完成与真实到位分开，无反馈时到位未知。 |
| 113 | [modules/motor/servo_motor/servo_motor.md](../../../modules/motor/servo_motor/servo_motor.md) | 舵机接入与角度映射说明 | 支持端点线性映射和可选多点分段标定，人工提供数据，关联型号与安装，复用参数保存和导出。 |
| 114 | [modules/motor/step_motor/step_motor.c](../../../modules/motor/step_motor/step_motor.c) | 未实现的步进驱动占位 | 本轮保留占位，配置选用明确报错，待实际驱动器和接口需求确定。 |
| 115 | [modules/motor/step_motor/step_motor.h](../../../modules/motor/step_motor/step_motor.h) | 步进驱动占位接口 | 后续接入统一电机接口，以能力声明区分脉冲推算位置与实测反馈；本轮仍占位。 |

### 116–130：公共算法

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 116 | [modules/algorithm/controller.c](../../../modules/algorithm/controller.c) | PID 计算和内部状态更新 | 调用者显式传入实测 dt（秒），算法独立于硬件时基；异常 dt／数值明确失败。 |
| 117 | [modules/algorithm/controller.h](../../../modules/algorithm/controller.h) | PID 参数、实例和生命周期接口 | 实例存储由调用模块提供，PID 提供校验、初始化／重置结果和只读状态；不采用统一动态分配实例池。 |
| 118 | [modules/algorithm/algorithm.md](../../../modules/algorithm/algorithm.md) | 公共算法模块和滤波使用说明 | 本轮加入一阶低通、有界滑动平均、二阶低通和陷波滤波，按实例配置，验证稳定性与延迟，不延长源有效期。 |
| 119 | [modules/algorithm/crc8.c](../../../modules/algorithm/crc8.c) | CRC-8 软件计算 | 常用算法使用只读查表，并保留独立逐位实现和对照验证，异常路径无需运行期初始化。 |
| 120 | [modules/algorithm/crc8.h](../../../modules/algorithm/crc8.h) | CRC-8 和公共校验入口 | 同时提供整块与调用者独立上下文的开始／更新／结束增量计算，分块不影响结果。 |
| 121 | [modules/algorithm/crc16.c](../../../modules/algorithm/crc16.c) | CRC-16 软件计算 | 本轮统一 CRC-8／16／32，验证既有协议及参数／崩溃记录兼容性；异常记录保留独立最小入口。 |
| 122 | [modules/algorithm/crc16.h](../../../modules/algorithm/crc16.h) | CRC 参数、上下文和结果接口 | 计算状态与 CRC 数值分开返回，校验参数、算法配置和上下文；计算成功与帧匹配分开判断。 |
| 123 | [modules/algorithm/kalman_filter.c](../../../modules/algorithm/kalman_filter.c) | 通用卡尔曼预测和量测校正 | 仅校正失败而预测合法时，可配置有限预测；超过时间／不确定度限制无效，保留最后有效量测时间。 |
| 124 | [modules/algorithm/kalman_filter.h](../../../modules/algorithm/kalman_filter.h) | 滤波维度、矩阵和工作区接口 | 调用模块提供工作区，算法报告所需大小并检查容量；启动固定状态维度和最大量测容量，运行不动态分配。 |
| 125 | [modules/algorithm/QuaternionEKF.c](../../../modules/algorithm/QuaternionEKF.c) | 四元数姿态 EKF 和零偏估计 | 本轮加入量测质量门控、有限预测以及基于安装几何和可信运动反馈的加速度补偿；缺条件时按门控／有限预测处理。 |
| 126 | [modules/algorithm/QuaternionEKF.h](../../../modules/algorithm/QuaternionEKF.h) | 姿态 EKF 状态和接口 | 按配置支持多实例，各自独立状态、工作区和坐标系；外置设备已提供合格姿态时可直接适配。 |
| 127 | [modules/algorithm/RLS.c](../../../modules/algorithm/RLS.c) | 递推最小二乘参数辨识 | 本轮修复数学实现及数值／初始化失败处理，接入可配置功率模型在线辨识；需可信功率测量与时间匹配，否则固定模型。 |
| 128 | [modules/algorithm/RLS.h](../../../modules/algorithm/RLS.h) | 辨识参数、质量和结果接口 | 通过质量检查后自动逐步应用，变化速率配置；异常回退已验证固定模型，候选与生效模型分别查询。 |
| 129 | [modules/algorithm/user_lib.c](../../../modules/algorithm/user_lib.c) | 角度、向量、数值和其他通用工具实现 | 内部校验非法输入，计算有界且明确返回状态，失败不改变原有效结果。 |
| 130 | [modules/algorithm/user_lib.h](../../../modules/algorithm/user_lib.h) | 通用数值工具声明及依赖 | 按角度／向量、滤波、矩阵等职责分类，按需引用，纯数值计算移除 HAL／RTOS 依赖。 |

### 131–139：BMI088 与姿态服务

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 131 | [modules/BMI088/bmi088.c](../../../modules/BMI088/bmi088.c) | 板载 IMU 初始化、采样和标定驱动 | 本轮同时支持周期阻塞与 DRDY 中断触发的任务异步 SPI 采样，配置选择；中断只记事件／时间，任务通过有界 SPI 事务读取。 |
| 132 | [modules/BMI088/bmi088.h](../../../modules/BMI088/bmi088.h) | BMI088 实例、配置、诊断与标定接口 | 初始化诊断、标定请求／状态和持久记录按实例管理，用稳定传感器标识区分，多颗 BMI088 独立采样与标定。 |
| 133 | [modules/BMI088/bmi088_data.h](../../../modules/BMI088/bmi088_data.h) | 公共 IMU 整组样本和新鲜度定义 | 用户选择陀螺仪、加速度、温度全部成功后才交付整组样本，使用统一时间与有效性；不拆为独立有效分量接口。 |
| 134 | [modules/BMI088/bmi088_regNdef.h](../../../modules/BMI088/bmi088_regNdef.h) | 寄存器、采样档位和换算常量 | 采样率、带宽、量程按车型选芯片支持档位；启动校验写入并读回，换算匹配实际量程，可查询实际配置；修改后重启。 |
| 135 | [modules/BMI088/bmi088_temperature.h](../../../modules/BMI088/bmi088_temperature.h) | 11 位有符号温度的独立换算 | 加入共享温度质量检查与可选突变检测，范围和判据配置；解码保持纯换算，温度不可信拒绝整组样本。 |
| 136 | [modules/BMI088/bmi088.md](../../../modules/BMI088/bmi088.md) | BMI088 接入、异步流程和标定说明 | 无可信记录时支持配置选择启动有限自动静止标定或等待人工维护标定；相关功能失能，通过静止判据且有总时限，取得可信标定前依赖功能无许可。 |
| 137 | [modules/imu/ins_task.c](../../../modules/imu/ins_task.c) | IMU 采样消费、姿态解算和结果发布 | 使用有界整组样本队列，按原始采样顺序处理，每个真实新样本仅融合一次，以采样时间差算 dt；限每轮预算，丢弃过期积压并报告缺口。 |
| 138 | [modules/imu/ins_task.h](../../../modules/imu/ins_task.h) | 姿态结果与坐标变换接口 | 发布一致只读姿态快照和参考版本；支持运行中姿态重建／换源，版本改变先撤销旧基准命令，重新对齐目标和反馈后恢复。 |
| 139 | [modules/imu/ins_task.md](../../../modules/imu/ins_task.md) | 姿态模块移植与使用说明 | 用户新决定：本轮接入 Fusion，作为配置可选后端，与 QuaternionEKF 共用姿态接口；适配单位、坐标、时间、运动补偿和恢复规则，更新配置与验收说明。 |


## 审阅主题与正式编号

此表记录用户的登记选择与编号关系，不复制 TODO 中的分诊或完成状态。

| 主题 | 内容 | 登记指向 |
|---|---|---|
| R01 | 车型配置与编译选择 | [MC02-0054](../../../.Doc/TODO.md)、[APP-10](../../../.Doc/TODO.md) |
| R02 | 独立任务与分阶段启动 | [MC02-0055](../../../.Doc/TODO.md)、[R25-12](../../../.Doc/TODO.md) |
| R03 | 模式仲裁与分功能运行许可 | [MC02-0056](../../../.Doc/TODO.md)、[MC02-0003](../../../.Doc/TODO.md)、[R25-06](../../../.Doc/TODO.md) |
| R04 | 通用底盘接口与运动反馈 | [MC02-0057](../../../.Doc/TODO.md)、[APP-10](../../../.Doc/TODO.md)、[PARAM-03](../../../.Doc/TODO.md) |
| R05 | 哨兵三轴搜索、跟踪与自旋协同 | [MC02-0058](../../../.Doc/TODO.md) |
| R06 | 发射就绪、热量与卡弹恢复 | [MC02-0059](../../../.Doc/TODO.md)、[MC02-0027](../../../.Doc/TODO.md)、[R25-15](../../../.Doc/TODO.md) |
| R07 | 经典 CAN 发送调度与恢复 | [MC02-0060](../../../.Doc/TODO.md)、[MC02-0047](../../../.Doc/TODO.md) |
| R08 | UART 与 USB 业务接口 | [MC02-0061](../../../.Doc/TODO.md)、[MC02-0062](../../../.Doc/TODO.md)、[BSPUART-03U1](../../../.Doc/TODO.md) |
| R09 | SPI／I²C 有界事务 | 用户本次暂缓新增，历史任务保留 |
| R10 | 板级资源、数字输入与 PWM | 用户本次暂缓新增，历史任务保留 |
| R11 | 单调时基与任务时序观测 | 用户本次暂缓新增，历史任务保留 |
| R12 | ADC 通道配置与标定 | 用户本次暂缓新增，历史任务保留 |
| R13 | H723 Flash 与参数事务 | [MC02-0063](../../../.Doc/TODO.md)、[MC02-0064](../../../.Doc/TODO.md)、[PARAM-03](../../../.Doc/TODO.md) |
| R14 | 整车健康与异步日志 | [MC02-0065](../../../.Doc/TODO.md)、[MC02-0066](../../../.Doc/TODO.md)、[MC02-0004](../../../.Doc/TODO.md) |
| R15 | 有限复位恢复与崩溃归档 | [MC02-0067](../../../.Doc/TODO.md)、[MC02-0068](../../../.Doc/TODO.md)、[LOG-06](../../../.Doc/TODO.md) |
| R16 | 任务监控与完成进度 | [MC02-0069](../../../.Doc/TODO.md) |
| R17 | 消息、事件与来源监控 | [MC02-0070](../../../.Doc/TODO.md)、[MC02-0071](../../../.Doc/TODO.md)、[MC02-0012](../../../.Doc/TODO.md)、[R25-14](../../../.Doc/TODO.md) |
| R18 | 通用电机与 DJI 反馈 | [MC02-0072](../../../.Doc/TODO.md)、[MC02-0017](../../../.Doc/TODO.md) |
| R19 | DM／HT 模式与共用 MIT 核心 | [MC02-0073](../../../.Doc/TODO.md)、[MOTOR-02](../../../.Doc/TODO.md)、[MOTOR-03](../../../.Doc/TODO.md) |
| R20 | LK 双控制方式与设备参数 | [MC02-0074](../../../.Doc/TODO.md)、[MOTOR-01](../../../.Doc/TODO.md)、[MOTOR-04](../../../.Doc/TODO.md)、[MOTOR-05](../../../.Doc/TODO.md)、[MC02-0018](../../../.Doc/TODO.md) |
| R21 | CyberGear 驱动 | [MC02-0075](../../../.Doc/TODO.md) |
| R22 | 舵机与有界机械回零 | 用户本次暂缓新增，历史任务保留 |
| R23 | PID 和数值工具 | [MC02-0015](../../../.Doc/TODO.md)、[MC02-0076](../../../.Doc/TODO.md)、[BSPMISC-02](../../../.Doc/TODO.md)、[BSPMISC-03](../../../.Doc/TODO.md) |
| R24 | 公共滤波与卡尔曼工作区 | [MC02-0077](../../../.Doc/TODO.md)、[MC02-0078](../../../.Doc/TODO.md) |
| R25 | 统一软件 CRC-8／16／32 | [MC02-0079](../../../.Doc/TODO.md) |
| R26 | 功率模型在线辨识 | [MC02-0080](../../../.Doc/TODO.md)、[MC02-0081](../../../.Doc/TODO.md) |
| R27 | 多 BMI088 采样与标定 | 用户本次暂缓新增，历史任务保留 |
| R28 | 多实例姿态服务与 Fusion 适配 | [MC02-0005](../../../.Doc/TODO.md)、[MC02-0082](../../../.Doc/TODO.md)、[MC02-0083](../../../.Doc/TODO.md) |
| R29 | 板载 RGB 与模块接入资料 | [MC02-0025](../../../.Doc/TODO.md)、[MC02-0026](../../../.Doc/TODO.md)、[MC02-0084](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |

## 融入参考项目的方式

| 参考来源 | 本次可采用的形式 | 范围说明 |
|---|---|---|
| [xioTechnologies/Fusion](https://github.com/xioTechnologies/Fusion) | 本轮适配为可配置姿态后端 | C 算法库，采用 [MIT 许可](https://github.com/xioTechnologies/Fusion/blob/main/LICENSE.md)。与 QuaternionEKF 共用采样、安装／运动补偿、姿态快照和业务接口；适配单位、坐标和恢复行为，并做结果及耗时验证。 |
| 上级 librm | 借鉴 CAN 优先级／期限调度、电机实例配置与能力、显式 PID dt 等设计 | 以本工程的 C 接口适配思路，不表示整体导入 C++ 库。CAN 调度思路已同意纳入方案。 |
| 上级 StandardRobot++ | 五类底盘接口、全向轮和轮腿方案的实现参考 | 具体机械参数、轮组方向、控制模型和许可证仍需按选定内容核对；不把参考车的 LQR／VMC 参数当成本车参数。 |
| 上级 PB 哨兵／导航等项目 | 参考自主阶段与上位机导航的职责划分 | 下位机输出速度／可选本地里程计，上位机负责场地定位与导航；其历史比赛规则常量不能直接套到当前比赛。 |
| [dmBots/motor-sdk](https://github.com/dmBots/motor-sdk) 及本工程附带厂商资料 | 核对 MIT 模式、量程、设备命令及反馈格式 | 仅使用匹配实际型号／固件的协议内容；DM 与 HT 共用编码机制，设备差异保留。LK 资料存在版本差异，仍需实际设备确认。 |

RoboMaster 论坛收藏页面此前因浏览器不可用、页面访问未成功而未读取；本汇总没有把其中未看见的项目列为已查阅或已同意融合的来源。

## 尚需确认的实施依据

这些内容不妨碍保留方案，但决定相应项目是否可以直接实施、采用什么验收条件。

- 哨兵实际轴距、轮距、轮径、轮子安装角、减速比、电机类型与 ID／总线、机械方向、各轴零位和范围，以及各 PID 与输出限制。
- 小 yaw 范围、三轴扫描和局部重捕获参数，实际底盘转速来源，自旋正弦参数、移动降速以及云台饱和协调阈值。
- 额外底盘 IMU／里程计的实际设备；IMU 安装旋转与位置几何、用于加速度补偿的反馈及时间对齐条件。
- 整组 IMU 样本统一时间戳的严格定义和允许采样时间差；队列容量、每轮预算、异常时间间隔、有限预测、姿态恢复和参考版本处理的具体参数。
- Fusion 适配版本和配置；量测质量与恢复行为必须符合已经讨论的公共姿态契约，需根据生产数据和实际耗时比较结果，不能预先认定优于现有 EKF。
- CyberGear 具体型号／固件及官方协议，LK／DM／HT 实际协议版本与量程，总线舵机厂商／型号／协议。DR 与步进尚无实际目标。
- 五类底盘除首台四轮全向轮外，本轮要实际完成哪些构型；半舵目前仅预留，轮腿控制模型和机械对象待细化。
- 各任务／设备的周期、优先级、截止时间、容量、重试／恢复次数、稳定清零时长以及 RAM、CPU、总线带宽预算。
- 崩溃持久归档的介质与分区：当前内部 Flash 布局已有程序和参数分区，不能把未声明空间当可用归档区。
- 板级实际引脚／外设冲突与硬件版本、RGB 灯珠及波形、ADC 量程和标定误差的仪器证据。
- RLS 功率辨识的真实测量来源、特征模型、输入时间匹配、质量判据与验证过的固定回退模型。

## 明确保留为后续的内容

- 较长 CAN FD 帧、10 位 I²C 地址、硬件 CRC 加速、多灯 RGB、DM 其他控制模式。
- 主机模拟／回放的通用模块调试平台；本轮先落实裸板独立调试。
- DR、步进具体实现与新电机接入模板；半舵具体机械模型。


## 记录如何使用

实施前按[范围规格](../specs/robomaster-framework-evolution.md)确认所选主题、暂缓接口和必要信息，在 [TODO](../../../.Doc/TODO.md)更新正式状态。不得把本文件中的全部讨论自动当作用户批准的实施范围，也不能用阅读源码或写出讨论记录代替生产接口和真实硬件验收。
