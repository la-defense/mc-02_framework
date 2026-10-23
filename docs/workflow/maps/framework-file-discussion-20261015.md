# MC-02 通用框架逐文件讨论记录

MC02-0076将原user_lib拆为标量／角度、三维向量和采样平均入口。下表原文件名及讨论编号保留历史含义，链接转到当前文件；当前用法见[算法入口](../../../modules/algorithm/algorithm.md)。

MC02-0079迁移后，六个旧CRC文件的链接指向当前统一入口；原文件名、行号、讨论编号和检查结论保留为当时依据，当前用法见[教学20](../../学习笔记/20-SP固定帧与CRC校验.md)。

讨论汇总与登记选择日期：2026-10-15～16（Asia/Shanghai）。本文保留文件用途和讨论依据，不维护任务进度；唯一任务状态见 [TODO](../../../.Doc/TODO.md)，本轮实施范围和验收设计见[范围规格](../specs/robomaster-framework-evolution.md)。

用户最终批准 R01–R08、R13–R21、R23–R26、R28–R29 共 23 个主题；R09、R10、R11、R12、R22、R27 本次不新增。已有任务不因暂缓被删除。下方“讨论结论”是讨论快照，不代表每行都进入本轮实施；具体以范围规格和 TODO 为准。1–218 为文件讨论编号，R01–R29 为主题标签，都不是正式任务 ID。追加的第140–145项安全告警结论经用户整批批准，合并已有 MC02-0084／MC02-0085，不新建重复任务。

## 最近确认及已覆盖的旧方案

- 2026-10-16登记：B2–B9、S1–S8全部获选；本次只改TODO及配套文档。新增0086–0104，已选原任务更新，六个暂缓主题不变。
- 新版上下位机瞄准／导航各自有效；首份合法新命令即可恢复对应业务，无预先握手。只有延迟补偿本身要求对时有效。
- CMake唯一构建，平台与OS通过本工程C接口隔离，首后端H723＋FreeRTOS；亚毫秒由平台定时唤醒任务，CubeMX本轮人工核对。
- 正式自主人工急停需明确解除，随后有效条件自动恢复无需再使能；持续供电的MCU复位不清急停。首台未温稳不单独阻止可信且无温控故障的运行。

- 第140–145项追加：持续表达最高优先级故障，清除后显示下一项；无更高优先级告警时显示整车状态灯效；灯效支持有界分段及常亮／闪烁／呼吸。灯声共用告警起点、节奏独立；可按车型配置允许等级和最长时限临时静音，静音不清故障或改变许可，灯光继续表达。
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
modules/        安全管理、任务监控、消息、来源监控、电机、算法、BMI088、姿态服务与告警
```

当前业务范围为 application／bsp／modules 中218个 `.c/.h/.md` 文件，第1–218项已完成本轮共同讨论。另有6份配套PDF／图片；构建、Core、USB_DEVICE、测试、工具、文档、CI、第三方及独立资料按支撑分组补记。覆盖方法与限制见下文，功能实现状态仍只在TODO。

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
| 36 | [usb_transport.c（原bsp/usb/bsp_usb.c）](../../../bsp/usb/usb_transport.c) | USB CDC 业务通信封装 | 原讨论意图保留；MC02-0062迁移为复制收发及有限恢复。USB用于上下位机业务；独立诊断串口由0004接入。 |
| 37 | [usb_transport.h（原bsp/usb/bsp_usb.h）](../../../bsp/usb/usb_transport.h) | USB 收发及链路接口 | 正式自主中 USB 失效时撤销旧移动、瞄准和开火命令，停止平移、禁发射，继续原地自旋与本地三轴索敌；恢复只使用新有效命令。原讨论意图不表示本轮已实现全部新版业务。 |
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
| 119 | [modules/algorithm/crc8.c（现统一CRC入口）](../../../modules/algorithm/crc.c) | CRC-8 软件计算 | 常用算法使用只读查表，并保留独立逐位实现和对照验证，异常路径无需运行期初始化。 |
| 120 | [modules/algorithm/crc8.h（现统一CRC入口）](../../../modules/algorithm/crc.h) | CRC-8 和公共校验入口 | 同时提供整块与调用者独立上下文的开始／更新／结束增量计算，分块不影响结果。 |
| 121 | [modules/algorithm/crc16.c（现统一CRC入口）](../../../modules/algorithm/crc.c) | CRC-16 软件计算 | 本轮统一 CRC-8／16／32，验证既有协议及参数／崩溃记录兼容性；异常记录保留独立最小入口。 |
| 122 | [modules/algorithm/crc16.h（现统一CRC入口）](../../../modules/algorithm/crc.h) | CRC 参数、上下文和结果接口 | 计算状态与 CRC 数值分开返回，校验参数、算法配置和上下文；计算成功与帧匹配分开判断。 |
| 123 | [modules/algorithm/kalman_filter.c](../../../modules/algorithm/kalman_filter.c) | 通用卡尔曼预测和量测校正 | 仅校正失败而预测合法时，可配置有限预测；超过时间／不确定度限制无效，保留最后有效量测时间。 |
| 124 | [modules/algorithm/kalman_filter.h](../../../modules/algorithm/kalman_filter.h) | 滤波维度、矩阵和工作区接口 | 调用模块提供工作区，算法报告所需大小并检查容量；启动固定状态维度和最大量测容量，运行不动态分配。 |
| 125 | [modules/algorithm/QuaternionEKF.c](../../../modules/algorithm/QuaternionEKF.c) | 四元数姿态 EKF 和零偏估计 | 本轮加入量测质量门控、有限预测以及基于安装几何和可信运动反馈的加速度补偿；缺条件时按门控／有限预测处理。 |
| 126 | [modules/algorithm/QuaternionEKF.h](../../../modules/algorithm/QuaternionEKF.h) | 姿态 EKF 状态和接口 | 按配置支持多实例，各自独立状态、工作区和坐标系；外置设备已提供合格姿态时可直接适配。 |
| 127 | [modules/algorithm/RLS.c](../../../modules/algorithm/RLS.c) | 递推最小二乘参数辨识 | 本轮修复数学实现及数值／初始化失败处理，接入可配置功率模型在线辨识；需可信功率测量与时间匹配，否则固定模型。 |
| 128 | [modules/algorithm/RLS.h](../../../modules/algorithm/RLS.h) | 辨识参数、质量和结果接口 | 通过质量检查后自动逐步应用，变化速率配置；异常回退已验证固定模型，候选与生效模型分别查询。 |
| 129 | [modules/algorithm/user_lib.c](../../../modules/algorithm/math_utils.c) | 角度、向量、数值和其他通用工具实现 | 内部校验非法输入，计算有界且明确返回状态，失败不改变原有效结果。 |
| 130 | [modules/algorithm/user_lib.h](../../../modules/algorithm/math_utils.h) | 通用数值工具声明及依赖 | 按角度／向量、滤波、矩阵等职责分类，按需引用，纯数值计算移除 HAL／RTOS 依赖。 |

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


### 140–145：安全告警

| 编号 | 文件 | 用途 | 讨论结论 |
|---:|---|---|---|
| 140 | [modules/alarm/alarm_def.h](../../../modules/alarm/alarm_def.h) | 告警等级及开关公共定义 | RGB 与蜂鸣器共用仲裁，持续表达最高优先级故障，清除后显示下一项；完整故障集仍在健康快照中查询。 |
| 141 | [modules/alarm/led.c](../../../modules/alarm/led.c) | 板载单灯模式执行 | 无更高优先级告警时，按初始化、未使能、标定、已使能等整车安全状态显示可配置灯效，替代统一粉色呼吸。 |
| 142 | [modules/alarm/led.h](../../../modules/alarm/led.h) | 灯效配置与输出接口 | 加入有界分段灯效，配置颜色、各段时长与重复；段数和单轮总时长有上限，按实际时间推进，保留常亮、普通闪烁和呼吸。 |
| 143 | [modules/alarm/buzzer.c](../../../modules/alarm/buzzer.c) | 蜂鸣器 PWM 音调与优先级执行 | 同一告警的灯光与声音共用起点，分别配置节奏；告警切换撤销旧模式，任务中有界播放。 |
| 144 | [modules/alarm/buzzer.h](../../../modules/alarm/buzzer.h) | 蜂鸣器实例、配置及控制接口 | 支持受限临时静音，可静音等级及最长时限按车型配置；只影响声音，不清故障、不改许可，灯光继续表达。 |
| 145 | [modules/alarm/buzzer.md](../../../modules/alarm/buzzer.md) | 蜂鸣器接入与使用示例 | 沿用前述结论修正文档函数名，补齐统一告警配置、仲裁、灯声节奏、静音边界及验证说明；本文件无额外功能取舍。 |

本批源码核对依据：当前 LED 初始化／注册／任务仅有定义和声明，尚未接入整车任务；蜂鸣器已有注册参数检查、资源失败返回及初始化回滚，不重复登记为缺失能力。`application/robot_task.h` 当前将蜂鸣器初始化失败上报全局初始化故障并锁存关断，来自旧策略，需在 MC02-0084 中适配已确认的辅助降级规则，不将其视为旧 R25-10 修复回归。`OCTAVE_8` 可注册但播放分支仅覆盖前七项；文档使用的 `AlarmSetStatus` 与实际 `BuzzerSetStatus` 不一致。这些依据不代表已完成固件修复。


### 146–158：上下位机及板间通信

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 146 | [modules/master_machine/master_process.c](../../../modules/master_machine/master_process.c) | 上下位机收发、命令快照和链路状态组织 | 保留现有SP29/43固定帧兼容选项，新增带版本的上下位机业务协议，由车型配置选择；本轮新增协议能力在下位机实施，上位机适配按第148项另行安排。 | [MC02-0089](../../../.Doc/TODO.md)、[MC02-0062](../../../.Doc/TODO.md) |
| 147 | [modules/master_machine/master_process.h](../../../modules/master_machine/master_process.h) | 上下位机实例、状态与业务接口声明 | 新协议下瞄准与导航移动分别判定有效性；正式自主且核心反馈/裁判正常，瞄准消息过期但导航移动仍有效时继续平移，禁止发射，继续底盘自旋与云台本地索敌；USB全链路失效仍撤销移动并停止平移。 | [MC02-0056](../../../.Doc/TODO.md)、[MC02-0089](../../../.Doc/TODO.md) |
| 148 | [modules/master_machine/master_process.md](../../../modules/master_machine/master_process.md) | 上下位机模块和协议接入说明 | 本轮先完成MC-02下位机的新协议实现、协议说明和校验样例，上位机代码适配另行安排；当前SP兼容保留，不将新业务协议视为上位机已支持。 | [MC02-0089](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 149 | [modules/master_machine/vision_protocol.c](../../../modules/master_machine/vision_protocol.c) | SP 帧查找、编解码及校验 | 沿用已批准的协议层有界拼帧/拆帧及统一CRC规则：补跨接收块残帧和同块多帧处理，保留SP29/43线格式及CRC结果兼容；CRC错误不续期，按话题既定最新快照/事件语义交付。已有帧头搜索、CRC和坏帧后重同步测试保留，不重复作为缺失能力登记；无新增策略选择。 | [MC02-0089](../../../.Doc/TODO.md)、[MC02-0079](../../../.Doc/TODO.md) |
| 150 | [modules/master_machine/vision_protocol.h](../../../modules/master_machine/vision_protocol.h) | SP 字段、帧与协议接口声明 | 新版本协议本轮实现下位机对时交互、时钟偏差和同步有效状态，并完成对应测试；旧SP29/43保持兼容，上位机代码适配依第148项另行安排。 | [MC02-0090](../../../.Doc/TODO.md) |
| 151 | [modules/master_machine/vision_control.c](../../../modules/master_machine/vision_control.c) | 视觉目标校验、单位转换及控制输入适配 | 本轮接通视觉目标角速度与角加速度前馈；两项分别按车型配置启用、系数和限幅，匹配实际轴系与电机物理控制量；不能仅保留字段或把角加速度直接当电流/力矩。 | [MC02-0091](../../../.Doc/TODO.md) |
| 152 | [modules/master_machine/vision_control.h](../../../modules/master_machine/vision_control.h) | 视觉控制输入、限制与输出契约 | 新协议对时有效、视觉明确目标参考时刻且命令未过期时，允许利用目标角速度/角加速度做有界延迟补偿；启用和补偿时长上限由车型配置，只补已有指令传输/排队延迟，不代替视觉弹道/击发决策、不延长原命令有效期；旧SP无源参考时间不虚构同步。 | [MC02-0091](../../../.Doc/TODO.md) |
| 153 | [docs/history/seasky/seasky_protocol.c](../../../docs/history/seasky/seasky_protocol.c) | 历史 Seasky 数据帧编解码 | 旧Seasky协议仅留源码与资料参考，移出固件构建；不重构为第三种可选通信协议，本轮维持SP兼容与新版业务协议。 | [MC02-0008](../../../.Doc/TODO.md)、[MC02-0054](../../../.Doc/TODO.md) |
| 154 | [docs/history/seasky/seasky_protocol.h](../../../docs/history/seasky/seasky_protocol.h) | 历史 Seasky 帧结构和接口声明 | 沿用第153项，仅作为旧Seasky参考接口保留，不成为活动协议接口；活动模块不依赖此头文件，不新增第三种兼容能力。 | [MC02-0008](../../../.Doc/TODO.md) |
| 155 | [docs/history/seasky/湖南大学RoboMaster电控组通信协议.md](../../../docs/history/seasky/湖南大学RoboMaster电控组通信协议.md) | Seasky 原始协议、署名和接入资料 | 沿用第153项，保留作者署名和旧Seasky格式作为参考，明确与当前SP及新版业务协议的区别；F4硬件CRC和更多数据类型是原文设想，不自动进入本轮。 | [MC02-0008](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 156 | [modules/can_comm/can_comm.c](../../../modules/can_comm/can_comm.c) | CAN 板间消息分片、重组和快照交付 | 本轮采用带协议版本、消息序号和分片编号的新板间CAN格式，两端同步升级，不额外维护旧格式兼容；仍使用最多8字节的经典CAN帧。 | [MC02-0092](../../../.Doc/TODO.md) |
| 157 | [modules/can_comm/can_comm.h](../../../modules/can_comm/can_comm.h) | 板间通信配置、实例及收发接口 | 按车型配置显式桥接跨板话题，CANComm作为消息中心的跨板传输适配，业务统一使用发布/订阅；保留消息类型、源有效期与失败结果，只传输明确配置的话题，不由业务维护单板/双板收发分支。 | [MC02-0092](../../../.Doc/TODO.md)、[MC02-0070](../../../.Doc/TODO.md) |
| 158 | [modules/can_comm/can_comm.md](../../../modules/can_comm/can_comm.md) | CAN 板间通信接入与格式说明 | 本轮加入CAN板间对时，映射源时间并报告同步有效状态；跨板转发保留源有效期，不直接比较不同板本地计时，不因重组/转发续期；通过现有计时的局部适配完成，不自动恢复R11全局时基重构。文档与新分片/话题桥接/实际任务解析路径及验证对应。 | [MC02-0093](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |

### 159–168：裁判系统

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 159 | [modules/referee/rm_referee.c](../../../modules/referee/rm_referee.c) | 裁判串口帧解析及命令分发 | 本轮只维护一套明确锁定的官方裁判协议版本，今后协议升级时明确迁移；不维护多赛季可选适配器，版本已由第161项锁定为2026 V2.0.0（2026-06-26）。已有CRC/长度/初始化保护保留，接收合法性、原始接收时间、跨块拼帧与数据分组有效性按已定公共契约修正。 | [MC02-0094](../../../.Doc/TODO.md) |
| 160 | [modules/referee/rm_referee.h](../../../modules/referee/rm_referee.h) | 裁判实例、状态结构与访问接口 | 本轮通过新版本业务协议加入面向上位机决策的结构化裁判摘要；字段和周期按车型配置，保留各数据组源时间与有效性；实际字段按锁定官方版本，本轮完成下位机输出，上位机接入仍依第148项另行安排。 | [MC02-0089](../../../.Doc/TODO.md)、[MC02-0094](../../../.Doc/TODO.md) |
| 161 | [modules/referee/referee_protocol.h](../../../modules/referee/referee_protocol.h) | 裁判命令码、数据字段和长度定义 | 锁定资料目录中的官方RoboMaster2026高校系列赛通信协议V2.0.0（2026-06-26）为本轮唯一基线；逐项核对命令、长度、字段、位域和支持范围，保留已正确实现部分，不凭旧注释年份认定全部过时。后续升级明确迁移，不维护多个赛季适配器。 | [MC02-0094](../../../.Doc/TODO.md) |
| 162 | [modules/referee/referee_task.c](../../../modules/referee/referee_task.c) | 裁判业务任务、机器人交互与 UI 组织 | 本轮先补齐可配置的裁判通道机器人间通用收发接口，内容ID/对端等按配置并遵守已锁定官方协议身份/长度/频率约束，具体队友状态/敌情/任务等协同消息以后定义；复用MC02-0022，不新增重复目标。裁判接收与可选UI依已定规则拆分，展示用真实状态并隔离演示数据。 | [MC02-0022](../../../.Doc/TODO.md)、[MC02-0094](../../../.Doc/TODO.md) |
| 163 | [modules/referee/referee_task.h](../../../modules/referee/referee_task.h) | 裁判任务及交互接口声明 | 本轮机器人间通用收发仅报告接受、本机发送完成、失败/超时；接收方确认和重试协议以后随具体协同消息定义，本机发送不代表对方收到，对方收到也不代表业务执行。不在本轮新增通用应用层ACK协议。 | [MC02-0022](../../../.Doc/TODO.md) |
| 164 | [modules/referee/referee_UI.c](../../../modules/referee/referee_UI.c) | 裁判客户端 UI 图形、字符串组帧与发送 | 本轮采用增量更新＋可配置低频分批重绘，支持检测到链路恢复时和人工触发重绘；周期及处理量有界，与机器人间通信共享发送预算。字符串/计数/边界及发送序号等问题复用REF-04/REF-03修复，不把本机发送成功当客户端已显示。 | [MC02-0095](../../../.Doc/TODO.md)、[REF-03](../../../.Doc/TODO.md)、[REF-04](../../../.Doc/TODO.md) |
| 165 | [modules/referee/referee_UI.h](../../../modules/referee/referee_UI.h) | 裁判 UI 图形结构、命令及绘图接口 | 本轮加入按车型配置的UI布局和数据绑定：配置位置、样式和只读数据来源，由UI模块统一绘制并处理增量更新/重绘；基础绘图仍需校验和明确结果，不要求各车型自行维护整套布局发送代码。 | [MC02-0095](../../../.Doc/TODO.md) |
| 166 | [modules/referee/crc_ref.c（现统一CRC入口）](../../../modules/algorithm/crc.c) | 裁判专用 CRC8／CRC16 软件实现 | 沿用已批准R25统一软件CRC方案，裁判CRC8/CRC16使用独立不可变算法配置并保留线上兼容；不能直接替换为现有crc_8/crc_16（多项式/反射/初值不同）。保留校验向量和协议完整帧测试，复用MC02-0079；无新增范围选择。 | [MC02-0079](../../../.Doc/TODO.md) |
| 167 | [modules/referee/crc_ref.h（现统一CRC入口）](../../../modules/algorithm/crc.h) | 裁判校验、追加与结果接口声明 | 沿用统一CRC契约：计算状态与数值分开、只读输入/明确长度和输出容量、有界追加及明确校验结果，算法配置不可变；协议封装声明自己的具体配置与字节序，不再暴露混淆的TRUE/FALSE及重复实现；迁移复用MC02-0079而非重复任务。 | [MC02-0079](../../../.Doc/TODO.md) |
| 168 | [modules/referee/referee.md](../../../modules/referee/referee.md) | 裁判、交互与 UI 接入说明 | 按第159–167项更新接入说明：已锁定2026V2.0.0并保留历史适配验收；裁判接收与可选UI分开、真实只读快照/分组源期限/上行摘要；官方共享发送限额下的通用机器人交互、本机结果语义、配置化UI及有界重绘，示例与实际函数一致。具体协同消息与对端ACK后续，复用MC02-0085/MC02-0022/REF-03/REF-04；无额外功能取舍。 | [MC02-0085](../../../.Doc/TODO.md)、[MC02-0094](../../../.Doc/TODO.md)、[MC02-0095](../../../.Doc/TODO.md) |

### 169–173：功率及超级电容

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 169 | [modules/power/power_manager.c](../../../modules/power/power_manager.c) | 底盘功率管理骨架：保存模型系数；反馈和RLS更新未实现，尚无业务调用。 | 裁判功率上限与底盘供电许可新鲜有效而缓冲能量过期时，以有效上限及配置保守余量继续限功率，停用依赖缓冲信息的额外功率；额度/供电许可失效不沿用该降级授权。复用已选固定模型回退及RLS质量约束。 | [MC02-0057](../../../.Doc/TODO.md)、[R25-15](../../../.Doc/TODO.md)、[MC02-0081](../../../.Doc/TODO.md) |
| 170 | [modules/power/power_manager.h](../../../modules/power/power_manager.h) | 声明功率模型系数、实例和控制接口；现接口混合全局与实例，含3508固定换算常量。 | 沿用统一电机物理量/能力声明、配置化模型及带时间有效性的快照；区分裁判预算、实测功率、候选/生效模型及限制结果，按实例管理，不将旧型号常量推广为通用参数。无新增产品取舍。 | [MC02-0080](../../../.Doc/TODO.md)、[MC02-0081](../../../.Doc/TODO.md) |
| 171 | [modules/super_cap/super_cap.c](../../../modules/super_cap/super_cap.c) | 超级电容CAN注册、反馈解码与控制发送。 | 尚未选型，具体电容设备协议适配与验证留待后续开发；不得认定现有6字节字段及ID适配实际硬件。继续保留车型配置可选、电容不可用按普通供电限制。 | [MISC-04](../../../.Doc/TODO.md)、[MC02-0057](../../../.Doc/TODO.md) |
| 172 | [modules/super_cap/super_cap.h](../../../modules/super_cap/super_cap.h) | 定义原始电容反馈、实例、CAN配置及注册/发送接口。 | 设备尚未选型，物理量/单位/能力与具体协议的接口适配留待后续；不将现有三项uint16原始字段视为已验证的通用物理反馈。沿用配置可选、有效性与普通供电回退约定。 | [MISC-04](../../../.Doc/TODO.md)、[MC02-0057](../../../.Doc/TODO.md) |
| 173 | [modules/super_cap/super_cap.md](../../../modules/super_cap/super_cap.md) | 超级电容驱动结构、初始化与发送示例说明。 | 记录实际接口差异与待选型边界；修正不存在的初始化字段，以及向固定8字节复制接口传入2字节变量的越界读取示例；H723接收解析已在任务中，不沿用文档中断直接解析描述。具体适配/实物验证不自动进入本轮。 | [MISC-04](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |

### 174–180：遥控与裸板诊断

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 174 | [modules/remote/remote_control.c](../../../modules/remote/remote_control.c) | DT7/DR16 DBUS解析、字段校验、遥控在线监测、独立键鼠心跳租约及一致输入快照。 | 本轮加入配置化摇杆中位/端点、死区与可选响应曲线，默认线性。保留已完成合法帧过滤、键鼠递增心跳/两帧全松开恢复及原子快照，按已确认模式授权和原始接收时间/异步恢复约定接入。 | [MC02-0096](../../../.Doc/TODO.md) |
| 175 | [modules/remote/remote_control.h](../../../modules/remote/remote_control.h) | 定义遥控摇杆/开关/键鼠类型、输入快照、在线状态与诊断查询。 | 复用一致只读快照、源时间及独立来源有效性；连续状态与离散事件沿用既定语义，原始DBUS保留诊断用途，业务映射不依赖驱动内部TEMP/LAST或uint8累计次数取模。第174项手感参数与参数校验接入；不附带新增其他遥控协议或双击手势。 | [MC02-0096](../../../.Doc/TODO.md)、[MSG-06](../../../.Doc/TODO.md) |
| 176 | [modules/remote/remote_safety.c](../../../modules/remote/remote_safety.c) | 识别遥控使能/失能/急停及解除手势，并提交安全请求。 | 正式自主发生人工急停后，操作者明确解除且核心/裁判/自主条件有效时无需另行使能，自动恢复；调试仍回SAFE。未明确解除前保持禁止；复用统一安全管理/立即禁止通道，保留既有调试验证记录。 | [MC02-0056](../../../.Doc/TODO.md) |
| 177 | [modules/remote/remote_safety.h](../../../modules/remote/remote_safety.h) | 声明安全操作手势状态及更新接口，传入当前时间、输入有效性和使能条件。 | 人工急停尚未明确解除时，持续供电的MCU复位需保留急停，不因已选有限自动恢复被绕过；正式自主明确解除后按176自动恢复，调试仍SAFE。本项不扩展为已确认断电持久化，也不在急停路径引入Flash擦写。 | [MC02-0067](../../../.Doc/TODO.md) |
| 178 | [modules/remote/remote_bench.c](../../../modules/remote/remote_bench.c) | bench_safe配置的裸板遥控接收及安全手势诊断任务。 | 保留编译配置约束、禁止READY、持续失能及执行器隔离；按已有裸板独立调试范围观察原始/归一化输入、手感参数效果、来源授权与异常。正式自主自动恢复不适用于此入口，不新增主机模拟回放平台。 | [MC02-0026](../../../.Doc/TODO.md) |
| 179 | [modules/remote/remote_bench.h](../../../modules/remote/remote_bench.h) | 声明裸板遥控诊断初始化及周期入口。 | 接口保持受诊断构建选择约束，复用只读快照/健康查询和已有MC02-0026；不得让正常构建误用诊断入口，也不得以独立调试接口绕过禁止READY。无额外产品取舍。 | [MC02-0026](../../../.Doc/TODO.md) |
| 180 | [modules/remote/remote.md](../../../modules/remote/remote.md) | DT7遥控/键鼠数据布局与操作映射说明。 | 修正鼠标按键/键盘位偏移等与实现不符说明和旧映射；按已确认统一安全操作、车型业务映射、手感配置、键鼠租约和正式/调试恢复差异更新。保留R25-03/05和MC02-0049已完成历史；通道参数与协议合法性分别校验，不能靠标定放宽非法帧。 | [MC02-0085](../../../.Doc/TODO.md)、[MC02-0096](../../../.Doc/TODO.md) |

### 181–190：温控及可选传感器

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 181 | [modules/imu/imu_heater.c](../../../modules/imu/imu_heater.c) | IMU加热控制、温度/供电质量检查、输出限制、故障锁存及维护长操作保护。 | 首台哨兵在姿态/标定和其他核心检查可信、无温控故障时允许未温稳使能；温稳只作提示，其他车型可配置要求等待。是否启用温控由配置决定，保留现有输出硬限制/输入质量/锁存/长操作及禁热保护，不重新打开已完成温控修复。 | [MC02-0056](../../../.Doc/TODO.md)、[MC02-0065](../../../.Doc/TODO.md) |
| 182 | [modules/imu/imu_heater.h](../../../modules/imu/imu_heater.h) | 温控状态、目标设置、强制关断、故障恢复与长操作保护接口。 | 沿用统一参数校验和只读健康快照，区分温度有效/加热启用与实际输出/温稳/具体故障；目标在已有硬限制内校验。第181项决定温稳默认仅提示；保留现有锁存恢复与新样本保护，不引入多IMU扩展。 | [MC02-0065](../../../.Doc/TODO.md)、[MC02-0064](../../../.Doc/TODO.md) |
| 183 | [modules/imu/BMI088Middleware.c](../../../modules/imu/BMI088Middleware.c) | 未被当前业务调用的旧BMI088固定GPIO、全局SPI逐字节HAL适配。 | 删除旧.c并清理构建/文档引用，配套旧.h与BMI088reg.h一并删除，Git历史保留来源；保留当前modules/BMI088生产路径和既有验证，不扩大R27。 | [MC02-0054](../../../.Doc/TODO.md) |
| 184 | [modules/imu/BMI088Middleware.h](../../../modules/imu/BMI088Middleware.h) | 旧BMI088中间层函数与全局SPI句柄声明。 | 随183删除，清理引用，保留Git来源；不再维护第二套BMI088接口。 | [MC02-0054](../../../.Doc/TODO.md) |
| 185 | [modules/imu/BMI088reg.h](../../../modules/imu/BMI088reg.h) | 未被当前采样路径引用的旧BMI088寄存器常量。 | 随183删除重复旧定义，保留现行bmi088_regNdef.h及既有固定档位；不把清理等同于恢复R27档位扩展。 | [MC02-0054](../../../.Doc/TODO.md) |
| 186 | [modules/ist8310/ist8310.c](../../../modules/ist8310/ist8310.c) | IST8310磁力计注册、配置、数据就绪与三轴磁场采样驱动。 | 用户指定先核验MC-02板载情况，若板载则完善；已据V1.1手册与V1.0完整原理图确认该版本板载BMI088六轴，无IST8310/其他磁力计，当前工程亦无接入配置或调用。本轮不因板载条件启动IST8310适配，保留外接扩展，未来实际选用前需补驱动可靠性与验收。 | [MC02-0051](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 187 | [modules/ist8310/ist8310.h](../../../modules/ist8310/ist8310.h) | 磁场原始反馈、I2C/复位/数据就绪配置及初始化接口声明。 | 随186不作为当前MC-02板载必需模块；保留外接扩展，未来接入先明确硬件/安装与协议，补带时间/有效性的只读磁场快照和错误结果，不默认取得姿态融合控制来源。 | [MC02-0085](../../../.Doc/TODO.md) |
| 188 | [modules/TFminiPlus/tfminiplus.c](../../../modules/TFminiPlus/tfminiplus.c) | TFminiPlus外接测距模块的I2C注册、取数命令及距离/信号强度解码。 | 用户选择暂缓实际接入，保留代码并明确未验证，待有设备需求再完善；已知9字节缓冲/11字节接收及注册/帧检查问题保留MISC-03、MC02-0051跟踪，不当作可用驱动或自动关闭任务。 | [MISC-03](../../../.Doc/TODO.md)、[MC02-0051](../../../.Doc/TODO.md) |
| 189 | [modules/TFminiPlus/tfminiplus.h](../../../modules/TFminiPlus/tfminiplus.h) | 声明测距实例、原始字段与注册/读距离接口。 | 随188暂缓实际接入，接口标明未验证；后续选用先按真实协议确定单位/质量/有效性/时间和错误结果，不将当前裸float返回视为可信物理测距。无本轮新驱动扩展。 | [MC02-0085](../../../.Doc/TODO.md) |
| 190 | [modules/TFminiPlus/tfminiplus.md](../../../modules/TFminiPlus/tfminiplus.md) | TFminiPlus通信实现及后续改造说明。 | 同步未启用/未验证及未来启用前置条件；保留实际协议、缓冲/帧校验、故障及任务时限待核对信息，不将文档中的中断改造设想表述成已实现。本轮不推进全局I2C平台。 | [MC02-0085](../../../.Doc/TODO.md) |

### 191–207：LCD、OLED及字体

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 191 | [modules/lcd/lcd.c](../../../modules/lcd/lcd.c) | 现有ST7789屏幕初始化、地址窗、逐字节SPI及传统图元/字符/图片绘制。 | 沿用已有配置/单一显示任务/边界校验与有界结果约定；初始化及写命令的等待与HAL失败向上反馈，显示仅按辅助模块降级，不以SPI返回成功冒充实物显示正常。保留关闭MC02-0046，不重启硬件排查。 | [MC02-0097](../../../.Doc/TODO.md) |
| 192 | [modules/lcd/lcd.h](../../../modules/lcd/lcd.h) | LCD硬件引脚、方向、可见区尺寸/偏移/边距、颜色及绘图接口声明。 | LCD自身屏幕参数与允许能力对齐车型配置；固定物理坐标与逻辑可见区明确，统一绘图调用约定。方向/边距实物验收继续原LCD-06/07记录，不扩大为暂缓的全板GPIO/资源平台或新增多屏适配。 | [MC02-0097](../../../.Doc/TODO.md)、[LCD-06](../../../.Doc/TODO.md)、[LCD-07](../../../.Doc/TODO.md) |
| 193 | [modules/lcd/lcd_buf.c](../../../modules/lcd/lcd_buf.c) | 使用静态行像素缓冲批量绘制色块、ASCII/中文及格式化数值，记录SPI和耗时诊断。 | 本轮数值/状态/故障页面为主，趋势曲线后续；完善有界字符串/字模索引/数值与绘图范围检查、统一失败反馈，复用现有行缓冲与任务预算，不引入完整GUI或全屏缓存的新需求。 | [MC02-0097](../../../.Doc/TODO.md) |
| 194 | [modules/lcd/lcd_buf.h](../../../modules/lcd/lcd_buf.h) | 声明行缓冲绘图、字符及数值显示接口和清屏耗时诊断。 | 衔接193有界长度/合法字模/数值/逻辑坐标与操作结果；显示任务拥有绘图/刷新，复用既有健康与任务预算；本轮不增加趋势曲线、GUI库或全屏缓存。 | [MC02-0097](../../../.Doc/TODO.md) |
| 195 | [modules/lcd/lcd_key_input.c](../../../modules/lcd/lcd_key_input.c) | 从有效按键档位生成按下/松开及持续时间事件，维护去抖状态。 | 保留无效ADC/非法键位取消手势与不产生伪释放；去抖/长按使用明确时间语义以适配配置周期，校验持续同键及切换行为。只处理输入，不直接执行标定或修改运行许可，不恢复R10通用数字输入平台。 | [MC02-0098](../../../.Doc/TODO.md) |
| 196 | [modules/lcd/lcd_key_input.h](../../../modules/lcd/lcd_key_input.h) | 定义五向键枚举、去抖状态与带持续时间的按键事件接口。 | 输入事件和维护授权分离，来源/时间/有效性遵守既定事件约定；保持现有LCD按键局部接口并复用参数/任务规则，不扩展未要求的双击/任意组合键系统。 | [MC02-0098](../../../.Doc/TODO.md) |
| 197 | [modules/lcd/lcd_task.c](../../../modules/lcd/lcd_task.c) | LCD初始化、按键操作分派、周期刷新、标定提示与状态/耗时统计任务。 | 用户允许运行状态进入IMU标定确认页；明确确认后由框架请求失能并验证既定静止/关热/维护条件，通过后由IMU任务标定。进入确认页本身不改变控制；请求/执行/结果分开反馈，不在显示任务执行采样标定。其余显示任务周期、预算及统计来源复用既定配置/健康规则。 | [MC02-0098](../../../.Doc/TODO.md)、[MC02-0056](../../../.Doc/TODO.md) |
| 198 | [modules/lcd/lcd_task.h](../../../modules/lcd/lcd_task.h) | 声明LCD独立任务入口。 | 复用按配置创建任务、固定启动周期及辅助模块监控；显示任务只消费只读快照与输入事件，维护由所有者任务执行，不将其他模块监控/恢复依赖于LCD存在。 | [MC02-0055](../../../.Doc/TODO.md) |
| 199 | [modules/lcd/lcd_ui.c](../../../modules/lcd/lcd_ui.c) | 整车/IMU/裁判功率/电机通信页面、值区缓存、冻结与标定提示。 | 本轮只读查询并加入197确定的标定确认流程；参数编辑走独立诊断串口，不新增本地参数编辑页或趋势曲线。接入已选统一健康/姿态/裁判分组快照及有效性，保留按行重复跳过，按实际有效功率来源修正reserved_3显示，绘图成功后再确认缓存。 | [MC02-0098](../../../.Doc/TODO.md) |
| 200 | [modules/lcd/lcd_ui.h](../../../modules/lcd/lcd_ui.h) | 页面数量与初始化、绘制、冻结、重绘缓存和标定提示接口。 | 新增严重故障时自动解除冻结并跳故障页，优先显示当前故障，使用统一safety/健康来源和优先级；页面/冻结状态单一所有者（复用LCD-14），清故障/解除冻结不产生维护请求或改变许可。 | [MC02-0098](../../../.Doc/TODO.md)、[LCD-14](../../../.Doc/TODO.md) |
| 201 | [modules/lcd/lcd_font_cn.c](../../../modules/lcd/lcd_font_cn.c) | 页面使用的16x16中文点阵常量，当前93字，每字32字节。 | 仅随已选UI标签按需补字，保留生成工具来源和可复现步骤，复用LCD-10已关闭不预扩字库的结论；静态核对枚举/生成器有效字符数及顺序一致，未执行字库生成或实物字形验证。 | [MC02-0102](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 202 | [modules/lcd/lcd_font_cn.h](../../../modules/lcd/lcd_font_cn.h) | 中文字符枚举、点阵声明及LCD常用标签的结束标记序列。 | 与生成器/点阵共同维护，校验标签全部引用有效枚举和有界终止；沿用201按需补字策略，不新增运行时中文编码转换。 | [MC02-0102](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 203 | [modules/lcd/lcdfont.h](../../../modules/lcd/lcdfont.h) | 12/16/24/32字号ASCII及旧中文索引点阵，供旧绘图与行缓冲路径读取。 | 按当前实际用到的字库和字号维护及裁剪构建资源；保留来源，不预扩未使用字模，不新增趋势/复杂图形库。本次静态核对四套ASCII均95字，初始化长度分别12/16/48/64，实物外观未测试。 | [MC02-0097](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 204 | [modules/oled/oled.c](../../../modules/oled/oled.c) | 128x64 OLED初始化、I2C字节发送、GRAM绘图/字符/格式化输出及全屏刷新。 | 用户选择暂缓实际接入，保留代码并明确未验证，待有OLED需求再完善；不新增本轮OLED显示后端或GUI移植，已知边界/错误结果由现有MISC-06等继续跟踪。 | [MISC-06](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 205 | [modules/oled/oled.h](../../../modules/oled/oled.h) | OLED固定尺寸、I2C地址、图元/字符/格式化/开关/刷新接口。 | 随204标明未接入/未验证，实际启用前确认屏幕型号/协议/接线、参数和安全接口；不把硬编码参数视为已经通用适配。 | [MISC-06](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| 206 | [modules/oled/oled.md](../../../modules/oled/oled.md) | OLED驱动当前不足、重构及图形/输入扩展设想说明。 | 同步当前未验证与待设备需求状态，记录未来启用前置条件；文中图形库/编码器设想只留参考，不自动纳入本轮实施。 | [MC02-0085](../../../.Doc/TODO.md) |
| 207 | [modules/oled/oledfont.h](../../../modules/oled/oledfont.h) | OLED 6x12 ASCII点阵与128x64 LOGO位图。 | 随OLED保留资源和来源，未来按实际页面需要验证/裁剪，不新增字体或图形；静态检查95字符每字12字节、LOGO128行每行8字节通过，未宣称实物字形验收。 | [MC02-0085](../../../.Doc/TODO.md) |

### 208–216：输入适配与待接入通信

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 208 | [modules/standard_cmd/std_cmd.c](../../../modules/standard_cmd/std_cmd.c) | 当前为空；预期将不同来源已解析合法输入按车型映射为标准控制意图。 | 用户选择本轮建立统一输入适配模块，统一SI单位、明确参考系和源有效期；robot_cmd唯一负责模式/来源仲裁，safety负责运行许可。协议解析留在各来源模块，不重发续期；底盘坐标变换复用已选运动接口。 | [MC02-0003](../../../.Doc/TODO.md) |
| 209 | [modules/standard_cmd/std_cmd.h](../../../modules/standard_cmd/std_cmd.h) | 当前为空；用于声明输入适配与标准控制意图接口。 | 随208建立带类型约束的意图/来源有效性接口，区分连续目标与离散事件，复用统一话题/快照约定。与仲裁/安全保持单一职责，不产生第二个许可状态所有者。 | [MC02-0003](../../../.Doc/TODO.md)、[MC02-0070](../../../.Doc/TODO.md) |
| 210 | [modules/standard_cmd/standard_cmd.md](../../../modules/standard_cmd/standard_cmd.md) | 不同输入源转换为底盘/云台/发射标准命令的规划说明。 | 更新为208的真实职责、单位/参考系/源时间、车型映射和接入/验证流程；本轮来源使用已选遥控/键鼠/上下位机，PS手柄/图传链路等只作后续设想，不自动新增驱动。 | [MC02-0085](../../../.Doc/TODO.md)、[MC02-0003](../../../.Doc/TODO.md) |
| 211 | [modules/unicomm/unicomm.c](../../../modules/unicomm/unicomm.c) | 当前为空；原设想为跨链路的可配置通用帧解析。 | 用户选择暂缓unicomm实现，各已选协议先复用已有CRC/缓冲/局部时间接口，SP/新业务协议/裁判/CAN分片保持已定职责，不新增通用组帧平台。 | [MC02-0085](../../../.Doc/TODO.md) |
| 212 | [modules/unicomm/unicomm.h](../../../modules/unicomm/unicomm.h) | 当前为空；原拟声明通用通信接口。 | 随211维持未实现占位，不对外宣称可用，不因空头文件引入另一套协议/业务/安全接口。 | [MC02-0085](../../../.Doc/TODO.md) |
| 213 | [modules/unicomm/unicomm.md](../../../modules/unicomm/unicomm.md) | 通用可配置帧解析及替代旧通信模块的设想。 | 说明本轮暂缓，将原拟直接替代master_process/CANComm的文字改为待评估设想；保留已批准独立业务/分片职责及可复用基础，不将其当作已实现能力。 | [MC02-0085](../../../.Doc/TODO.md) |
| 214 | [modules/bluetooth/HC05.c](../../../modules/bluetooth/HC05.c) | 通过串口收发固定帧格式的HC05蓝牙数据。 | 用户确认本轮暂缓实际接入，保留代码并标明未验证；待明确设备需求后再完善。已发现长度/帧尾、接收有效性及缓冲接口问题保留为后续接入依据，不作为本轮修复承诺。 | [MC02-0085](../../../.Doc/TODO.md) |
| 215 | [modules/bluetooth/HC05.h](../../../modules/bluetooth/HC05.h) | 声明HC05固定数据容量、消息结构及初始化和发送入口。 | 沿用214项暂缓范围；文档说明当前能力与未验证状态，后续接入再补齐结果、容量和数据有效性契约。保留历史已完成蓝牙任务记录。 | [MC02-0085](../../../.Doc/TODO.md) |
| 216 | [modules/encoder/encoder.md](../../../modules/encoder/encoder.md) | 外接光电及霍尔编码器支持的设想，当前没有独立驱动实现。 | 用户确认本轮暂缓，保留接入设想，待设备、接口和用途明确后开发；不据此新增编码器驱动或通用定时器BSP，电机已有编码器反馈保持原范围。 | [MC02-0085](../../../.Doc/TODO.md) |

### 217–218：公共定义及模块入口

| 编号 | 文件 | 用途 | 最后讨论结论 | 关联任务 |
|---:|---|---|---|---|
| 217 | [modules/general_def.h](../../../modules/general_def.h) | 定义角度、转速、时间和长度的通用换算常量。 | 沿用已确认SI/纯数值分类接口：换算命名明确输入输出单位、同源常量推导并消除重复定义；随调用迁移核对旧协议/输出轴/减速比/方向及目标、反馈、PID参数，不能仅替换宏值。归并现有数值与电机单位任务，不新增单位方案。 | [MC02-0076](../../../.Doc/TODO.md) |
| 218 | [modules/module.md](../../../modules/module.md) | 模块层总说明，当前仅有一句占位文字。 | 补充现有模块职责、配置/编译依赖、初始化结果、接口数据单位/参考系/源时间/有效性、缓冲归属/任务职责、故障事实到safety和裸板验证入口；链接已有说明，状态只在TODO。通用驱动模板及暂缓模块仍保持原范围。 | [MC02-0085](../../../.Doc/TODO.md) |

## B2–B9、S1–S8 批次选择与任务映射

下表所有组均经用户整批批准，描述登记范围，不记录实施状态。第4批在超级电容尚未选型的条件下登记接口／边界，未承诺具体设备适配；S1-3 最终选人工核对，S3 选无强制握手，后续选择覆盖组内早期备选。只给出未来说明的组不产生对应硬件驱动实现。

| 讨论组 | 主题 | 批准范围 | 关联任务 |
|---|---|---|---|
| B2-1 | 新版本上下位机业务协议 | 保留SP兼容；下位机有界拼帧/多帧；瞄准与导航独立有效；上位机代码另行安排。新版协议首合法未过期新命令恢复，无预先握手；详见 S3。 | [MC02-0089](../../../.Doc/TODO.md)、[MC02-0062](../../../.Doc/TODO.md)、[MC02-0056](../../../.Doc/TODO.md) |
| B2-2 | 上下位机对时 | 下位机对时交互、偏差与同步有效性，协议/校验样例和主机行为测试。 | [MC02-0090](../../../.Doc/TODO.md) |
| B2-3 | 视觉前馈与有界延迟补偿 | 速度/加速度前馈分别配置；对时和参考时刻有效时有界外推，不改变视觉击发时机或延长有效期。 | [MC02-0091](../../../.Doc/TODO.md) |
| B2-4 | Seasky仅留参考 | 源码/头文件/署名资料保留，移出固件构建，不实现第三种兼容协议。 | [MC02-0008](../../../.Doc/TODO.md)、[MC02-0054](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| B2-5 | 新版CAN板间协议与话题桥接 | 经典8字节，新版本/消息序号/分片编号；两端升级；配置显式跨板话题，业务统一消息接口。 | [MC02-0092](../../../.Doc/TODO.md)、[MC02-0014](../../../.Doc/TODO.md)、[MC02-0070](../../../.Doc/TODO.md) |
| B2-6 | CAN板间对时 | 映射源时间并发布同步有效状态，保留源有效期；局部适配现有时基，不恢复R11。 | [MC02-0093](../../../.Doc/TODO.md) |
| B3-1 | 裁判接收与分组状态 | 保留既有2026V2.0.0适配成果，修正合法帧续期、原始接收块/时间、跨块拼帧及一致只读分组快照，裁判与可选UI解耦。 | [MC02-0094](../../../.Doc/TODO.md)、[BSPUART-03U1](../../../.Doc/TODO.md) |
| B3-2 | 结构化裁判摘要上报 | 新业务协议按车型配置输出决策所需状态及源时间/有效性；本轮仅下位机，上位机代码后续。 | [MC02-0089](../../../.Doc/TODO.md) |
| B3-3 | 通用机器人间收发 | 按配置的内容ID/对端、锁定官方身份/长度/共享频率约束完成收发与本机结果；具体协同消息和对端ACK后续。 | [MC02-0022](../../../.Doc/TODO.md)、[REF-03](../../../.Doc/TODO.md) |
| B3-4 | 配置化UI与恢复 | 配置布局/样式/只读来源，增量更新+低频分批重绘+链路恢复/人工重绘；修复字符串、计数、边界及序号问题。 | [MC02-0095](../../../.Doc/TODO.md)、[REF-03](../../../.Doc/TODO.md)、[REF-04](../../../.Doc/TODO.md) |
| B3-5 | CRC与接入文档 | 裁判CRC采用独立参数配置保持线上结果，合入统一库；文档按已选接口、协议版本、预算、UI及结果语义更新。 | [MC02-0079](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| B4-1 | 底盘功率预算与保守降级 | 有效功率上限/供电许可下，缓冲数据失效使用配置余量且停用其额外功率；复用平移/自旋优先级。 | [R25-15](../../../.Doc/TODO.md)、[MC02-0057](../../../.Doc/TODO.md) |
| B4-2 | 配置化模型接口与在线辨识接入 | 区分额度、真实测量、候选/生效模型和限制结果，统一电机单位/型号适配；可信数据条件未齐不认定在线辨识可用。 | [MC02-0080](../../../.Doc/TODO.md)、[MC02-0081](../../../.Doc/TODO.md)、[MC02-0072](../../../.Doc/TODO.md) |
| B4-3 | 现有电容边界与文档 | 接收长度/在线新鲜度、发送长度/结果/空参数及不安全示例；修订MISC-04现状，不包含未知设备协议实现。 | [MISC-04](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| B5-1 | 配置化遥控手感 | 中位/端点、死区、可选响应曲线，默认线性；参数校验及裸板只读观测。 | [MC02-0096](../../../.Doc/TODO.md)、[MC02-0064](../../../.Doc/TODO.md)、[PARAM-03](../../../.Doc/TODO.md) |
| B5-2 | 一致输入与事件 | 连续输入快照、离散操作有期限事件、原始接收时间与各来源有效性；移除业务对8位累计数取模的依赖，保留已验键鼠租约。 | [MC02-0096](../../../.Doc/TODO.md)、[MSG-05](../../../.Doc/TODO.md)、[MSG-06](../../../.Doc/TODO.md)、[MC02-0070](../../../.Doc/TODO.md) |
| B5-3 | 安全手势与恢复 | 正式自主人工急停明确解除后无需再次使能，调试仅SAFE；持续供电期间人工急停跨复位保留，未明确解除不能自动运行。 | [MC02-0056](../../../.Doc/TODO.md)、[MC02-0067](../../../.Doc/TODO.md) |
| B5-4 | 裸板诊断与文档 | 受独立构建约束、禁止READY/执行器动作，观察原始/处理后输入及租约；文档对应实际字段、映射和恢复规则。 | [MC02-0026](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| B6-1 | 温控状态与许可衔接 | 首台哨兵在姿态/标定可信且无温控故障时允许未温稳使能，状态提示；健康快照区分温度/加热/温稳/故障，复用参数与周期约定，现有保护和禁热边界保留。 | [MC02-0056](../../../.Doc/TODO.md)、[MC02-0065](../../../.Doc/TODO.md) |
| B6-2 | 旧BMI088路径清理 | 删除BMI088Middleware.c/.h与BMI088reg.h三个旧文件，清理构建/文档引用，保留当前modules/BMI088生产路径与Git来源。 | [MC02-0054](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| B6-3 | 传感器硬件依据与待接入边界 | 保存官方MC-02无板载磁力计证据；IST8310外接扩展和TFminiPlus实际接入后置，保留未验证说明及未来启用前置条件，不新增两种驱动完善任务。 | [MC02-0051](../../../.Doc/TODO.md)、[MISC-03](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| B7-1 | LCD绘图与刷新 | 有界字符串、字模/坐标/数值、操作结果/失败传播与缓存提交、任务预算，保留行缓存，不引入全局SPI平台。 | [MC02-0097](../../../.Doc/TODO.md)、[LCD-06](../../../.Doc/TODO.md)、[LCD-07](../../../.Doc/TODO.md) |
| B7-2 | 只读健康与故障界面 | 复用健康/姿态/裁判分组等只读快照，纠正无效/保留字段显示，页面冻结单所有者，新严重故障解除冻结跳故障页。 | [MC02-0098](../../../.Doc/TODO.md)、[LCD-14](../../../.Doc/TODO.md) |
| B7-3 | 按键与标定确认 | 适配周期的时间语义、无效采样取消；运行可进入确认页，确认后框架失能/检查并由IMU任务标定，界面显示请求/执行/结果。 | [MC02-0098](../../../.Doc/TODO.md)、[MC02-0056](../../../.Doc/TODO.md) |
| B7-4 | 资源文档与待接入边界 | 字模按实际标签增补与一致性检查、OLED未验证/后续接入说明，保留MISC-06/LCD-10/0046真实历史。 | [MC02-0085](../../../.Doc/TODO.md)、[MC02-0102](../../../.Doc/TODO.md)、[MISC-06](../../../.Doc/TODO.md) |
| B8-1 | standard_cmd输入适配 | 已解析合法的遥控/键鼠/上下位机输入按车型映射为标准控制意图，统一SI单位、参考系和源有效期；仲裁归robot_cmd，许可归safety。 | [MC02-0003](../../../.Doc/TODO.md) |
| B8-2 | unicomm暂缓边界 | 保留空实现，文档明确通用组帧平台后置；各具体协议复用既有CRC、缓冲和时间接口，不自动替代现有协议模块。 | [MC02-0085](../../../.Doc/TODO.md) |
| B8-3 | 蓝牙与编码器待接入说明 | HC05保留代码并注明未验证，外接编码器保留设想；待设备与用途明确后开发，不把缺陷记录转成本轮实现任务。 | [MC02-0085](../../../.Doc/TODO.md) |
| B9-1 | 通用数值与单位换算 | 统一换算来源及清晰单位命名，逐调用迁移并校验目标/反馈/PID参数配套，验证已知值与往返误差。 | [MC02-0076](../../../.Doc/TODO.md) |
| B9-2 | 模块层接入说明 | module.md作为现有模块职责与配置、初始化、快照、故障和验证约定的入口，链接实际说明，不新增通用驱动模板。 | [MC02-0085](../../../.Doc/TODO.md) |
| S1-1 | 唯一CMake构建 | 删除旧Makefile/编辑器任务，CMake按平台与车型选择源码，维护可复现构建产物。 | [BUILD-02](../../../.Doc/TODO.md)、[MC02-0020](../../../.Doc/TODO.md)、[MC02-0054](../../../.Doc/TODO.md) |
| S1-2 | 自有C平台与OS适配 | 业务接口隐藏HAL/RTOS原生类型和芯片寄存器，逻辑资源/能力声明、平台与车型分配置，第一后端H723+FreeRTOS。 | [MC02-0086](../../../.Doc/TODO.md) |
| S1-3 | 生成与布局保护 | 明确平台生成与定制位置，核对 Core／RTOS／USB、启动、链接及 Flash／RAM／.noinit；本轮先提供人工核对流程，新增 CubeMX 自动检查后续。 | [MC02-0055](../../../.Doc/TODO.md)、[MC02-0063](../../../.Doc/TODO.md)、[MC02-0068](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| S1-4 | 调试配置与产物一致 | 继续维护VSCode/OpenOCD/Ozone，按平台和构建产物选择目标，分离探针/安装路径/个人会话，共用匹配ELF/map的构建标识。 | [MC02-0020](../../../.Doc/TODO.md)、[MC02-0068](../../../.Doc/TODO.md)、[MC02-0103](../../../.Doc/TODO.md) |
| S2-1 | 分阶段平台启动与错误传播 | 硬件初始化按启用依赖返回结果；核心失败与辅助降级分开，硬件等待有界，保留启动关断、诊断锁存与已有故障策略。 | [MC02-0087](../../../.Doc/TODO.md)、[MC02-0055](../../../.Doc/TODO.md) |
| S2-2 | 统一RTOS任务与高频唤醒 | 业务不暴露RTOS类型；配置支持亚毫秒硬件定时唤醒、启动分辨率与预算校验，控制留任务中；现有只读任务统计由OS后端适配，不新增R11统计平台。 | [MC02-0088](../../../.Doc/TODO.md)、[MC02-0069](../../../.Doc/TODO.md) |
| S2-3 | H723外设能力与生命周期 | 逻辑资源绑定到实际配置；落实已选扩展CAN过滤器与模式能力，修正局部IRQ派发/退出清理，保留共享时钟管理、禁热及现有超时保护。 | [MC02-0087](../../../.Doc/TODO.md)、[MC02-0060](../../../.Doc/TODO.md)、[MC02-0075](../../../.Doc/TODO.md) |
| S2-4 | 标准库与诊断出口 | 标准输出和旧fputc统一有界日志；标准输入不交互，诊断仍用既定协议；退出/断言与故障流程衔接，堆边界和配置预算明确，更新Core接入说明。 | [MC02-0066](../../../.Doc/TODO.md)、[MC02-0067](../../../.Doc/TODO.md)、[MC02-0087](../../../.Doc/TODO.md) |
| S3-1 | USB平台接口与生命周期 | USB_DEVICE/H7及ST USB栈类型留后端；初始化逐项返回结果，配置/复位/断开/挂起状态与业务数据有效性区分，容量和描述符对应实际单CDC/Full Speed配置。 | [MC02-0062](../../../.Doc/TODO.md)、[MC02-0087](../../../.Doc/TODO.md) |
| S3-2 | 收发缓冲及处理预算 | 保留ISR复制/任务派发/源时间与在途持久TX缓冲；明确队列/块长上限、丢块和忙/错误/完成结果，单消费者按预算处理，不静默截断或改写在途数据。 | [MC02-0062](../../../.Doc/TODO.md)、[MC02-0061](../../../.Doc/TODO.md) |
| S3-3 | 首合法命令恢复与验证说明 | 无需预先握手；物理或业务连接失效撤销旧接收/拼帧/命令与USB开火授权，首份合法未过期新命令恢复对应业务；瞄准/移动独立，模式/safety许可规则不变。兼容SP，补USB发送/生命周期/过期回归与人工再生成核对。 | [MC02-0056](../../../.Doc/TODO.md)、[MC02-0062](../../../.Doc/TODO.md)、[MC02-0089](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| S4-1 | 主机回归入口与环境 | Linux/WSL完整基线、Windows可移植子集；保留Debug/Release及负向检查器控制；声明跳过原因、每类用例有限超时和统一命令 | [MC02-0020](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| S4-2 | 平台解耦后的测试分层 | 业务测试通过框架接口可控替身调用生产逻辑；H723后端测试保留局部HAL/寄存器模型；避免测试复制公共业务结构、明确无操作临界区/成功替身的覆盖边界 | [MC02-0086](../../../.Doc/TODO.md)、[MC02-0087](../../../.Doc/TODO.md)、[MC02-0088](../../../.Doc/TODO.md)、[MC02-0070](../../../.Doc/TODO.md) |
| S4-3 | 已选功能的必要行为回归与依据 | 为已批准安全/协议/算法改造补相应生产接口回归；数值验收使用实际执行运算的后端与可核对结果；保留初始化故障注入；记录替身边界和实物验收差异，不建设整车模拟/回放 | [MC02-0056](../../../.Doc/TODO.md)、[MC02-0076](../../../.Doc/TODO.md)、[MC02-0078](../../../.Doc/TODO.md)、[MC02-0079](../../../.Doc/TODO.md)、[MC02-0080](../../../.Doc/TODO.md)、[MC02-0082](../../../.Doc/TODO.md)、[MC02-0083](../../../.Doc/TODO.md) |
| S5-1 | 只读观测整合与旧入口退役 | MC02-0100软件实现及WSL／ARM验收已完成；24V禁热裸板验收待执行。性能/任务/CAN/LCD查询进入probe_snapshot_live；保留有限白名单、运行状态与ELF身份／布局核对；旧探针已删除，RTT由OpenOCD原生接收客户端提供；不扩展R11 | [MC02-0100](../../../.Doc/TODO.md)；[任务记录](../../workflow/tickets/MC02-0100.md) |
| S5-2 | 裸板输入与最小协议测试帧 | SP与新版协议的最小禁火测试帧；明确物理量/单位、显式设备/端口、有界参数/发送/错误停止和结果；不改生产上位机，不建设整车模拟回放；保留DT7身份/输入租约/两帧全松开恢复规则并纠正固定UI端口说明，不给live虚构180s上限 | [MC02-0101](../../../.Doc/TODO.md) |
| S5-3 | 设备监听与字模生成维护 | USB-CAN仅维护已观测私有格式，校验类型/长度/DLC/ID、有限读取与异常退出，不宣称通用CAN/主动收发/扩展帧能力；字体和输出路径可指定，先验证标签/字体再交付整组生成结果，保持枚举/字节约定并记录生成依据，不扩未用字库 | [MC02-0102](../../../.Doc/TODO.md) |
| S5-4 | 现有工作流检查适配 | ELF/profile检查按选中车型模块及平台产生期望，保持bench隔离与诊断身份；增量与干净配置一致性跟随新CMake入口/源码选择/输入布局；保留TODO契约与课程局部链接/可选构建；说明命令/依赖/产物，新的CubeMX自动检查后置 | [BUILD-02](../../../.Doc/TODO.md)、[MC02-0020](../../../.Doc/TODO.md)、[MC02-0054](../../../.Doc/TODO.md)、[MC02-0032](../../../.Doc/TODO.md) |
| S6-1 | 当前入口与历史参考 | 根README按已选方向重组，提供当前架构/支持范围/快速入口和文档导航；上游长教程移历史参考并保留来源，调试指南区分H723现行操作与F4/Makefile历史；引用/图示随迁移修正，计划能力不标已实现 | [MC02-0085](../../../.Doc/TODO.md)、[MC02-0020](../../../.Doc/TODO.md)、[BUILD-02](../../../.Doc/TODO.md) |
| S6-2 | 术语和架构边界 | 用中文及必要英文标识统一平台/机器人配置、业务接口、来源/许可/有效数据等领域含义；GLOSSARY不充当实现规格，协议完整CRC参数/单位/时间规则放对应规格；记录C接口隔离HAL与RTOS、首后端H723+FreeRTOS且不引入LibXR C++实现这一有实际取舍的架构决策 | [MC02-0086](../../../.Doc/TODO.md)、[MC02-0056](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| S6-3 | CI与可复现构建包 | 现有Debug/Release主机和固件检查随平台/车型及profile调整，保留非法配置和bench隔离；成功构建包包含ELF/HEX/BIN/map、构建标识/车型/配置与校验依据，CI可下载、本地同格式；不改变默认本地交付，也不接入自动硬件操作 | [MC02-0103](../../../.Doc/TODO.md)、[MC02-0068](../../../.Doc/TODO.md) |
| S6-4 | 教学与记录衔接 | 修正已确认本地票据链接、当前说明中的退役工具和不再适用流程；已变更主题随实现更新原笔记/课程及索引；明确课程模型与生产回归、软件与实物证据边界；历史报告/票据/硬件证据保留日期和适用版本，状态仅TODO；代理技能本地覆盖/锁定来源保留，不自动升级或恢复审查/推送/PR流程 | [MC02-0085](../../../.Doc/TODO.md) |
| S7-1 | 第三方基线与构建来源 | 锁定HAL1.11.1、FreeRTOS10.3.1、RTT7.58c及实际CMSIS/USB文件基线；记录来源、版本/指纹、实际参与构建文件和必要本地补丁；保留组件原有来源/许可说明，整体升级另行安排；未用示例/模板按组说明，继续显式选源码 | [MC02-0020](../../../.Doc/TODO.md)、[MC02-0099](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| S7-2 | 同源DSP最小源码构建 | 固定CMSIS_5 5.7.0 commit a65b7c9a3e6502127fdb80eb288d8cbdf251a6f4的DSP1.7.0，匹配现用头；9个所需C源码与同源头/表按目标构建，移除实际构建对不明来源预编译库/重复头的依赖；目标上设置矩阵维度检查，固件FP/ABI选项匹配且无fast-math，业务不强制后端宏；Linux/WSL以该版本真实运算分支验算，独立工作区保护求逆输入和旧结果 | [MC02-0099](../../../.Doc/TODO.md)、[MC02-0076](../../../.Doc/TODO.md)、[MC02-0078](../../../.Doc/TODO.md)、[MC02-0080](../../../.Doc/TODO.md) |
| S7-3 | 第三方后端接入与验证 | HAL/USB/RTOS/RTT依赖留选中平台后端，业务经框架C接口；port选择按内核修订/浮点/中断约束核对，不能仅凭ARM_CM4F目录名判错；RTT作为有界辅助日志输出，缓冲/模式/临界区及启用Cache时可见性由平台负责；复用现有启动/收发/故障/OS适配验证，不泛建资源或事务平台 | [MC02-0086](../../../.Doc/TODO.md)、[MC02-0087](../../../.Doc/TODO.md)、[MC02-0088](../../../.Doc/TODO.md)、[MC02-0066](../../../.Doc/TODO.md) |
| S7-4 | 资源用途与新旧图示 | 61个资源记录引用/用途；当前架构、任务/初始化/CAN/消息流程随已批准接口更新可维护图示；上游教程截图/机器人展示留历史参考并保留来源，迁移引用同步；未找到引用资源先说明，未授权批量裁剪 | [MC02-0085](../../../.Doc/TODO.md)、[MC02-0092](../../../.Doc/TODO.md)、[MC02-0070](../../../.Doc/TODO.md) |
| S8-1 | 资料目录与查阅入口 | 在主工程整理独立资料仓库来源/修订、18套厂商例程与硬件资料的用途索引；标明USAGE中的过时路径，以实际目录/原文档为准；厂家例程只作相关接口与寄存器参考，不并入业务或批量更名/改写 | [MC02-0085](../../../.Doc/TODO.md) |
| S8-2 | 六份业务配套协议资料 | 补记218个.c/.h/.md之外的3份PDF和3张报文图；关联BMI088、HT、LK驱动任务；保留资料原文与版本，HT电流/力矩表述冲突、LK型号/固件/模式差异按设备待确认项管理；不因资料存在就判定实物能力已验证 | [MC02-0073](../../../.Doc/TODO.md)、[MC02-0074](../../../.Doc/TODO.md)、[MC02-0085](../../../.Doc/TODO.md) |
| S8-3 | 独立裁判模拟器对齐与复用 | 保持独立仓库，锁定2026 V2.0.0与现固件的业务定义，建立共同已核对字节向量、错误帧/失联/恢复/UI/功率热量场景及裸板验证流程；必要串口写时限/短写结果与文档校正；MC-02与模拟器分别维护关联TODO，不新增GUI/CLI/MQTT功能 | [MC02-0104](../../../.Doc/TODO.md)、[MC02-0022](../../../.Doc/TODO.md)、[MC02-0026](../../../.Doc/TODO.md)、[MC02-0079](../../../.Doc/TODO.md) |

## 工程支撑覆盖与方法（2026-10-15～16）

主仓库审查基线为 `c0bb78c`，`git ls-files` 共2245个跟踪文件。统计按文件归属互斥，业务218项另按讨论顺序编号；“有用途与核对记录”不等于对所有厂商源码、文档论述、图片像素或模型逐行／逐帧验收。本次未修改这些代码或资源。

| 归属 | 数量 | 用途及核对方法 |
|---|---:|---|
| 根目录 | 19 | 构建、启动、链接、CubeMX、SVD、调试、说明与流程入口；源码选择／布局／个人配置按 S1、S6核对 |
| `.Doc/` | 1 | 唯一任务状态文件 TODO；独立资料仓库不计入此数 |
| `.agents/` | 101 | 本地技能及来源锁定／辅助资料；按流程入口与覆盖规则核对，未激活全部技能或逐篇重验外部内容 |
| `.assets/` | 61 | 图示／教程截图／机器人展示；检查引用和用途，当前图示随接口更新，历史资源保留来源；60有引用、1未检出引用，不自动删除 |
| `.github/` | 1 | CI：现有主机及固件检查；构建包下载后续由0103实现 |
| `.vscode/` | 2 | 构建任务与调试；移除 Makefile 入口、匹配 CMake 产物属后续任务 |
| `Core/` | 31 | 启动、外设、RTOS、中断和标准库适配，文件用途如下 |
| `Drivers/` | 1444 | HAL／CMSIS／DSP／器件支持；按实际选中构建、版本／来源与未用模板分组核对 |
| `Middlewares/` | 62 | FreeRTOS、USB、DSP头及RTT等；按后端依赖与实际选中实现核对 |
| `USB_DEVICE/` | 8 | CDC与USB底层接入，文件用途如下 |
| `application/` | 29 | 第1–29项 |
| `bsp/` | 52 | 第30–81项 |
| `modules/` | 143 | 第82–218项共137个.c/.h/.md，加6份配套资源 |
| `docs/` | 106 | 导航、流程、规格／票据／映射、历史报告、学习、课程和硬件依据；检查入口／引用及方案迁移影响，不逐句重验技术论述 |
| `tests/` | 172 | 按生产接口、场景和替身边界分组核对，详见下文；未运行固件或全套主机回归 |
| `tools/` | 13 | 输入、观测、监听、字模及工作流工具；独立模拟器不计入此数 |
| **合计** | **2245** | 独立仓库、未跟踪资料和缓存单独说明 |

### Core 与 USB_DEVICE 文件用途

两组均按源码、配置与调用连接核对。USB 的 HS 命名不等于实际 High Speed；当前使用内置 Full Speed PHY。SPI 的枚举模式不证明 IRQ／DMA 已配置，OSPI 控制器初始化不证明外部 Flash 器件可用。各后端改造依赖0086／0087／0088，USB业务规则归0062。

| 文件 | 用途 |
|---|---|
| [Core/Src/main.c](../../../Core/Src/main.c) | 硬件、时钟与框架启动 |
| [Core/Inc/main.h](../../../Core/Inc/main.h) | 芯片头及板级引脚定义 |
| [Core/Src/adc.c](../../../Core/Src/adc.c) | ADC1双通道采样与DMA配置 |
| [Core/Inc/adc.h](../../../Core/Inc/adc.h) | ADC平台句柄与初始化声明 |
| [Core/Src/dma.c](../../../Core/Src/dma.c) | DMA时钟和中断初始化 |
| [Core/Inc/dma.h](../../../Core/Inc/dma.h) | DMA初始化声明 |
| [Core/Src/fdcan.c](../../../Core/Src/fdcan.c) | 三路经典CAN及消息RAM、过滤器、引脚和中断配置 |
| [Core/Inc/fdcan.h](../../../Core/Inc/fdcan.h) | 三路CAN平台句柄与初始化声明 |
| [Core/Src/gpio.c](../../../Core/Src/gpio.c) | 板级输入输出、初始电平和EXTI配置 |
| [Core/Inc/gpio.h](../../../Core/Inc/gpio.h) | GPIO初始化声明 |
| [Core/Src/i2c.c](../../../Core/Src/i2c.c) | I2C2时序、引脚、DMA与中断配置 |
| [Core/Inc/i2c.h](../../../Core/Inc/i2c.h) | I2C2平台句柄与初始化声明 |
| [Core/Src/octospi.c](../../../Core/Src/octospi.c) | OSPI2控制器与引脚初始化，尚无外部Flash器件读写协议 |
| [Core/Inc/octospi.h](../../../Core/Inc/octospi.h) | OSPI2平台句柄与初始化声明 |
| [Core/Src/spi.c](../../../Core/Src/spi.c) | SPI1显示通信和SPI2传感器通信的初始化 |
| [Core/Inc/spi.h](../../../Core/Inc/spi.h) | SPI平台句柄与初始化声明 |
| [Core/Src/tim.c](../../../Core/Src/tim.c) | TIM1/2/3/12的PWM及引脚配置 |
| [Core/Inc/tim.h](../../../Core/Inc/tim.h) | 定时器平台句柄与初始化声明 |
| [Core/Src/usart.c](../../../Core/Src/usart.c) | 八路UART/USART参数、RS485、DMA和中断配置，附旧fputc串口输出 |
| [Core/Inc/usart.h](../../../Core/Inc/usart.h) | 串口平台句柄与初始化声明 |
| [Core/Src/stm32h7xx_it.c](../../../Core/Src/stm32h7xx_it.c) | 主要CPU异常处理及外设IRQ派发 |
| [Core/Inc/stm32h7xx_it.h](../../../Core/Inc/stm32h7xx_it.h) | IRQ处理函数声明 |
| [Core/Src/freertos.c](../../../Core/Src/freertos.c) | FreeRTOS默认任务、USB启动、静态内核任务存储及运行时间统计钩子 |
| [Core/Inc/FreeRTOSConfig.h](../../../Core/Inc/FreeRTOSConfig.h) | RTOS tick、优先级、堆、栈检测、内核选项及异常钩子配置 |
| [Core/Src/stm32h7xx_hal_msp.c](../../../Core/Src/stm32h7xx_hal_msp.c) | 基础HAL MSP时钟与PendSV优先级设置 |
| [Core/Src/stm32h7xx_hal_timebase_tim.c](../../../Core/Src/stm32h7xx_hal_timebase_tim.c) | TIM23 HAL计时、暂停/恢复及阻塞等待租约 |
| [Core/Inc/bsp_hal_tick.h](../../../Core/Inc/bsp_hal_tick.h) | HAL阻塞等待租约入口声明，属于平台适配内部 |
| [Core/Src/system_stm32h7xx.c](../../../Core/Src/system_stm32h7xx.c) | CMSIS启动阶段时钟/FPU设置及系统时钟变量计算 |
| [Core/Src/syscalls.c](../../../Core/Src/syscalls.c) | C标准库输入输出、进程/文件占位调用与退出的嵌入式适配 |
| [Core/Src/sysmem.c](../../../Core/Src/sysmem.c) | 通过链接边界实现newlib堆扩展_sbrk |
| [Core/Inc/stm32h7xx_hal_conf.h](../../../Core/Inc/stm32h7xx_hal_conf.h) | HAL模块、晶振、tick优先级、回调及断言编译配置 |
| [USB_DEVICE/App/usb_device.c](../../../USB_DEVICE/App/usb_device.c) | 初始化USB设备、注册CDC类和接口并启动设备。 |
| [USB_DEVICE/App/usb_device.h](../../../USB_DEVICE/App/usb_device.h) | 声明USB设备初始化，当前暴露芯片与USB栈依赖。 |
| [USB_DEVICE/App/usbd_cdc_if.c](../../../USB_DEVICE/App/usbd_cdc_if.c) | CDC接收、发送、完成及初始化/退出/控制回调；接收入队、发送复制持久缓冲。 |
| [USB_DEVICE/App/usbd_cdc_if.h](../../../USB_DEVICE/App/usbd_cdc_if.h) | CDC接口、2048字节接收/发送容量与回调声明。 |
| [USB_DEVICE/App/usbd_desc.c](../../../USB_DEVICE/App/usbd_desc.c) | USB设备与字符串描述符，并从芯片UID派生序列号。 |
| [USB_DEVICE/App/usbd_desc.h](../../../USB_DEVICE/App/usbd_desc.h) | 描述符入口与H723 UID地址等定义。 |
| [USB_DEVICE/Target/usbd_conf.c](../../../USB_DEVICE/Target/usbd_conf.c) | ST USB设备栈到HAL PCD的适配，含IRQ、连接事件、端点、FIFO和静态类存储。 |
| [USB_DEVICE/Target/usbd_conf.h](../../../USB_DEVICE/Target/usbd_conf.h) | USB栈编译参数、内存/延时/日志钩子和底层接口声明。 |

### tests：172个文件的覆盖口径

39个功能测试源码＋4个入口／检查器文件＋2个 BMI088 公共夹具＋125个替身文件＋2个 Python 工具测试，共172。功能测试按生产编译来源、失败断言和场景核对；125个 HAL／RTOS／设备替身按被测边界分组，不逐个重复讨论同类声明。Debug／Release 检查和负向检查器控制沿用，完整 Linux／WSL、Windows可移植子集为后续基线。

| 功能测试源码 | 当前检查的行为与局限 |
|---|---|
| [bmi088_calibration_temperature_test.c](../../../tests/host/bmi088_calibration_temperature_test.c) | 生产注册/启动与按需标定，温度单位、失败样本跳过、维护保护与让出执行 |
| [bmi088_init_failure_test.c](../../../tests/host/bmi088_init_failure_test.c) | 初始化失败诊断、阶段、重试及协作等待 |
| [bmi088_init_write_error_test.c](../../../tests/host/bmi088_init_write_error_test.c) | 初始化SPI错误终止本轮并重试，失败写入不修改设备模型 |
| [bmi088_post_reset_dummy_read_test.c](../../../tests/host/bmi088_post_reset_dummy_read_test.c) | 加速度计复位后虚读与芯片ID确认 |
| [bmi088_sample_status_test.c](../../../tests/host/bmi088_sample_status_test.c) | 整组采样原子发布、SPI失败、非有限值、序号和新鲜度 |
| [bmi088_temperature_test.c](../../../tests/host/bmi088_temperature_test.c) | 温度有符号解码与端点 |
| [bsp_adc_snapshot_test.c](../../../tests/host/bsp_adc_snapshot_test.c) | DMA整批快照、缓存处理调用、配置与过期/错误状态 |
| [bsp_crash_frame_test.c](../../../tests/host/bsp_crash_frame_test.c) | 基本/扩展异常帧、地址边界与压栈错误 |
| [bsp_safety_test.c](../../../tests/host/bsp_safety_test.c) | H723寄存器关断顺序和中断屏蔽状态 |
| [bsp_spi_status_test.c](../../../tests/host/bsp_spi_status_test.c) | SPI返回状态、总线归属、超时与并发争用，HAL时基等待租约 |
| [can_comm_freshness_test.c](../../../tests/host/can_comm_freshness_test.c) | CAN通信快照和校验失败不续期；CRC为加和替身 |
| [chassis_motion_test.c](../../../tests/host/chassis_motion_test.c) | 运动学、单位转换、跟随/自旋与轮速限幅、非法输入 |
| [chassis_task_test.c](../../../tests/host/chassis_task_test.c) | 生产底盘任务、配置缺失失能、写目标后使能，电机替身 |
| [crash_guard_test.c](../../../tests/host/crash_guard_test.c) | 异常重入立即复位，用setjmp模拟复位终止 |
| [fatal_error_entry_test.c](../../../tests/host/fatal_error_entry_test.c) | Error_Handler最小关断入口 |
| [hal_tick_suspend_test.c](../../../tests/host/hal_tick_suspend_test.c) | HAL时基挂起/恢复与阻塞等待租约 |
| [imu_heater_fault_recovery_test.c](../../../tests/host/imu_heater_fault_recovery_test.c) | 真实温控和PID在使能/禁用配置下的故障、边界、恢复和维护 |
| [imu_heater_sample_test.c](../../../tests/host/imu_heater_sample_test.c) | 温控拒绝过期样本及保护，PID为替身 |
| [init_diagnostic_test.c](../../../tests/host/init_diagnostic_test.c) | 首个初始化失败保留与只读诊断 |
| [init_resource_test.c](../../../tests/host/init_resource_test.c) | daemon/message注册参数、容量、分配失败回滚 |
| [kalman_filter_init_test.c](../../../tests/host/kalman_filter_init_test.c) | 卡尔曼分配/矩阵初始化失败回滚；矩阵运算为占位 |
| [lcd_key_input_test.c](../../../tests/host/lcd_key_input_test.c) | 按键输入对无效/过期ADC样本的撤销 |
| [lcd_spi_timing_test.c](../../../tests/host/lcd_spi_timing_test.c) | LCD生产SPI配置/片选边界，非实测波形 |
| [message_freshness_test.c](../../../tests/host/message_freshness_test.c) | 话题快照、时间零点/边界/回绕、容量 |
| [motor_profile_test.c](../../../tests/host/motor_profile_test.c) | 电机任务按固件profile隔离，驱动控制为计数替身 |
| [param_operation_test.c](../../../tests/host/param_operation_test.c) | 参数Flash维护保护和RAM旧值保留，mmap模拟Flash |
| [quaternion_ekf_inv_sqrt_test.c](../../../tests/host/quaternion_ekf_inv_sqrt_test.c) | 生产EKF反平方根路径有限性与模长回归，非完整姿态精度 |
| [remote_command_integration_test.c](../../../tests/host/remote_command_integration_test.c) | 生产UART/遥控/安全/命令链路、输入租约与来源交接，视觉为快照替身 |
| [remote_control_test.c](../../../tests/host/remote_control_test.c) | DBUS合法帧、字段/序号、事件/操作映射、键鼠租约与裸板隔离 |
| [robot_cmd_task_test.c](../../../tests/host/robot_cmd_task_test.c) | 生产命令来源切换/失效撤销、发射和故障；双板CAN为替身 |
| [robot_safety_test.c](../../../tests/host/robot_safety_test.c) | 整车故障优先级、当前启动初始化锁存和恢复策略 |
| [shoot_freshness_test.c](../../../tests/host/shoot_freshness_test.c) | 发射初始化、命令过期停机和新命令恢复，电机替身 |
| [task_monitor_shutdown_test.c](../../../tests/host/task_monitor_shutdown_test.c) | 任务失效先关断、喂狗停止、有限维护窗口与等待 |
| [tim_profile_test.c](../../../tests/host/tim_profile_test.c) | TIM引脚/加热配置及重初始化隔离 |
| [usart_snapshot_test.c](../../../tests/host/usart_snapshot_test.c) | UART注册失败回滚、不可变派发快照与原始时间 |
| [vcp_queue_freshness_test.c](../../../tests/host/vcp_queue_freshness_test.c) | USB队列不可变快照与过期；CDC发送为替身 |
| [vision_limits_test.c](../../../tests/host/vision_limits_test.c) | 机械范围、非有限值、单位转换、输入/输出失败 |
| [vision_protocol_test.c](../../../tests/host/vision_protocol_test.c) | SP帧解析与真实CRC向量、视觉期限及发射来源选择 |
| [watchdog_debug_safety_test.c](../../../tests/host/watchdog_debug_safety_test.c) | 看门狗调试配置、独立于HAL tick的有界等待 |

入口及检查器：[tests/host/CMakeLists.txt](../../../tests/host/CMakeLists.txt)、[tests/host/test_check.h](../../../tests/host/test_check.h)、[tests/host/test_check_test.c](../../../tests/host/test_check_test.c)、[tests/host/expect_check_failure.cmake](../../../tests/host/expect_check_failure.cmake)。公共夹具：[tests/host/bmi088_host_fixture.c](../../../tests/host/bmi088_host_fixture.c)、[tests/host/bmi088_host_fixture.h](../../../tests/host/bmi088_host_fixture.h)。工具测试：[tests/tools/test_dt7_input.py](../../../tests/tools/test_dt7_input.py)、[tests/tools/test_probe_snapshot_live.py](../../../tests/tools/test_probe_snapshot_live.py)。

替身的覆盖边界必须随功能任务保留：CANComm测试使用加和CRC替身不证明真实CRC；KF初始化的矩阵替身不证明数值正确；USB业务队列中发送总成功替身不证明真实CDC缓冲寿命；空临界区不证明并发安全；寄存器模型不证明实际波形。需要时为已选行为补生产接口回归，不能把此项扩成整车模拟／回放平台。

### tools：13个文件用途与最终处理

| 文件 | 用途 | 已选后续处理 |
|---|---|---|
| [tools/dt7_input.py](../../../tools/dt7_input.py) | 裸板DT7键鼠转发；list/replay/live，输入状态、心跳、释放、日志及设备身份校验 | 保留已验收MC02-0049；UI标题COM4固定，应显示实际端口；180s只适用于replay，勿声称live已有总时限；其两帧全松开恢复规则与新版USB业务无强制握手是不同来源规则 |
| [tools/examples/dt7_bench_replay.json](../../../tools/examples/dt7_bench_replay.json) | 18秒固定键鼠回放输入 | 保留独立裸板诊断样例；不赋予执行器运行许可 |
| [tools/probe_snapshot_live.py](../../../tools/probe_snapshot_live.py) | 离线ELF解析+运行中SWD白名单RAM读数，输出性能或诊断会话JSONL | 先核对Build ID、身份布局及RAM边界；保留有限读取/逐次running检查/loopback；动态逐字段读取并非原子整车快照，归MC02-0100 |
| [tools/openocd_running_inspection.cfg](../../../tools/openocd_running_inspection.cfg) | H723+CMSIS-DAP运行中观测服务配置 | 保持关闭GDB/Tcl、禁止控制/写入命令，按选中后端分离配置，不宣称其他硬件已经验证 |
| tools/probe_snapshot.py（已退役） | 旧性能探针、任务占用率、CAN/LCD统计 | 已由 `tools/probe_snapshot_live.py --performance` 替代；不扩 R11。详见 [MC02-0100记录](../../workflow/tickets/MC02-0100.md)。 |
| tools/rtt_dump.py（已退役） | 旧OpenOCD读取并消费RTT上行缓冲 | 已改用 `tools/rtt_monitor.py` 与 OpenOCD 原生RTT服务；客户端只接收，不改写RTT环形缓冲。 |
| [tools/vision_sim.py](../../../tools/vision_sim.py) | SP视觉帧串口发生器 | 始终禁火，明确单位／端口及发送期限；支持 SP 与新版协议最小测试帧，归0101。 |
| [tools/usbcan_serial_monitor.py](../../../tools/usbcan_serial_monitor.py) | 观测特定USB-CAN私有串口输出，统计ID和帧率 | 只维护已核对私有格式，补类型／长度／DLC／ID及有界读取，不新增主动收发，归0102。 |
| [tools/lcd_font/gen_cn_font.py](../../../tools/lcd_font/gen_cn_font.py) | 按固定标签集生成16×16中文字模与枚举 | 参数化字体／输出路径，先验证再交付整组，保留枚举和字节约定，归0102。 |
| [tools/workflow/check_todo.py](../../../tools/workflow/check_todo.py) | 校验未完成任务格式、稳定ID重复、分类/状态与验收条件 | 复用既有TODO契约，不建设第二任务状态源 |
| [tools/workflow/check_firmware_profile.py](../../../tools/workflow/check_firmware_profile.py) | 按ELF符号检查profile与执行器隔离 | 当前robot强制底盘/云台/发射均在，需按车型启用模块产生期望；保留bench禁执行器/诊断身份检查及编译链接证据边界 |
| [tools/workflow/check_cmake_regeneration.py](../../../tools/workflow/check_cmake_regeneration.py) | 在独立副本验证增删移源码/头目录后的增量与干净配置一致 | 保留现有自动回归，适配配置选源码；新 CubeMX 自动检查后置，不删除已有检查。 |
| [tools/workflow/check_exercises.py](../../../tools/workflow/check_exercises.py) | 课程结构、局部文档链接检查及可选临时目录CMake/CTest | 保留教学流程、主机环境和本地链接校验；不宣称覆盖全仓文档；不建设整车模拟回放 |

### 文档、CI、依赖和资源

`docs/` 的106项包括：导航1、agents3、ADR1、workflow28、development1、学习笔记25、课程38、硬件定位5、根层历史记录4。另核对根 README／GLOSSARY／AGENTS／LICENSE／skills-lock、CI和61个资源的用途。当前文档与历史证据须标适用版本；笔记07的票据相对链接需随0085修正，第三方技能中的示意路径不是项目缺失文件。

第三方先锁基线：HAL1.11.1、FreeRTOS10.3.1、RTT7.58c，CMSIS／USB按实际文件来源和指纹记录，系统库整体升级另行安排。FreeRTOS `ARM_CM4F` 目录名本身不能判定 M7 不兼容，需核对内核修订、FPU和中断约束。平台后端持有厂商依赖，普通业务不继承其头路径。

DSP固定 [CMSIS_5 5.7.0来源](https://github.com/ARM-software/CMSIS_5/tree/a65b7c9a3e6502127fdb80eb288d8cbdf251a6f4/CMSIS/DSP)，DSP1.7.0；现用 Middleware头与该来源归一换行后匹配，旧预编译.a仅可核对ABI，不能据此认定源码版本。后续0099选取6个matrix f32函数（init/add/sub/mult/trans/inverse）、sin/cos和common_tables共9个C文件，目标级启用矩阵检查及匹配的sin表配置；固件不启用fast-math。求逆会改输入工作区，失败结果须通过独立工作区隔离，不能仅因描述符为const就当输入不变。本轮只核对可行性，未构建该新后端。

### 六份业务配套资源

这些文件不占第1–218项的.c/.h/.md编号，仍有用途记录；原资料保留，不凭截图认定型号／量纲已确认。

| 文件 | 用途及限制 |
|---|---|
| [BMI088数据手册](../../../modules/BMI088/bmi088-datasheet.pdf) | Bosch 数据手册，当前附本修订1.8、2022年1月；用于现有采样／换算核对，不恢复R27档位或多实例驱动扩展 |
| [HT控制报文](../../../modules/motor/HTmotor/控制报文.png) | MIT位打包、反馈和特殊命令参考；前馈量纲须与实际固件核对 |
| [HT驱动器说明](../../../modules/motor/HTmotor/驱动器硬件说明.pdf) | HT-04硬件与控制参考；与截图前馈电流／力矩表述不同，归0073待设备确认 |
| [LK协议V2.3](../../../modules/motor/LKmotor/LK-TECH电机CAN协议说明V2_3.pdf) | 单电机／多电机命令、状态和参数读写参考，归0074匹配实际型号／固件 |
| [LK反馈报文](../../../modules/motor/LKmotor/反馈报文.png) | 反馈字段、编码器及电流量程截图，不能推广为所有型号 |
| [LK多电机报文](../../../modules/motor/LKmotor/报文格式.png) | 多电机组帧、ID及模式互斥参考，不单独充当新版设备协议 |

### 独立资料与裁判模拟器

| 工作区 | 审查基线与用途 | 覆盖限制及后续 |
|---|---|---|
| [厂商资料](../../../.Doc/资料/README.md) | 独立Git `0ad8a919495c6da38f36b9352ca1118e490ef4c2`；4942个跟踪文件，2D图纸6、3D模型887、README3、例程4037、图片3、说明书6；共18套例程、10份PDF | 按资料分组及相关硬件／协议证据查阅，不逐行验全部例程或检验全部CAD模型。USAGE保留旧目录路径，入口以实际目录和原文为准；不在本次批量改写例程、模型或重命名目录 |
| [裁判模拟器](../../../tools/referee_sim/README.md) | 独立Git `392a0ba81f9950784a78d419226e8c47b07a8998`；49个跟踪文件，协议／串口／GUI／场景／测试按用途核对 | 本轮未运行GUI／串口／MQTT或全套测试；已有场景可复用，但README旧固件描述与串口写时限／短写需对齐，归0104及[REFSIM-0001](../../../tools/referee_sim/TODO.md) |

厂商例程包括 ADC、ALL、BUZZER、CAN、DBUS、FDCAN、IMU、IMU_TempCtrl、KEY、LCD、MPU、OV5640、RS485、TIM、UART、USB、W25Q64、WS2812。例程或 OSPI 初始化存在不证明用户板上已装对应存储器；LOG-06归档介质仍待补。核对的官方V1.1手册及V1.0完整原理图未列板载磁力计，IST8310保留外接扩展；未据此实测用户PCB修订或外接线。

裁判唯一基线为本地资料中的官方2026 V2.0.0（2026-06-26，SHA256 `8fdc484c58d503a3fb68f7c0515ae618b68eb2b058a8c876e1add5eeb8a7d81b`）。其PDF是资料仓库内未跟踪的本地文件，不复制进主仓库。模拟器与固件共享已核对字节向量，双方各自解析／生成不能单独充当正确性证明；裸板验证保持禁热、禁火、不连接执行器。本次不新增模拟器GUI／CLI／MQTT功能。

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
| [上级LibXR](../../../../libxr-upstream/) | 借鉴system／driver分层与能力边界 | 核对基线 `55c20b41cce6a6af82772b5d8427a172fa650013`；本工程保留C实现，不导入其C++20代码或直接暴露RTOS类型。 |
| [上级XRobot](../../../../XRobot/) | 借鉴模块依赖与逻辑资源绑定 | 核对基线 `15bff04ec4a21c2fdb31d4411e2802851e3de4c9`；只借鉴接口思路，不新增其代码生成或C++框架依赖。 |
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
- CyberGear 具体型号／固件及官方协议，LK／DM／HT 实际协议版本与量程。DR 与步进尚无实际目标；总线舵机随R22保留后续，不作为本轮新增实现的待补前置。
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
