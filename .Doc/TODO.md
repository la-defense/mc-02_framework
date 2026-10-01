# Work To be done & optimized

- **待完成**：不完成可能导致整车功能不完整
- **待优化**：对已有的功能进行性能提高/模块解耦/可维护性增强
- **待添加**：不紧急的/锦上添花的功能

**==标为黄色高亮的代表紧急程度高。==**

## 2026-09-30 复审新增待办

详细证据、影响和验收方法见 [工程复审_2026-09-30](../docs/工程复审_2026-09-30.md)。以下编号只记录本轮新发现；既有问题继续在 [代码审查报告第 16 节](../docs/代码审查报告_MC-02.md#16-修复-todolist) 跟踪。

### P1：上车前关闭

- [x] **R25-01** USB CDC 发送入口检查 class 指针与设备配置态；上电不接 USB、延迟枚举、拔插均不得崩溃。已加入判空/配置态检查并完成固件构建、烧录验证。
- [ ] **R25-02** [bug] [ready-for-agent] 视觉接管增加在线、有效帧年龄、有限值和机械范围判定；mode 2 后断链立即停止发射与接管。验收：主机测试覆盖断链、过期、NaN/Inf 和范围外角度，实机台架确认失联后输出失能。
- [ ] **R25-03** [bug] [ready-for-agent] 遥控回调仅接受完整且合法的 18 字节帧，通过校验后才喂 daemon 和置在线。验收：短帧、噪声、错误校验帧不能解除急停，合法帧可恢复在线状态。
- [ ] **R25-04** [bug] [ready-for-human] 关键电机反馈超时后停止相关输出并上报故障。验收：逐路断开关键 CAN 反馈，相关输出在规定超时内停止且故障可读取。
- [ ] **R25-05** [bug] [ready-for-agent] 将电机离线、CAN BusOff、倾斜等已定义故障位接入 RobotSafety 的 READY/失能决策。验收：每个故障位均有状态转换测试，故障存在时不能进入 READY。
- [ ] **R25-06** [bug] [ready-for-agent] 每周期明确计算发射总使能，覆盖手动、键鼠、视觉切换及视觉失联。验收：mode 2→1→0、控制源切换和断链测试均无残留开火使能。验证：`VisionControlBuildShootCommand` 已在无效模式、过期帧和断连时清除开火、摩擦轮与装填命令；主机回归测试通过。视觉对应任务为 [VIS25-0005](../sp_vision_25-upstream/TODO.md)；仍需 GitHub CI 和台架验证完整模式切换。
- [ ] **R25-07** [bug] [ready-for-agent] 底盘跟随计算改用 fabsf，限制角速度和四轮速度；复核小陀螺 4000 deg/s 定值（与既有 APP-08 合并）。验收：边界及非有限输入测试通过，输出不越过配置限值。
- [ ] **R25-08** [bug] [ready-for-human] 从实车配置加载俯仰机械限位并执行钳位，拒绝非法视觉角度（与既有 APP-10 合并）。验收：配置边界、超界、缺失配置均有测试，台架角度不越机械限位。
- [ ] **R25-15** [bug] [ready-for-human] 将裁判功率、枪口热量和弹速限制接入底盘与发射输出；裁判数据缺失时使用保守限值。验收：模拟器覆盖正常、超限和失联数据，台架确认不超功率且不能非法击发。

### P2：稳定性与结构

- [x] **R25-09** USB 异步发送使用完成回调或双缓冲，检查 `USBD_BUSY` 并统计丢帧；发送期间不得改写正在使用的缓冲（与既有 VISION-07 合并）。CDC 忙时拒绝提交，改用 CDC 持久缓冲，并增加成功/丢帧计数；已构建、烧录验证。
- [ ] **R25-10** [bug] [ready-for-agent] 初始化失败返回可诊断状态；注册器检查容量和 malloc 结果，避免关中断阶段无限循环或越界写。验收：故障注入测试覆盖容量耗尽和分配失败，初始化返回可诊断错误且系统不挂死。
- [ ] **R25-11** [bug] [ready-for-agent] 为消息增加接收时间与过期策略，对 CANComm、消息中心和视觉统一执行超时失能；合并 CANComm 未更新数据检查。验收：主机测试证明过期输入不能维持旧控制，在线新数据可恢复。
- [ ] **R25-12** [enhancement] [ready-for-human] 按 Debug 构建主 RAM 81.16% 使用率复核静态缓冲与各任务栈余量，并结合最坏耗时拆分电机任务中的协议解析。验收：提交内存及栈水位测量、最坏耗时数据和明确的余量结论。
- [ ] **R25-13** [enhancement] [ready-for-agent] CI 增加断链、短帧、越界角度、发射模式切换和运动学主机测试，并清理本轮 Debug 构建告警。验收：GitHub Actions 在 main 与 PR 上构建固件并运行全部新增主机测试。
- [ ] **R25-14** [bug] [ready-for-agent] 审核跨任务共享的命令、传感器状态和消息中心队列，按生产者/消费者选择临界区、队列或双缓冲，并覆盖既有并发检查条目。验收：共享数据清单逐项有同步策略，主机或台架测试验证抢占时读写一致。

## assorted

并发核对条目已合并到 R25-14；完成判据与该任务共用。

**并发说明修正**：普通 `bool` 的“检查再赋值”不是原子操作，不能作为互斥锁。任务间按需求使用 FreeRTOS 互斥量/队列或短临界区；中断与任务间使用中断安全 API、受控临界区或单生产者单消费者缓冲，并明确内存可见性与队列满时的策略。中断中不能等待互斥量。

目前，我们在bsp_dwt中添加了一个位锁，防止中断中调用DWTGetDeltaT或DWTGetTimeline函数更新DWT维护的时间时打断任务中的相同函数导致计数被重复更新，或引起错误的DWT溢出检测。

## BSP

### 待完成



### 待优化

#### bsp_pwm

- [x] 是否允许修改预分频计数器？

### 待添加

#### bsp_iic

- [ ] **MC02-0001** [enhancement] [needs-triage] 为 BSP I2C 增加 10 位地址支持。验收：地址编解码符合 HAL 接口要求，覆盖 7 位兼容和 10 位设备测试。

#### bsp_blueteetch

- [x] 增加蓝牙功能，方便调试和测试

#### bsp_wifi

- [ ] **MC02-0002** [enhancement] [needs-triage] 增加无线局域网功能，方便调试和测试。验收：先确定模块、协议和调试场景，再提交连通测试及断连恢复验证。

---

## Module

### 待完成

Unicom

- [ ] **MC02-0003** [enhancement] [needs-triage] 为键鼠、遥控器、PS 手柄和视觉上位机提供统一控制输入接口，转换为标准底盘速度与云台角度。验收：各输入适配器产生同一领域命令类型，现有控制模式有迁移说明和输入回归测试。

- [ ] **MC02-0004** [enhancement] [needs-triage] 统一模块调试日志开关与运行时等级策略。验收：明确编译期与运行时策略，日志关闭时不产生格式化开销，分级输出测试通过。

#### ==referee==

- [x] 裁判协议适配官方《RoboMaster 2026 机甲大师高校系列赛通信协议 V2.0.0（20260626）》。
  当前 `modules/referee/referee_protocol.h` 仍为旧版结构体，需同步更新：
  - 0x0003：32B → 20B（双方血量改为己方/对方前哨站与基地）
  - 0x0201：13B → 17B（新增 bullet_speed_limit float 及电源输出位域后移）
  - 0x0202：16B → 14B（移除一路 17mm 热量）
  - 0x0203：16B → 12B（x/y/angle，无 z/yaw）
  - 0x0204：6B → 8B（新增 remaining_energy）
  - 删除已废弃的 0x0102 场地补给站动作标识
  并补上 0x0104/0x0105 与 0x0208~0x020E、0x0303 的解析；
  同步修改 `rm_referee.c` 解析长度与 `referee_UI.c` 使用到的字段；
  协议实现可参考 `tools/referee_sim`（独立仓库）中的官方字段定义与测试。
  （2026-08-09：结构体/解析/编译已完成，静态断言核对 21 个结构体长度通过；
  已烧录并用模拟器 + GDB 上电验收：18 条官方命令全字段核对一致，CFSR=0）

#### ==servo_motor==

舵机模块，需要预先定义90/180/360连续旋转的电机类型，并且能够设定max和min位置。

- [x] 编写舵机模块（待测试和优化）
- [x] 可能需要串口舵机的支持？

#### imu

- [ ] **MC02-0005** [enhancement] [needs-triage] 完善 BMI088 与姿态算法交互，评估并实现 SO(3) 上的异步量测更新 IEKF。验收：算法决策有 ADR，回放数据覆盖异步更新且与基准姿态误差有对比。

> 继续使用四元数（S3）也许是一个更好的选择，改动较小且运算开销也较小。后续考虑修改一套ESKF_INS并移植到框架中。

#### ==master_machine==

- [ ] **MC02-0006** [enhancement] [ready-for-agent] 为 master_machine IMU 数据增加采样时间戳。验收：时间戳随新样本更新，消费者可检测重复、乱序和超期数据。
- [ ] **MC02-0007** [enhancement] [ready-for-agent] 将加速度计数据纳入 master_machine 数据结构与传输。验收：字段单位、坐标系和更新频率有定义，模拟数据可端到端读取。
- [ ] **MC02-0008** [enhancement] [needs-triage] 重构 SeaSky protocol 接口。验收：明确兼容范围与目标报文后，协议层可独立解析、校验并通过主机测试。
- [ ] **MC02-0009** [bug] [ready-for-agent] 为 master_machine 处理数据未更新情况。验收：过期或缺失样本不能继续驱动旧控制，恢复新样本后在线状态正确恢复。

需要一个更简单的协议以加快速度。若有必要，可能需要重新编写一个简单的调试上位机UI。


### 待优化

#### buzzer

> 是否需要在module层就和**daemon**模块配合？

当前实现为buzzer是单独的module，若需要蜂鸣器警报的module可以自行包含buzzer.h以创建不同情况下的警报，如电机离线、堵转、遥控器离线等。

也许还有其他方式提醒离线和异常。

目前急需一个无线遥控继电器，防止机器人的急停模式失效。

#### BMI088

- [ ] **MC02-0010** [enhancement] [needs-triage] 完善 SO(3) IEKF 与 INS 任务交互，评估由 IMU 中断唤醒异步任务的数据路径。验收：任务时序图和队列策略经审查，台架测得样本时间与处理时间一致。

根据BMI088的datasheet在初始标定完成后将gyro和acc都设置为中断触发，当数据准备好时传感器会在对应的引脚输出跳变，通过EXTI捕获跳变并触发中断，在回调函数中启动SPI DMA传输。陀螺仪数据来到时进行姿态的预测即propagation，加速度计数据到来时进行量测更新（correct）。

也许需要找到一种更好的方式构建INS任务以方便和其他模块、应用的交互。

另外，若要进一步提升自瞄效果，在姿态得到更新时（陀螺仪和imu的数据到来时），需要额外的引脚连接到相机上完成硬触发采集，以获得更好的时间对齐效果，防止视觉得到的姿态数据发生漂移。目前视觉端假设姿态更新的频率是1khz（当前每次完成姿态解算都会向上位机发送当前的姿态）

#### remote_control

- [ ] **MC02-0011** [enhancement] [needs-triage] 评估并实现遥控器按键长按、短按检测。验收：先确定实际控制用例，再以边界时长测试区分点击与长按。

#### message_center

- [ ] **MC02-0012** [enhancement] [ready-for-agent] 为 message_center 增加队列余量与消息时间戳查询。验收：队列空、满及过期场景返回准确状态，不改变既有值传递行为。
- [ ] **MC02-0013** [enhancement] [needs-triage] 评估 message_center 直接传递指针的接口。验收：明确所有权、生命周期、队列满和并发规则，并以测量证明收益后再决定是否实现。

#### can_comm

- [ ] **MC02-0014** [bug] [ready-for-agent] 为 CANComm 增加数据未更新处理。验收：超时反馈使对应命令进入安全输出并报告故障；有效帧恢复后按安全状态机恢复。

#### controller

- [ ] **MC02-0015** [enhancement] [needs-triage] 评估将 PID 初始化改为 PIDRegister 并由 controller 统一分配内存。验收：确定注册失败处理、容量上限和实例生命周期，迁移后 PID 行为回归测试通过。

#### dji_motor

- [ ] **MC02-0016** [enhancement] [needs-triage] 增加 3508 和 2006 电机开环零位校准函数。验收：校准流程记录零位结果，方向和重复性在台架上通过验证。
- [ ] **MC02-0017** [enhancement] [ready-for-agent] 为 DJI 电机实例增加独立低通滤波系数配置。验收：不同实例可配置不同系数，默认行为不变且滤波响应有单元测试。

#### LKmotor

- [ ] **MC02-0018** [enhancement] [ready-for-agent] 为 LK 电机增加正反转标志并统一反馈量与 PID 方向。验收：正转、反转和零速测试方向正确，既有默认方向兼容。

#### HTmotor

- [ ] **MC02-0019** [enhancement] [ready-for-agent] 为 HT 电机增加正反转标志并统一反馈量与 PID 方向。验收：正转、反转和零速测试方向正确，既有默认方向兼容。

### 待添加

#### unicomm

- [ ] **MC02-0020** [enhancement] [needs-triage] 明确“初版构建”对应的目标板、配置和交付物并完成构建基线。验收：干净检出后按文档命令生成目标 ELF/HEX/BIN，记录工具链版本及内存占用。

#### step_motor

- [ ] **MC02-0021** [enhancement] [needs-triage] 增加步进电机模块。验收：明确驱动器接口、步进速率、加减速和急停约束后，模块仿真及板级脉冲测试通过。

#### referee_communication

- [ ] **MC02-0022** [enhancement] [needs-info] 增加裁判系统多机通信功能。验收：取得比赛协议支持依据与目标拓扑后，模拟器覆盖完整收发和异常帧。

#### controller

- [ ] **MC02-0023** [enhancement] [needs-triage] 评估扰动观测器控制器及其模块边界。验收：仿真数据证明相较现有控制器的收益，噪声和饱和工况无不稳定输出。
- [ ] **MC02-0024** [enhancement] [needs-triage] 评估基于模型的控制器及其模块边界。验收：模型假设、算力预算和仿真性能对比经审查后再确定实现范围。

#### ws2816

- [ ] **MC02-0025** [enhancement] [needs-triage] 评估通过 BSP PWM 支持 WS2816。验收：确认器件时序与 DMA/定时器资源后，逻辑分析仪测得波形满足器件规格。

---

## APP

### 待完成

#### all app

- [ ] **MC02-0026** [enhancement] [needs-triage] 增加独立应用调试配置，在未连接其他应用时允许使用模拟输入调试。验收：模拟依赖与真实依赖可切换，调试配置不会进入实机默认构建。

#### ==shoot==

- [ ] **MC02-0027** [enhancement] [needs-info] 增加卡弹检测与安全反转。验收：明确传感器证据、反转时长、能量限制及禁止条件后，模拟器和无弹台架测试通过。

### 待优化

#### robot_cmd

- [ ] **MC02-0028** [enhancement] [ready-for-agent] 优化 robot_cmd 消息发布和接收性能。验收：先记录现有周期耗时和复制量，优化后行为回归通过且最坏耗时有可复现改善。

#### gimbal

- [x] 增加底盘速度前馈控制

#### chassis

- [x] 根据电机的实际速度计算底盘的真实运动（轮式里程计）
- [ ] **MC02-0029** [enhancement] [needs-info] 若采用双板架构，根据 IMU 数据融合电机实际速度。验收：确认板间数据来源、时钟同步和融合算法后，以回放数据验证误差和延迟。

## 2026-10-01 本机 WSL 自瞄联调新增待办

- [x] **HIL-01 / P0** BIOS VT-x 已启用（HypervisorPresent=True）；WSL 已更新并设默认版本 2，Ubuntu 22.04.5 LTS 已安装并以 WSL 2 运行。
- [x] **HIL-02 / P0** 官方仓库已克隆到 WSL Linux 文件系统 /home/mc02/src/sp_vision_25，commit bd9f5e798fa3c6dd3b483ae6627796afb41c608d；联调 overlay 已应用，个人 fork 未修改。
- [x] **HIL-03 / P0** OpenVINO 路径支持 CMake cache 配置，WSL CPU preset 已成功构建 standard_mpc、auto_aim_test、bareboard_hil。
- [x] **HIL-04 / P0** 裸板联调入口已支持 demo/camera/bridge、空闲模式、演示四元数/MC-02 四元数；瞄准帧固定 fire=false，无目标、失联、退出发中立帧。demo 实测 300 帧、268 帧目标/瞄准输出。
- [x] **HIL-05 / P0** MC-02 CDC 已 USB/IP 直通 WSL 并解析到持续有效状态帧；工业相机直通时 Linux MVS 枚举报 0x80000006，改用 Windows MVS 桥向 WSL 识别程序传入 100 帧，约 29 fps。
- [x] **HIL-06 / P0** demo HIL 实测 300 帧，其中 268 帧有目标、268 次有限瞄准规划输出；相机桥 100 帧进入识别程序，现场无装甲板时持续撤销控制。
- [x] **HIL-07 / P0** 重插 CMSIS-DAP 后经 OpenOCD/SWD 读取 HIL 前后固件计数：RX 1243→1545（+302），CRC 错误维持 0；增加量与 268 个瞄准帧、32 个中立帧及退出中立帧一致，证明下位机解析成功。断链后视觉超时清零已由固件验证。
- [x] **HIL-08 / P1** 已记录 WSL 依赖、CMake 构建、demo/相机运行、USB/IP 与 MVS 桥命令及测试结果；遗留 CMSIS-DAP 计数复核已记录在部署文档。
- [x] **HIL-09 / P1** Windows COM14 连续监听 5.027 秒收到 1004 个 CRC 正确的 43 字节状态帧（199.73 Hz），同期固件状态帧提交 +1006、USB busy 丢帧不变。WSL HIL 活跃中段 SWD 采样约 200 Hz，busy 计数保持不变；程序退出且无读取者后 busy 计数继续增长，已记录为无人读取时的待优化现象。
- [ ] **HIL-10** [bug] [needs-info] 历史记录显示长时间 OpenOCD 会话出现 CMSIS-DAP USB I/O 错误，短时重连后 RX=0、TX=1；目前没有原始命令、完整错误日志或可重现反馈循环，复位原因未确认。验收：短时 SWD 快照脚本连续读取且不复位、不停核；报告区分探针 I/O 错误、目标复位和计数清零原因。继续前需补充出错会话的 OpenOCD/GDB 命令与原始日志。

## 工程与教学工作流

规格：[工程与教学工作流](../docs/workflow/specs/engineering-learning-workflow.md)。

- [x] **MC02-0030** [enhancement] 把本地 skills、TODO、教学工作区及 C/C++ 练习统一到可验证流程。验收：skill 指引不再写入 `.scratch`，TODO 为唯一状态源，四组离线练习、链接检查和 CMake/CTest 均通过；视觉侧对应 **VIS25-0013**（[视觉 TODO](../../sp_vision_25-upstream/TODO.md)）。验证：TODO contract、课程离线链接和四组 CMake/CTest 主机练习通过。
- [ ] **MC02-0031** [enhancement] [ready-for-human] 建立公开视觉 fork、私有 HIL 仓库和受限 Windows runner，并配置手动双 SHA 联调。验收：runner 只运行仓库内受信任工作流，报告记录固件/视觉 SHA、构建烧录结果、禁止开火帧、RX/CRC 计数和复位状态；视觉侧对应 **VIS25-0014**（[视觉 TODO](../../sp_vision_25-upstream/TODO.md)）。已创建 [公开视觉 fork](https://github.com/la-defense/sp_vision_25) 和 [私有 HIL 仓库](https://github.com/la-defense/mc-02-hil)；runner、手动工作流与报告尚待配置。

