# Daemon 来源监控

<p align='right'>neozng1@hnu.edu.cn</p>

Daemon 用 HAL 毫秒时钟监控通信来源，并把状态变化交给指定任务处理。主要入口在 [daemon.h](daemon.h)；来源模块只在协议校验完成后报告接收结果。

## 在线与数据有效分别判断

每个来源独立保存通信期限、关键数据期限和首帧宽限。期限单位都是毫秒，范围不超过 `INT32_MAX`；0时刻有效，跨一次毫秒计数回绕用无符号时差计算。时间恰好等于期限时仍有效。

来源状态按以下顺序判断：

| 状态 | 条件 |
| --- | --- |
| `DAEMON_SOURCE_WAITING_FIRST_FRAME` | 还没有合法通信帧，且首帧宽限未到期 |
| `DAEMON_SOURCE_ONLINE_DATA_VALID` | 通信帧和关键数据都在各自期限内，且数据有效 |
| `DAEMON_SOURCE_ONLINE_DATA_INVALID` | 通信仍在线，但没有关键数据、数据无效或数据已过期 |
| `DAEMON_SOURCE_OFFLINE` | 首帧宽限或通信期限已过 |

接收模块用 `DaemonReportAt()` 传入组成数据的原始 RxEvent 毫秒时间。协议合法时设置 `DAEMON_REPORT_TRANSPORT_VALID`；当前帧含关键数据时再设置 `DAEMON_REPORT_DATA_PRESENT`，数值和语义检查通过后才设置 `DAEMON_REPORT_DATA_VALID`。协议错误不刷新通信时间，旧接收时间也不能让已经超时的来源重新上线。

`DaemonGetSourceStatus()` 只复制当前状态、最近接收时刻、年龄和期限，不改变来源时间，也不产生事件。需要再次接收合法帧才能刷新相应期限。

## 状态变化由来源任务处理

`DaemonTask()` 周期检查来源并把状态变化放入每来源固定容量队列；它不调用模块回调。每个实例只配置一个消费者，可在 `Daemon_Init_Config_s` 中选择绑定任务或独立事件任务。

| 策略 | 队列满时 | 适用行为 |
| --- | --- | --- |
| `DAEMON_EVENT_FIFO` | 拒绝新事件并累计溢出数 | 保留每次状态变化的先后次序 |
| `DAEMON_EVENT_COUNT` | 把相同来源的变化计入一条记录 | 保留首次旧状态、最新状态、次数和最后检测时间 |
| `DAEMON_EVENT_COALESCED_STATE` | 用最新状态替换未处理状态 | 只关心当前状态时使用 |

事件超过 `event_max_age_ms` 后会丢弃并计数。FIFO 和合并结果都通过 `DaemonProcessTaskEvents()` 在所属任务中交付；MotorTask 和 RobotCMDTask 分别处理自身来源。独立模式由 `DaemonEventTaskInit()` 创建 1ms 轮询任务，每轮最多处理8条，任务创建失败可查询并重试。事件回调应短小，不等待；恢复设备的步骤在回调所属模块中执行。

实例只能在启动期串行注册。监控或事件消费开始后不允许注销，以免队列处理期间释放仍在使用的实例。来源模块必须一直保存成功注册的实例；运行期间不再分配。

## 验证范围

[主机生产测试](../../tests/host/daemon_source_monitor_test.c)覆盖期限边界、时钟回绕、旧帧拒绝、通信与数据有效分离、队列策略、溢出、事件过期、消费者路由、任务创建失败重试和并发读写。MC02-0071 的 WSL Debug／Release 各413项通过，专项 ASan／UBSan 11项通过，六种 ARM 禁热固件和增量构建检查通过，见[任务记录](../../docs/workflow/tickets/MC02-0071.md)。这些软件测试不替代实物通信或设备恢复验证。
