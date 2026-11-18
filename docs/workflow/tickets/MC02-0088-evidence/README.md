# MC02-0088 本机验证

主机用生产周期算法及 FreeRTOS 适配器，替换底层调度时钟和等待调用。覆盖配置不可精确表示、预算大于周期、执行预算与周期截止点分别检查、计数回绕、调度迟到与跳拍、提前唤醒、非法调用上下文。ASan／UBSan 专项通过。实物定时器、调度抖动和运行栈水位不在本机验证范围内。

- `verify.py --fast --host --firmware --presets bench-debug,rcdiag-debug` 通过，主机 Debug／Release 各 806/806，六种正式车型预设按预期拒绝。
- 普通裸板、遥控诊断和 `commissioning/observe` 遥控观察版均通过 ARM 构建及源码、头依赖和 ELF 隔离检查。普通裸板及受限观察版 ELF 中均有周期等待入口。
- 与上一轮同预设构建相比，普通裸板及遥控诊断的 FLASH 各增加 664 B，RAM 用量不变。普通裸板单函数静态栈报告为 `StartROBOTTASK` 64 B、`PlatformTaskPeriodicWait` 56 B、`PlatformPeriodicPoll` 32 B。

## TIM5 独占唤醒

- WSL Debug／Release 全量各 807/807。周期算法和 TIM5 后端两项通过 ASan／UBSan。TIM5 测试用寄存器和 RTOS 替身，覆盖时钟不整除、优先级分组错误、资源占用、静态信号量失败、跨任务访问、计数回绕、先通知后阻塞、无关或重复中断、设置时已迟到、提前通知、丢失中断后的超时及释放后重新申请。
- 普通裸板、遥控诊断与独立 `timertest_bench_safe` 均通过 ARM 构建和隔离检查。独立 ELF 中检查到强定义 `TIM5_IRQHandler` 和四个观测符号，普通裸板只有默认弱定义。六种正式预设仍按既有规则拒绝。额外验证 `commissioning`、同时启用遥控诊断、以及隔离视图中 CubeMX 已配置 TIM5 三种情况均拒绝配置。
- 与同工具链普通裸板相比，TIM5 自测版 FLASH 为 258640 B，增加 2208 B。RAM 为 122232 B，增加 160 B。默认关闭该选项时未引入 TIM5 后端，普通裸板和遥控诊断各内存区用量沿用前一轮。实物计时、中断传递和调度抖动待测。

## TIM5 共享计数

- 主机用生产后端及能够模拟 SR 写 0 行为的寄存器替身。覆盖四通道唤醒、同时到期分别通知、一个通道超时并退出时另一通道保留通知、信号量申请失败、容量用尽、释放后复用，以及首次申请和最后一次释放才改变硬件运行状态。
- `verify.py --fast --host --firmware --presets bench-debug,rcdiag-debug` 覆盖普通裸板、遥控诊断与预期拒绝的正式车型配置。WSL 主机 Debug／Release 各 807/807，两个周期接口专项通过 ASan／UBSan。独立 `timertest_bench_safe` 通过 ARM 构建和隔离检查，ELF 含 TIM5 中断入口及四个诊断观测符号。
- 与同工具链上一轮 TIM5 自测版相比，FLASH 为 259104 B，增加 464 B。RAM 为 122512 B，增加 280 B。实际唤醒时延、抖动和独立车辆任务的执行预算仍需实测。
