# MC02-0088 本机验证

主机用生产周期算法及 FreeRTOS 适配器，替换底层调度时钟和等待调用。覆盖配置不可精确表示、预算大于周期、执行预算与周期截止点分别检查、计数回绕、调度迟到与跳拍、提前唤醒、非法调用上下文。ASan／UBSan 专项通过。实物定时器、调度抖动和运行栈水位不在本机验证范围内。

- `verify.py --fast --host --firmware --presets bench-debug,rcdiag-debug` 通过，主机 Debug／Release 各 806/806，六种正式车型预设按预期拒绝。
- 普通裸板、遥控诊断和 `commissioning/observe` 遥控观察版均通过 ARM 构建及源码、头依赖和 ELF 隔离检查。普通裸板及受限观察版 ELF 中均有周期等待入口。
- 与上一轮同预设构建相比，普通裸板及遥控诊断的 FLASH 各增加 664 B，RAM 用量不变。普通裸板单函数静态栈报告为 `StartROBOTTASK` 64 B、`PlatformTaskPeriodicWait` 56 B、`PlatformPeriodicPoll` 32 B。
