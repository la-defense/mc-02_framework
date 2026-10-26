# BSPMISC-06：WS2812 与 ADC 引脚资源检查

本记录汇总引脚冲突的源码依据、修复和软件验收。TODO 状态只以[本地任务表](../../../.Doc/TODO.md)为准。

## 冲突与修复

- `Core/Src/adc.c` 将 ADC1 通道19配置到 PA5；`bsp/bsp_init.c` 启动 ADC1 的循环 DMA 采样。
- 原 `bsp/ws2812/ws2812.c` 同时将 PA5、PA7配置为 SPI6 复用功能。若调用 `WS2812_Init()`，PA5 会离开 ADC 模拟输入状态。
- 现 `WS2812_SPI6_Init()` 只配置 PA7 为 SPI6 MOSI。SPI6 的移位时钟留在外设内部，不把 SCK 输出映射到 PA5，因此按键采样仍占用 PA5。
- 当前没有生产代码调用 `LEDInit()`；修复同时覆盖以后显式接入该入口的情况。

## 验收证据

- 新增主机测试直接链接生产 WS2812 驱动，记录 HAL GPIO 配置。修改前测试因引脚掩码包含 PA5 而失败；修改后确认只配置 PA7，并确认重复初始化不重复设置时钟、SPI和GPIO。
- WSL Debug 与 Release 主机测试各 **419/419** 通过。
- ARM 固件使用 GCC 10.3.1；bench_safe、rcdiag、robot single/chassis/gimbal 的 Debug／RelWithDebInfo 共 **10种** 构建通过，WS2812 驱动均参与编译。
- 构建观察到 `modules/alarm/led.c` 未包含声明 `malloc()` 和 `abs()` 的标准库头文件，仍是警告；这是可选驱动初始化审查 MC02-0051 的后续项，本次未修改该模块。

## 验收边界

本轮使用主机 GPIO 模型和 ARM 编译，不给开发板上电。测试只能证明固件向 HAL 请求的 GPIO 配置不占用 PA5，不能证明真实 ADC 噪声、WS2812 电平或灯珠时序。WS2812 实际波形仍由 MC02-0025 验收。
