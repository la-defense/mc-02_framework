# MISC-03：TFmini 接收帧长度

本记录说明 TFmini Plus I²C 响应长度与成员缓冲区不一致问题的修复范围和验证证据。模块仍是可选项，本轮没有连接或启用传感器。

## 实现

- TFmini Plus 用户手册表 8 规定，`0x5A 0x05 0x00 0x06 0x65` 毫米取数命令返回 9 字节。依据：[TFmini Plus 用户手册](https://en.benewake.com/uploadfiles/2025/04/20250430175221028.pdf)。
- 在 `tfminiplus.h` 定义 `TFMINI_DATA_FRAME_SIZE`，缓冲区和 `GetDistance()` 的 `IICReceive()` 调用共同使用该值，避免容量与请求长度分开维护。
- 距离、强度及模式字段的现有偏移和取数命令保持不变。

## 主机验证

`tests/host/iic_registration_test.c` 的 `tfmini-frame` 场景调用生产 `GetDistance()`、`IICTransmit()`、`IICReceive()` 与 `IICRegister()`，只替换 HAL。模型覆盖 3 字节短响应、完整 9 字节响应以及源端有 11 字节可读的数据；检查 HAL 收取长度始终为 9、完整响应仍解析为距离 `0x1234`／强度 `0x5678`／模式 `0x9A`，并在数组紧邻的结构体填充处检查哨兵未变。

- WSL Debug：全量 CTest 416/416 通过。
- WSL Release：全量 CTest 416/416 通过。
- AddressSanitizer／UndefinedBehaviorSanitizer：I²C 注册、调用方失败和 TFmini 长度场景 4/4 通过。
- `production_tfmini_fixed_frame_bounds` 设置 10 秒超时，并在 Release 中通过始终执行的 `TEST_CHECK` 检查。

## 固件与边界

bench_safe、rcdiag、robot-single、robot-chassis、robot-gimbal 的 Debug 和 RelWithDebInfo 共 10 种 ARM 预设均已重新配置并构建。各配置的编译数据库、`build.ninja` 与 ELF 符号均未包含 TFmini；固件仍未启用该可选模块。

`IICReceive()` 不报告 HAL 状态或实际接收长度，所以本轮只证明接收最多写 9 字节，不证明短响应有效，也没有补充帧头、校验、距离单位、设备时序或物理传感器测试。启用前仍需明确短帧处理；注册及 GPIO 生命周期等可选驱动检查继续由 MC02-0051 跟踪。
