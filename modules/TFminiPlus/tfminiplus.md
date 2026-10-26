# TFmini Plus I²C 模块

本目录保留 TFmini Plus 的可选阻塞式 I²C 入口。`TFMiniRegister()` 登记 7 位地址 `0x10`，`GetDistance()` 发送 `5A 05 00 06 65` 请求并读取毫米数据响应。响应长度统一由 `TFMINI_DATA_FRAME_SIZE` 定义为 9 字节，与实例缓冲区容量相同。

该长度依据 [TFmini Plus 用户手册表 8](https://en.benewake.com/uploadfiles/2025/04/20250430175221028.pdf) 的 I²C 毫米数据格式。主机回归通过生产 `IICReceive()` 路径检查请求长度、短读、完整响应和多余可用输入均不会越过 9 字节数组，并核对距离字段仍按原偏移组合。

`IICReceive()` 当前不返回 HAL 状态或实际接收长度。因此，短帧发生时本模块无法判断哪些字节已更新；`GetDistance()` 也不检查帧头、校验和或距离单位。该驱动仍未验证实际设备通信，启用前还需处理有效帧判定和失败结果；I²C 注册、GPIO 生命周期等其他问题继续由 MC02-0051 跟踪。
