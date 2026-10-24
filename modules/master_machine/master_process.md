# master_process

<p align='right'>neozng1@hnu.edu.cn</p>

本模块登记视觉链路、保存合法SP命令、检查来源时限并回传状态。先调用VisionInit()，再通过VisionGetSnapshot()复制命令及状态；发送内容由VisionUpdateTx()准备，VisionSend()追加CRC并尝试发送。公开用法见[master_process.h](master_process.h)，字节布局见[vision_protocol.h](vision_protocol.h)。

## 接收与时间（BSPUART-03U1）

默认使用USB VCP，编译时可选择VISION_USE_UART，同时选择两种传输会报错。SP命令29字节、状态43字节，未改线格式。旧A5编解码已移至[历史参考目录](../../docs/history/seasky/README.md)，不属于当前视觉链路或活动固件构建。

UART和USB回调分别接收只读块，使用共同的VisionProtocolStreamFeed保存29字节候选和每个字节的原通知时间；跨块保存残帧，同块多帧按顺序处理。头或CRC失败后向后移一字节继续找头。缺块、连接变化、显式重启或组帧超过50ms时放弃残帧。

发布命令只在头和真实CRC通过后发生；last_rx_ms取首个组成UART块的原RxEvent通知时间，0也合法。查询和发送都会检查50ms来源期限，边界允许；超时清零命令并撤销有效标记，保留原时间供诊断。离线回调清命令与时间，并请求下一次UART接收重建残帧。DaemonReload不能让旧来源获得新期限。

CRC不能证明字段可用于控制。本模块拒绝非法模式和非有限六轴字段，后续控制仍须检查机械范围及当前许可。USB连接代号失效时，读取立即撤销旧命令；普通任务中的连接通知也不能误清已经收到的新连接首帧。当前模块只支持单个视觉实例。

## 发送与验证

VisionSend使用局部完整候选追加CRC；USB和UART都在提交时复制。USB只允许一份在途内容，忙时拒绝；底层完成或失败由普通任务回调更新结果。接受和本机完成均不表示对端业务已经执行。USB生命周期、100ms发送期限及有限重建见[USB接口](../../bsp/usb/usb_transport.h)和[MC02-0062记录](../../docs/workflow/tickets/MC02-0062.md)。

[流式回归](../../tests/host/uart_stream_test.c)使用真实UART队列和生产解析，检查分割、逐字节、多帧、错误、来源时间及恢复；[CRC协议回归](../../tests/host/crc_protocol_test.c)继续覆盖USB正常路径和CRC失败。完整证据见[BSPUART-03U1](../../docs/workflow/tickets/BSPUART-03U1.md)，本轮没有硬件操作。


## V2 对时与映射（MC02-0090）

V2 开启后，`VisionSend()` 在原状态发送节流之外推进一个单请求探测器：首次可发送时立即发 `0x83`，之后按 500 ms 周期探测，等待回应最多 100 ms。回应 `0x03` 的负载含请求编号和 PC 接收时刻，帧头来源时刻含 PC 发送时刻；MCU 以探测帧来源时间作为 T1，并使用完整回应帧最后字节的接收块时间作为 T4。每个匹配样本要求交换不超过 100 ms、计算 RTT 不超过 50 ms。三个有效样本建立映射，偏差跳变超过 50 ms会重新收集；2秒无有效样本后映射过期。USB代际切换清理旧映射，失效后仍按周期重新探测。

`VisionGetTimeSyncSnapshot()`只复制状态；`VisionMapRemoteTime()`只在同步映射有效时转换远端毫秒，错误保留输出。它不更新消息中心的来源时间，不改变业务帧首次接收时刻和控制新鲜度。SP不含来源时刻，SP构建的对时查询返回不可用。上位机生产实现、物理链路时钟精度和CAN板间对时不属于本记录的验收范围，后者由MC02-0093独立复用契约。

生产路径测试见[vision_v2_link_test.c](../../tests/host/vision_v2_link_test.c)、纯映射边界见[time_mapping_test.c](../../tests/host/time_mapping_test.c)，协议向量见[vision_protocol_v2_test.c](../../tests/host/vision_protocol_v2_test.c)。具体结果和限制记录在[MC02-0090任务记录](../../docs/workflow/tickets/MC02-0090.md)。
