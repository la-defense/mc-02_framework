# master_process

<p align='right'>neozng1@hnu.edu.cn</p>

本模块登记视觉链路、保存合法命令并回传状态。先调用`VisionInit()`，再通过`VisionGetSnapshot()`复制命令及状态；发送内容由`VisionUpdateTx()`准备，`VisionSend()`按所选协议追加CRC并尝试发送。默认SP仍保持29/43字节；V2另含姿态、裁判摘要和IMU加速度状态。公开用法见[master_process.h](master_process.h)，字节布局见[vision_protocol.h](vision_protocol.h)与[vision_protocol_v2.h](vision_protocol_v2.h)。

## 接收与时间（BSPUART-03U1）

默认使用USB VCP，编译时可选择VISION_USE_UART，同时选择两种传输会报错。SP命令29字节、状态43字节，未改线格式。旧A5编解码已移至[历史参考目录](../../docs/history/seasky/README.md)，不属于当前视觉链路或活动固件构建。

UART和USB回调分别接收只读块，使用共同的VisionProtocolStreamFeed保存29字节候选和每个字节的原通知时间；跨块保存残帧，同块多帧按顺序处理。头或CRC失败后向后移一字节继续找头。缺块、连接变化、显式重启或组帧超过50ms时放弃残帧。

发布命令只在头和真实CRC通过后发生；last_rx_ms取首个组成UART块的原RxEvent通知时间，0也合法。查询和发送都会检查50ms来源期限，边界允许；超时清零命令并撤销有效标记，保留原时间供诊断。离线回调清命令与时间，并请求下一次UART接收重建残帧。DaemonReload不能让旧来源获得新期限。

CRC不能证明字段可用于控制。本模块拒绝非法模式和非有限六轴字段，后续控制仍须检查机械范围及当前许可。USB连接代号失效时，读取立即撤销旧命令；普通任务中的连接通知也不能误清已经收到的新连接首帧。当前模块只支持单个视觉实例。

## 发送与验证

VisionSend使用局部完整候选追加CRC；USB和UART都在提交时复制。USB只允许一份在途内容，忙时拒绝；底层完成或失败由普通任务回调更新结果。接受和本机完成均不表示对端业务已经执行。USB生命周期、100ms发送期限及有限重建见[USB接口](../../bsp/usb/usb_transport.h)和[MC02-0062记录](../../docs/workflow/tickets/MC02-0062.md)。

[流式回归](../../tests/host/uart_stream_test.c)使用真实UART队列和生产解析，检查分割、逐字节、多帧、错误、来源时间及恢复；[CRC协议回归](../../tests/host/crc_protocol_test.c)继续覆盖USB正常路径和CRC失败。完整证据见[BSPUART-03U1](../../docs/workflow/tickets/BSPUART-03U1.md)，本轮没有硬件操作。


## V2 IMU 加速度状态（MC02-0006／0007）

V2 新增类型`0x84`，负载固定17字节：有效标记1字节、BMI088样本序号`uint32`小端、传感器x/y/z三个IEEE-754 binary32小端值。有效样本的坐标轴沿BMI088传感器本身，单位m/s²且包含重力；尚未应用机器人安装方向修正。无效样本的标记为0、序号和三轴均为0。采样时刻放在V2帧头`source_time_ms`，取BMI088完整样本准备时的本机毫秒计数，0合法；它不是PC时钟，也不是消息被读取的时刻。

INS在原始采样有效性检查后发布快照，不依赖EKF校正是否成功。样本序号由BMI088驱动递增并跳过0，接收方可用序号识别重复／乱序，用帧头源时刻按无符号毫秒差判断年龄；V2接收端还可用MC02-0090的时间映射解释远端时刻，但本地采样时间不需要映射。这里不实现生产上位机的样本消费策略。

V2每50ms最多提交一个状态帧，姿态、裁判摘要和IMU按轮次轮流；IMU状态名义约150ms出现一次。只有采样年龄不超过5ms时才携带有效值。采样无效、超过5ms，或消息中心快照已经过期时，MCU发出明确全零无效状态和0源时间，供接收方撤销旧数值。坏样本不会用旧加速度续期。SP构建不改变原29/43字节帧，也不伪造采样时间；`VisionUpdateIMUSample()`在SP构建返回不可用。

生产端到端测试见[vision_v2_link_test.c](../../tests/host/vision_v2_link_test.c)，固定字节和CRC向量见[vision_protocol_v2_test.c](../../tests/host/vision_protocol_v2_test.c)。它们验证的是下位机编码、序号／时间字段和失效发布，不代表生产上位机已经接入或完成实体传输测试；任务范围与结果见[MC02-0006／0007记录](../../docs/workflow/tickets/MC02-0006-0007.md)。

## V2 对时与映射（MC02-0090）

V2 开启后，`VisionSend()` 在原状态发送节流之外推进一个单请求探测器：首次可发送时立即发 `0x83`，之后按 500 ms 周期探测，等待回应最多 100 ms。回应 `0x03` 的负载含请求编号和 PC 接收时刻，帧头来源时刻含 PC 发送时刻；MCU 以探测帧来源时间作为 T1，并使用完整回应帧最后字节的接收块时间作为 T4。每个匹配样本要求交换不超过 100 ms、计算 RTT 不超过 50 ms。三个有效样本建立映射，偏差跳变超过 50 ms会重新收集；2秒无有效样本后映射过期。USB代际切换清理旧映射，失效后仍按周期重新探测。

`VisionGetTimeSyncSnapshot()`只复制状态；`VisionMapRemoteTime()`只在同步映射有效时转换远端毫秒，错误保留输出。它不更新消息中心的来源时间，不改变业务帧首次接收时刻和控制新鲜度。SP不含来源时刻，SP构建的对时查询返回不可用。上位机生产实现、物理链路时钟精度和CAN板间对时不属于本记录的验收范围，后者由MC02-0093独立复用契约。

生产路径测试见[vision_v2_link_test.c](../../tests/host/vision_v2_link_test.c)、纯映射边界见[time_mapping_test.c](../../tests/host/time_mapping_test.c)，协议向量见[vision_protocol_v2_test.c](../../tests/host/vision_protocol_v2_test.c)。具体结果和限制记录在[MC02-0090任务记录](../../docs/workflow/tickets/MC02-0090.md)。
