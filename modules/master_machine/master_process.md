# master_process

<p align='right'>neozng1@hnu.edu.cn</p>

本模块登记视觉链路、保存合法SP命令、检查来源时限并回传状态。先调用VisionInit()，再通过VisionGetSnapshot()复制命令及状态；发送内容由VisionUpdateTx()准备，VisionSend()追加CRC并尝试发送。公开用法见[master_process.h](master_process.h)，字节布局见[vision_protocol.h](vision_protocol.h)。

## 接收与时间（BSPUART-03U1）

默认使用USB VCP，编译时可选择VISION_USE_UART，同时选择两种传输会报错。SP命令29字节、状态43字节，未改线格式。旧seasky_protocol是独立A5工具，不属于当前视觉链路。

UART回调接收只读USART_RxBlock_t，以长期context保存29字节候选和每个字节的原通知时间；跨块保存残帧，同块多帧按顺序各处理一次。头或CRC失败后向后移一字节继续找头。连续性中断、显式重启或组帧超过50ms时放弃残帧。USB保持无状态片段扫描，找到首个完整正确帧返回，不新增USB跨片段拼接。

发布命令只在头和真实CRC通过后发生；last_rx_ms取首个组成UART块的原RxEvent通知时间，0也合法。查询和发送都会检查50ms来源期限，边界允许；超时清零命令并撤销有效标记，保留原时间供诊断。离线回调清命令与时间，并请求下一次UART接收重建残帧。DaemonReload不能让旧来源获得新期限。

CRC不能证明模式、数字或机械角度合法，后续控制仍须检查是否允许使用。当前模块只支持单个视觉实例。

## 发送与验证

状态帧从持久静态缓冲发送。USB忙时记丢弃次数，UART忙时跳过；当前发送入口不返回对端收到或完成状态，不提供通用队列。原有异步发送管理的其他限制由原TODO跟踪。

[流式回归](../../tests/host/uart_stream_test.c)使用真实UART队列和生产解析，检查分割、逐字节、多帧、错误、来源时间及恢复；[CRC协议回归](../../tests/host/crc_protocol_test.c)继续覆盖USB正常路径和CRC失败。完整证据见[BSPUART-03U1](../../docs/workflow/tickets/BSPUART-03U1.md)，本轮没有硬件操作。