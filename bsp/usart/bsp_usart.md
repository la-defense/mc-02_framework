# bsp_usart

<p align='right'>neozng1@hnu.edu.cn</p>

本模块登记UART、复制接收数据、在任务中派发并有限重试接收错误。先配置HAL句柄、接收DMA及中断，再在启动阶段串行注册；每个硬件句柄只能有一个所有者。接口说明集中在[bsp_usart.h](bsp_usart.h)，纯接收描述符见[usart_rx.h](usart_rx.h)。本页随BSPUART-03U1更新。

## 注册和借用

```c
static void Receive(void *context, const USART_RxBlock_t *block)
{
    MyParser *parser = context;
    // 在这里检查连续性、长度、来源时间，再复制需要保留的残帧。
    MyParserFeed(parser, block);
}

USART_Init_Config_s config = {
    .usart_handle = &huart1, .recv_buff_size = 128,
    .module_callback = Receive, .context = &parser,
};
USARTInstance *uart = USARTRegister(&config);
```

以上MyParser为调用示意，真实例子是[裁判接收](../../modules/referee/rm_referee.c)。配置值被复制，HAL句柄和context指向的对象必须长期有效；注册失败返回NULL。业务不能读取recv_buff，它一直由DMA复用。回调的block及其只读data只在本次回调返回前有效，跨回调残帧由协议自己复制。

length是此次RxEvent实际字节数，单位字节；它可能只是一帧的一部分，也可能包含多个帧。received_ms是在原RxEvent通知中记录的HAL毫秒值，0合法，不是逐字节的物理到达时间。sequence是BSP连续性编号，通知、丢块、错误和显式重启都会消耗编号，自然回绕；相邻已派发块编号不连续时，协议必须清除残帧。

## 中断、任务与容量

中断复制字节及元数据到队列，再重装DMA。USARTProcessRx()只由一个任务调用，直接借用队列数据，不把数据复制回DMA数组。回调返回之后才释放该槽；递归派发直接返回。

每路四槽、可用三槽，满时丢新块。接收容量为1～255字节，八个登记名额共享3072字节静态池。每槽占align4(16 + 接收容量)字节，容量和名额都可能使注册失败。任务入口记录各路已有数据的末位置，每路最多处理三块，处理中新增的块留到下一轮。超过50ms的排队数据丢弃，等于50ms允许；不把这些数量上限解释为板上耗时保证。

USARTServiceInit()供任务显式重启，打断连续性并隔离此前排队块。正常通知后的DMA重装使用内部入口，保持连续性。错误中断只计数并请求恢复；任务中最多尝试三次Abort及重装，失败后保留诊断，等待显式重启。注册启动失败后先确认中止；中止也失败时保留隔离对象和池空间，禁用业务回调，避免释放硬件仍可能借用的数组。

## 发送和验证

旧USARTSend()保留兼容；新业务需要结果时使用USARTSendChecked()。返回ACCEPTED仅表示HAL接受启动，IT/DMA完成前原发送数组仍须有效且不变。本模块没有通用发送队列；裁判发送的持久缓冲由其模块管理。

[队列测试](../../tests/host/usart_snapshot_test.c)检查只读借用、派发中到达、满队列、重启、有限恢复及注册失败；[端到端测试](../../tests/host/uart_stream_test.c)用真实BSP、裁判/SP解析及CRC检查跨块、多帧和原时间。[任务记录](../../docs/workflow/tickets/BSPUART-03U1.md)保存验证范围。主机模型不代替DMA时序、电平或真实吞吐测试。