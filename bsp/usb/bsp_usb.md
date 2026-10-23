# USB复制传输与生命周期

本目录提供单CDC传输。公开入口为[usb_transport.h](usb_transport.h)，不暴露ST或HAL类型。业务先调用USBTransportInit注册长期回调，既有defaultTask调用USBTransportProcess管理启动／恢复，唯一接收任务调用USBTransportProcessRx解析数据。仅普通任务可以提交或推进；中断只通过内部入口登记。

接收固定8项，每项最多128字节；每轮最多处理4项。队列保存实际长度、原接收毫秒值、连续性编号和连接代号。端点存储可在中断返回后复用；业务回调中的数据地址只在回调期间有效。超长整块拒绝，丢块不继续拼接旧SP候选。

发送成功时复制到内部唯一在途数组，调用方随后可以改写原存储。忙时拒绝，不增加等待项；状态上行由下一周期重新组装最新内容。达到100ms未完成就失败，确认停止前保留数组；完成通知在普通任务中仅执行一次，接收者仍需自行确认业务是否处理。

USB复位、反初始化、暂停或断开会撤销旧连接。最多3次自动重建，连续60秒双方通信正常后补满额度；仅重新枚举不清次数。额度耗尽后等待主机重新配置；停止无法确认则保持隔离，等待系统重新初始化。详细规则及证据见[MC02-0062](../../docs/workflow/tickets/MC02-0062.md)和[学习笔记16](../../docs/学习笔记/16-USB-CDC虚拟串口通信.md)。

## CubeMX人工核对

保留当前单CDC、Full Speed内置PHY、USB DMA关闭、VBUS检测关闭及原描述符。重新生成后逐项对照本轮提交和`.ioc`：

| 文件 | 必须核对的接入 |
|---|---|
| USB_DEVICE/App/usb_device.c | MX入口只登记模块，不再直接启动库；实际启动由唯一USB任务完成 |
| Core/Src/freertos.c | defaultTask每轮调用USBTransportProcess，不从控制任务或ISR重建 |
| USB_DEVICE/App/usbd_cdc_if.c/.h | 实际长度入队、重新武装结果、配置／反初始化／真实完成通知；不恢复旧借用发送入口 |
| USB_DEVICE/Target/usbd_conf.c | 时钟失败和HAL初始化失败返回USBD_FAIL；复位／暂停／恢复／断连接入；FIFO清理及IRQ对应当前后端 |
| cmake/mc02_firmware_sources.cmake | 登记实际核心和H723后端，观测模块标记跟随真实源码 |

生成器不会自动理解这些改动，部分返回路径位于生成区之外；人工核对不能省略，也不声称能自动防止覆盖。第三方库与HAL版本保持不变。当前软件验证、裸板实测和未验证条件以任务记录为准。
