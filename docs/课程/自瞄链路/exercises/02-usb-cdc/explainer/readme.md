# 练习 2：USB CDC（解析）

这里把异步 USB 堆栈替换成一份确定性的内存模型。`busy_` 模拟端点仍在读缓冲区；直到 `complete()` 才允许复用。对照[CDC 状态速查页](../../../reference/02-usb-cdc.html)和[当前USB复制传输实现](../../../../../../bsp/usb/usb_transport.c)。MC02-0062已移除旧bsp_usb.c借用入口，练习仍只说明异步归属，不代替真实CDC或裸板测试。
