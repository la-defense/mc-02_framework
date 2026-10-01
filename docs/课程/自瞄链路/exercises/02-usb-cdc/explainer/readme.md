# 练习 2：USB CDC（解析）

这里把异步 USB 堆栈替换成一份确定性的内存模型。`busy_` 模拟端点仍在读缓冲区；直到 `complete()` 才允许复用。对照[CDC 状态速查页](../../../reference/02-usb-cdc.html)和[USB 发送缓冲修复](../../../../../../bsp/usb/bsp_usb.c)。
