# bsp iic

> 预计增加模拟iic

## 注意事项

使用时写入地址，不需要左移。7 位和 10 位设备都按数据手册填写原值，再在
`IIC_Init_Config_s.address_mode` 选择 `IIC_ADDRESS_7BIT` 或 `IIC_ADDRESS_10BIT`。

HAL 句柄的 `Init.AddressingMode` 必须与设备一致：7 位设备使用
`I2C_ADDRESSINGMODE_7BIT`，10 位设备使用 `I2C_ADDRESSINGMODE_10BIT`。登记时
两种模式不匹配会返回 `NULL`，不会把 10 位地址当 7 位设备悄悄发出去。

7 位地址保存时左移一位，最低位留给读写方向；10 位地址按原值保存。10 位地址
`0x020` 和 7 位地址 `0x10` 的编码值相同，所以只能靠 `address_mode` 区分，不能
按数值是否大于 `0x7F` 猜测。

cubemx未配置dma时请勿使用dma传输，其行为是未定义的。

## 总线机制详解

关于I2C的序列传输,Restart condition和总线仲裁,请看:

https://blog.csdn.net/NeoZng/article/details/128496694

https://blog.csdn.net/NeoZng/article/details/128486366

使用序列通信则在单次通信后不会释放总线,继续占用直到调用传输函数时传入`IIC_RELEASE`参数. 这个功能只在一条总线上挂载多个主机的时候有用.
