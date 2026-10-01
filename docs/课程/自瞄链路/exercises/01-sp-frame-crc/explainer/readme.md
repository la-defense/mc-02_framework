# 练习 1：SP 帧与 CRC（解析）

验证分三层：帧长度和帧头决定是否是候选帧；CRC 判断覆盖范围内是否被意外修改；只有全部通过才能使用字段。CRC 不提供身份认证。对照[SP 速查页](../../../reference/01-sp-frame-crc.html)和[固件解析器](../../../../../../modules/master_machine/master_process.c)。
