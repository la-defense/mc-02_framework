# REF-03验证证据

此目录保存本轮软件与硬件身份、可复核记录；任务状态只见[TODO](../../../../.Doc/TODO.md)，解释及命令见[任务记录](../REF-03.md)。

- `firmware.json`：六种禁热固件text／data／bss、ELF SHA-256及UART复制存储的地址与尺寸。
- `backup.json`：烧录前两份1MiB Flash及独立256KiB参数区的大小和SHA-256；二进制原件保留于本机`build/ref03-hardware/`，不提交整片固件或参数内容。
- `software.json`：本机测试、构建、非法配置、增量发现和工具链检查结果。

后续裸板原始UART、MCU记录、会话身份、60秒被动基线与参数回读只在取得后纳入，不将软件模型结果作为硬件通过。
