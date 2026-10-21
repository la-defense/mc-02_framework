# REF-03验证证据

任务状态只在[TODO](../../../../.Doc/TODO.md)，方法、范围与失败定位见[任务记录](../REF-03.md)。这里只保存本轮可复核原始数据和身份，不保存整片Flash或参数内容。

- [firmware.json](firmware.json)：六种禁热固件text／data／bss、ELF SHA-256、UART复制存储地址及尺寸。
- [software.json](software.json)：WSL主机与ARM构建、非法配置、增量及工具链检查结果。Windows仅承担串口／探针宿主，没有原生构建验收。
- [backup.json](backup.json)：烧录前双份1MiB Flash及独立256KiB参数区身份。后两次完整备份的摘要见任务记录；二进制原件只保留于本机build/ref03-hardware。
- [startup-failure.jsonl](startup-failure.jsonl)、[startup-rtos.json](startup-rtos.json)：前两次启动失败及真实FreeRTOS预调度状态，不能作为通过证据。
- [primary/session.json](primary/session.json)：最终固件源ad41d3204b60b3f928dc972e38b76fec818c9ed0、ELF、参数保留、应用回读和接线身份。
- [primary/baseline-report.json](primary/baseline-report.json)、[primary/load/report.json](primary/load/report.json)、[primary/load/metrics.json](primary/load/metrics.json)：首轮被动／有限负载与MCU计时统计。
- [retry/baseline-report.json](retry/baseline-report.json)、[retry/load/report.json](retry/load/report.json)、[retry/load/metrics.json](retry/load/metrics.json)：用户复查接线后仅复位、使用同一固件完成的重复验证。
- 各轮post-load-report／final-tx-state记录结束后的安全及资源回收；`*-snapshots.jsonl.gz`含只读RAM与固定GPIO／TIM3值，`*-sp.bin.gz`为被动SP原始字节。
- 各轮load目录的uart.bin.gz／frames.csv.gz／mcu.jsonl.gz保存裁判口原始字节、逐帧表及MCU读前／读后版本、原始456字和解码记录。identity.json.gz保存有界身份注入的PC时刻，PC时刻不当作精确线端时间。
- [manifest.json](manifest.json)给数据大小与SHA-256，另对gzip列解压后摘要。文件仍在写入时的大小不作为无数据结论；只接受关闭后完整记录与MCU通知对应的核验。

复核时将每轮load中的三个gzip分别解压成uart.bin、frames.csv、mcu.jsonl，再调用tools/referee_budget_capture.py的report_capture(Path(...))。报告重新解码原始MCU字，核对通知顺序、完整CRC、固定内容、时长、34ms、115ms与滚动字节额度；数据不完整则拒绝。软件拒绝注入由主机测试覆盖，板上没有故意制造DMA停止失败或硬件故障。

最终固件默认禁热、没有执行器入口，有限负载phase=3后停止。它不保证真实裁判服务器或客户端行为，真实DMA物理停止时序和波形仍不在本轮范围。
