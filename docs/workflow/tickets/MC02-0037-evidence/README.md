# MC02-0037 裸板基线证据

本目录保存 2026-10-20 在 12.5V 限流基线下取得的一组探索性记录。它用于复核生产 ADC 诊断、串口状态帧和误差报告流程；由于仪表准确度、实物修订和完整误差预算未确认，**这组数据不构成 VIN 精度合格结论**。

万用表为 DELIXI DLX890D+，用户报告显示分辨率 0.1V、没有校准记录且无法找到准确度规格。用户报告板端输入读数为 12.4V，电源设定为 12.5V、限流 2A；本点没有记录电流表读数或实测 ADC 参考电压。全过程保持 12.5V，没有升压；板上连接 LCD、DR16 接收端、USB 与 DAPLink，没有执行器，也没有发送控制帧。

- `baseline-12-5V.csv` 保存最终复采的 30 份完整 ADC 诊断记录；`baseline-12-5V-attempts.jsonl` 保存采集尝试和原始字段，`baseline-12-5V-session.json` 固定该次测量使用的会话配置副本。
- `baseline-summary.json` 和 `baseline-summary.csv` 是工具生成的逐点统计与缺项判断；`baseline-vin-error.png`、`baseline-vin-error.svg` 显示本次探索性偏差。
- `com14-passive-raw.bin.gz` 保存 65.309 秒 COM14 原始串口流；`com14-passive-report.json` 记录解压前大小、有效帧数和 CRC／四元数检查结果。
- `imu-final-snapshots.jsonl`、`safety-final-snapshots.jsonl` 保存只读运行快照。`manifest.json` 列出固件、备份区和本目录文件的身份信息及 SHA-256。

最终复采取得 30 份完整且递增的 ADC 记录，原始 VIN 为 22188～22198（平均 22192.7）；生产换算值为 12.289989～12.295527V，平均 12.292592V，标准差 0.001319V。相对用户报告的 12.4V 表显，平均读数差为 −0.107408V（−0.866%）。这个差值接近仪表一位显示分辨率，且仪表准确度未知，因此不能解释成实际误差上限或通过精度验收。报告明确为 `not-qualified`，MC02-0037 继续开放。
