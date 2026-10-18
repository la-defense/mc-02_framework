# 固定版本的最小 CMSIS-DSP 源码

这里保存 CMSIS_5 **5.7.0** 的 DSP **1.7.0**，提交为
`a65b7c9a3e6502127fdb80eb288d8cbdf251a6f4`。厂商头文件、九个 C 文件及
Apache-2.0 许可按原字节保留；[provenance.json](provenance.json)记录每个文件的上游路径和 SHA256。
上游根地址为 [ARM-software/CMSIS_5](https://github.com/ARM-software/CMSIS_5/tree/a65b7c9a3e6502127fdb80eb288d8cbdf251a6f4)。

[cmake/mc02_dsp.cmake](../../../cmake/mc02_dsp.cmake)是固件和主机共用的显式源码清单。
构建完全离线，矩阵尺寸检查由 DSP 目标自身启用。表文件只启用浮点 sin 表；cos 复用该表。
固件沿用 Cortex-M7、fpv5-d16、hard-float；主机通过该版本自带的 `__GNUC_PYTHON__` 分支使用可移植代码。
不设置 fast-math。原有 CMSIS Core、HAL 和 RTOS 未升级，清单同时记录实际 Core 编译器头文件指纹。

该版本的 `arm_mat_inverse_f32()` 会改写输入矩阵。业务代码通过
[dsp_matrix.h](../../../modules/algorithm/dsp_matrix.h)提供的两个独立数组计算，成功且有限后才复制结果。
厂商文件不改写中文注释；本目录说明和适配层解释工程中的用法。

执行 `python tools/workflow/check_dsp_sources.py` 核对源码和当前 Core 头指纹。
数值测试入口为 `production_dsp_matrix_and_trigonometry`；硬件编译和裸板证据见
[MC02-0099](../../../docs/workflow/tickets/MC02-0099.md)。
