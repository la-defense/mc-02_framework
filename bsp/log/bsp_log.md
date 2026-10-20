# bsp_log

<p align='right'>neozng1@hnu.edu.cn</p>

本文说明 RTT 日志和可单独使用的三位小数文字转换。日志需要先初始化 RTT；`Float2Str()` 不访问硬件，也不需要初始化。

## 日志输出

`bsp_log.h` 提供 `LOGINFO`、`LOGWARNING`、`LOGERROR`，分别输出信息、警告和错误。当前实现先用 newlib 的 `snprintf()` 准备文字，再调用 `SEGGER_RTT_WriteString()`。CMake 已链接 `_printf_float`，所以可以直接使用 `%f`：

```c
LOGINFO("current=%.3f", (double)current_feedback);
```

无颜色输出可用 `LOG()`；普通函数入口名称是 `PrintLog()`：

```c
int required = PrintLog("Motor %d error %d\n", 3, 1);
```

`PrintLog()` 返回完整格式化所需字节数，不含结尾0；负值表示格式化失败。当前本地数组最多交付255字节，返回值可能大于实际写出文字长度；它不证明 RTT 查看器已收到或显示。日志宏的数组还要保留换行和颜色复原码空间。

RTT 查看需要支持它的调试器与工具。使用与实际探针匹配的项目配置；本机文字测试不验证 SWD 连接、查看器显示或物理耗时。

## 固定三位小数（BSPMISC-08）

需要独立保存文字时，可包含 `float_to_string.h`；`bsp_log.h` 也会引入该声明：

```c
char text[FLOAT2STR_BUFFER_SIZE];
Float2StrStatus_t status = Float2Str(text, sizeof(text), current_feedback);
if (status == FLOAT2STR_OK)
    LOGINFO("current=%s", text);
```

`text` 是字符数组。旧示例的 `char *text[64]` 声明的是指针数组，不能用作这个输出数组。

- 固定三位小数并补0，`1.2f` 得到 `1.200`；小数进位会进入整数部分。
- 舍入到最近的千分之一，恰好半数时远离0；`1.0625f` 得到 `1.063`，`-1.0625f` 得到 `-1.063`。
- 正负零及舍入后的零均为 `0.000`，小数点固定为 ASCII `.`。
- 输入和舍入结果须处于 `[-2147483648,2147483647]`。NaN、无穷大和超范围输入分别返回错误。
- `capacity` 包含结尾0；`FLOAT2STR_BUFFER_SIZE` 为16，可容纳最长结果 `-2147483648.000` 及结尾0。容量不足整条拒绝。
- 任何失败都保留调用者数组的原字节，调用方只有检查成功后才使用本次结果。

旧的两参数入口已移除，仓库中没有需要迁移的生产调用方。详细范围和返回状态只在 [float_to_string.h](float_to_string.h) 维护；实现无共享历史、动态分配或 RTT 依赖。舍入按传入 float 的实际值计算，十进制字面量不一定能在 float 中精确保存，见[学习笔记18](../../docs/学习笔记/18-一条日志的旅程.md#10-把浮点数写成三位小数文字bspmisc-08)。
