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

## 格式检查与关闭宏（LOG-04）

自有C源码启用格式检查，并将选定格式告警作为编译错误。`PrintLog()` 声明了printf格式属性，调用时的格式符、实参类型和数量由编译器检查；日志宏直接使用标准 `snprintf()`。外部文字应写为 `PrintLog("%s", text)`，不能直接作为不可确定的格式串。

这不检查字符串指针的实际可读性，也不保证动态文字完整。`PrintLog()` 超过255字节时按原规则仅交付前面放得下的部分，返回值仍为完整所需长度；负返回值不发送。日志宏的格式化容量为248字节，其中已含前导空格、类型和颜色，之后追加换行和颜色复原码；格式化失败仅交付结尾。

`DISABLE_LOG_SYSTEM=1`仅关闭 `LOGINFO`、`LOGWARNING`、`LOGERROR`：保留编译期格式检查，运行时不求值参数、不发送。无颜色 `LOG()` 和普通函数 `PrintLog()` 仍可输出。`LOG_TEST=1`保留五条合法混排样例和结束说明，故意超长常量另作编译失败测试。

配置时编译正反例，测试再核对实际编译命令，自有源码与第三方源码的策略分开。当前MinGW存在长度诊断不足，Windows运行测试不能替代必要的ARM／WSL长度编译检查。用法及容量见[学习笔记18第11节](../../docs/学习笔记/18-一条日志的旅程.md#11-编译通过与文字完整是两件事log-04)，证据见[任务记录](../../docs/workflow/tickets/LOG-04.md)。

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
