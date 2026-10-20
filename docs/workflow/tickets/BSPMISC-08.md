# BSPMISC-08：Float2Str 舍入和数组容量

本文记录三位小数转换的实现、测试和本机构建依据；任务状态只在 [TODO](../../../.Doc/TODO.md) 维护。本轮不连接硬件。

## 基线与实现

- 基线：`da697b2a617e26fa60a3cfe800ba90e378aea469`；分支：`fix/float2str-bounds`。
- [float_to_string.h](../../../bsp/log/float_to_string.h) 集中声明 `Float2Str(buffer, capacity, value)` 及返回状态；[实现](../../../bsp/log/float_to_string.c) 单独依赖标准 C，`bsp_log.h` 引入公开声明。旧两参数入口已删除，检索未发现需要迁移的生产调用方。
- 固定三位小数，舍入到最近的千分之一，恰好半数时远离0；正负零和舍入后的零统一写成 `0.000`。规则针对传入 float 的实际数值，`1.0625f → 1.063`、`−1.0625f → −1.063`；`1.0005f` 实际略小于理想十进制半数，结果为 `1.000`。
- 输入及舍入结果均限定在 `[-2147483648,2147483647]`。先用 double 检查范围和舍入，再用 int64_t 保存千分数，避免越界强制转整数；小数进位和补0由整数拆分完成。
- `capacity` 以字节计并包含结尾0，最大结果需要16字节。先准备局部完整文字，再检查容量并复制；空指针、非有限值、超范围、空间不足均返回具体错误，失败保留调用者数组全部原字节。
- 没有初始化、共享历史或分配器调用；不同数组可同时使用，同一数组由调用者串行访问。Makefile 同步新增源码，固件 CMake 自动发现该文件，主机目标直接链接生产实现。

## 数值与写入测试

[C 测试](../../../tests/host/float_to_string_test.c) 包含固定答案、实际半数及两侧相邻 float、正负小数进位、补0、零、极小值、整数上下界、NaN／Inf和越界值。容量覆盖准确长度及少1字节，最长文字的容量从0逐项检查；成功时检查数组两侧及未写尾部，失败时比较整个数组原字节。不同数组交错10000轮，WSL另用两个真实线程分别调用10000次。

[Python 参考](../../../tests/host/float_to_string_reference.py) 由 `Decimal.from_float()` 取得 binary32 实际值，再以 `ROUND_HALF_UP` 生成三位小数答案，没有复用生产函数的千倍整数拆分。固定种子 `0xB508`、半数两侧、十进制网格、2的幂及边界合计 **27035** 个不同位形，全部相符。C 测试的检查在 Release 仍执行，四项 CTest 的超时均为10秒。

| 本机验证（2026-10-20） | 结果 |
|---|---|
| WSL Debug／Release 全量 | 各173/173通过，包含新增4项 |
| Windows Debug／Release 支持子集 | 各109/109通过，包含新增4项 |
| WSL ASan／UBSan | 新增4/4通过；`detect_leaks=1`，错误立即退出 |
| 六种禁热固件 | bench_safe、robot、rcdiag × Debug／RelWithDebInfo 全部通过 |
| ELF 配置隔离 | 六个 ELF 全部通过，rcdiag同时检查遥控诊断符号 |
| 非法配置 | bench_safe启用加热、robot启用遥控诊断均被CMake拒绝 |
| CMake增量检查 | 新增／删除／移动源码、多层目录、仅移动头目录及排除模板均通过；当前基线编译162个文件，包含新转换文件 |

Windows 既有 `production_init_resource_failures` 不在支持子集：MinGW 的该测试配置缺少 `__real_malloc`、`__real_calloc`、`__real_free` 链接入口；这项仍由 WSL 全量执行。本轮没有更改它。

可在仓库根目录按下列入口重现主机验证；Windows沿用支持子集选择，WSL执行全量：

```sh
cmake -S tests/host -B build/host-Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-Debug --parallel 4
ctest --test-dir build/host-Debug --output-on-failure
# Release使用独立目录，并把CMAKE_BUILD_TYPE改为Release。
```

ASan／UBSan 使用独立 Debug 目录，C标志为 `-fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie`，链接标志为 `-fsanitize=address,undefined -no-pie`；仅构建 `float_to_string_test`，用 `ctest -R production_float_to_string_` 执行新增四项。环境启用 `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1` 和 `UBSAN_OPTIONS=halt_on_error=1`。

`tools/workflow/check_todo.py` 通过，差异空白检查通过；本轮新增和修改的本地文件及标题链接检查通过。

## 固件来源与资源

构建技能的 `cmake_builder.py --detect` 在导入阶段因 `No module named 'tool_config'` 退出，尚未执行配置；随后使用仓库现有 CMake／Ninja 入口完成全部验证，没有修改技能安装。六种配置均显式关闭加热；robot开启UI，bench_safe／rcdiag关闭UI，rcdiag仅在bench_safe基础上启用遥控诊断。

基线和本轮都在同一 Linux 源码路径构建，使用 WSL `arm-none-eabi-gcc 10.3.1` 及相同选项，避免源码路径文字影响尺寸。六份编译命令保留 Cortex-M7、`fpv5-d16`、hard-float，未启用 `-Ofast` 或 `-ffast-math`。

| 配置 | 基线／本轮 Flash（text+data） | 变化 | 基线／本轮静态 RAM（data+bss） | 变化 |
|---|---:|---:|---:|---:|
| bench_safe Debug | 174544 B | 0 B | 104316 B | 0 B |
| bench_safe RelWithDebInfo | 172236 B | 0 B | 104268 B | 0 B |
| robot Debug | 204068 B | 0 B | 113216 B | 0 B |
| robot RelWithDebInfo | 199864 B | 0 B | 113160 B | 0 B |
| rcdiag Debug | 176724 B | 0 B | 107484 B | 0 B |
| rcdiag RelWithDebInfo | 174324 B | 0 B | 107436 B | 0 B |

逐份核对 `compile_commands.json`，新转换文件恰好编译一次；`nm` 确认该对象只有一处 `Float2Str` 定义，原 `bsp_log.c` 不再提供重复定义。六个目标对象的 `.data` 和 `.bss` 都为0，未引用 HAL、RTT、RTOS 或分配器。当前没有生产调用方，链接器按原有规则移除未使用函数，六个最终 ELF 不保留该符号；主机测试直接执行生产计算函数。

本机日志保存在 WSL `/home/mc02/build/float2str/`（主机、Sanitizer、六种基线和当前固件、非法配置及 `validation-audit.json`）和 Windows `build/float2str/windows-Debug/`、`windows-Release/`。增量证据目录为 `/home/mc02/build/float2str/regeneration/run-20261020-125359-9w3bcczl`。隔离固件副本位于 `/home/mc02/build/float2str-source`，四个修改的生产文件已逐份核对 SHA-256 与交付工作区相同。

## 使用边界与复盘

本轮不验证 RTT 查看器显示、调试器连接、板上执行耗时或其他物理行为。普通日志仍可直接用 `%f`；只有需要单独保存固定三位小数文字时才使用此接口。后续调用者必须提供真实数组容量并检查本次状态，不能把失败后保留的旧文字当作新结果。

把舍入、容量和失败结果放进同一个小接口，使测试能够直接运行生产函数；独立十进制参考补充了手写固定答案，尤其能检查 float 半数附近的实际结果。

教学入口：[学习笔记18第10节](../../学习笔记/18-一条日志的旅程.md#10-把浮点数写成三位小数文字bspmisc-08)、[日志模块说明](../../../bsp/log/bsp_log.md)。
