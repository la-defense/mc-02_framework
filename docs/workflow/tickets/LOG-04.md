# LOG-04：格式串与文字长度编译检查

本文记录编译策略、生产接口测试及本机构建证据；任务状态仅在 [TODO](../../../.Doc/TODO.md) 维护。本轮没有连接硬件。

## 基线与实现

- 基线：`1c9032c18c660dd5e281fede7994b27627161b20`；分支：`chore/log-format-diagnostics`。
- [共用CMake配置](../../../cmake/mc02_format_diagnostics.cmake)对自有目录的C源文件启用 `-Wformat=2`、`-Wformat-overflow=2`、`-Wformat-truncation=2`，并将 format、format-security、format-nonliteral、format-overflow、format-truncation 五类选定告警作为错误。没有新增工程全局 `-Werror`，已有测试自己的策略保留。
- 目录范围为 `bsp`、`modules`、`application`、`Core/Src`、`USB_DEVICE` 和 `tests`；DSP、HAL、RTOS等厂商C源不附加该组策略。旧Makefile仅对自有C对象增加相同八个选项，检查脚本核对两处声明一致。
- [格式属性头](../../../bsp/log/format_check.h)不引入HAL、RTT或RTOS，也不生成运行时代码。`PrintLog()`、`UICharDraw()`、`OLED_printf()` 的格式位置分别为 `(1,2)`、`(10,11)`、`(3,4)`；MinGW沿用标准库声明的printf方言，其他已支持GCC使用printf属性。三个接口的签名和运行职责不变。
- 关闭的 `LOGINFO`、`LOGWARNING`、`LOGERROR` 使用常假分支保留类型检查，执行时不求值参数、不保留发送调用；`LOG()`、`PrintLog()` 仍可直接输出。等级与异步队列没有扩展。
- `LOG_TEST=1`保留五条合法样例和结束说明；原超长数字常量改为编译负例，运行时长文字单独测试。补齐 `bsp_log.c` 对 `stdarg.h` 的显式包含。

本轮新策略未发现需要更改业务行为的现存格式错误；没有增加告警屏蔽或通过隐藏生产调用点长度绕过检查。

## 编译能力与检查方式

[检查脚本](../../../tools/workflow/check_format_diagnostics.py)在配置阶段编译生产头文件的正反例，并将源文件、命令、诊断及报告写入构建目录。每轮完整集合有29项：开启日志16项、关闭日志13项，包含两个合法控制和各类错误；关闭宏无输出，不对死分支要求长度推断，类型错误仍须拒绝。

| 工具链 | 格式类型／数量／不可确定格式 | 已知长度负例 |
|---|---|---|
| ARM GCC 10.3.1 | 按预期拒绝 | 三类均拒绝，`-Og`和`-O2`已核对 |
| WSL GCC 11.4.0 | 按预期拒绝 | 三类均拒绝，主机测试覆盖`-O0`和`-O2` |
| MinGW GCC 11.5.0 | 按预期拒绝 | 三类未报告，记录为覆盖限制，不能算拒绝成功 |

三类长度负例分别是标准 `snprintf()`、生产日志宏、标准 `sprintf()`。MinGW的报告为 `length-diagnostic-unavailable`；必要的ARM／WSL长度检查不因此省略。脚本必须同时核对编译失败及具体 `-Werror=format...` 类别，缺头文件、未知选项或工具启动失败均不能当作覆盖。合法控制若失败也立即失败，不自动撤掉策略。

CTest按 PrintLog、UI／OLED、宏、长度四组分别运行，每组都有合法控制，在`-O0`、`-O2`重复验证，每项超时10秒。另有实际 `compile_commands.json` 检查：六种固件各检查106条自有C命令和54条厂商C命令，自有文件具有全部选项，厂商文件没有附加这些选项。汇编不纳入C告警检查。

重现编译能力检查和实际命令核对：

```sh
python3 tools/workflow/check_format_diagnostics.py --compiler arm-none-eabi-gcc --optimization=-Og --output build/format-arm
python3 tools/workflow/check_format_diagnostics.py --compile-commands build/firmware/compile_commands.json
```

参考：[GCC 10.3告警说明](https://gcc.gnu.org/onlinedocs/gcc-10.3.0/gcc/Warning-Options.html)、[GCC 10.3格式属性](https://gcc.gnu.org/onlinedocs/gcc-10.3.0/gcc/Common-Function-Attributes.html)。这些检查不能确认字符串指针有效、生命周期正确或动态文字完整。

## 生产运行行为与主机回归

[生产日志测试](../../../tests/host/log_test.c)先初始化RTT，再调用生产日志；仅替换RTT、时间及一次格式化失败，正常格式化仍用真实标准库。运行时输入来自进程参数，覆盖长度0、236、237、245、246、247、255、256、300字节、彩色及无色正文、完整结尾、格式化失败和自测。

`PrintLog()` 最多写255字节文字，返回值仍为完整所需长度；负值不发送。日志宏先用248字节容量格式化包含前导空格、颜色和类型的正文，再追加换行和颜色复原码；失败仅输出结尾。这些既有行为没有改变。

Release首轮暴露了故障模型问题：对合法常量输入，GCC可能提前计算返回长度，包装标准库返回负值不能模拟该调用。故障注入改用运行时输入后，Debug／Release和Sanitizer均通过，生产实现未为测试改写。

[关闭宏测试](../../../tests/host/log_disabled_test.c)检查自增不发生，并故意不提供取样函数及RTT实现；成功链接与执行共同证明没有保留这些调用。

| 验证（2026-10-20） | 结果 |
|---|---|
| WSL全量 Debug／Release | 各187/187通过 |
| Windows支持子集 Debug／Release | 各123/123通过 |
| 日志ASan／UBSan | 5/5通过，开启泄漏检查，遇错立即退出 |
| 最终头文件及测试说明整理 | 受影响的五项日志测试在两平台、两构建类型及WSL Sanitizer重新构建并通过 |
| 固件矩阵 | bench_safe、robot、rcdiag × Debug／RelWithDebInfo，六种均禁热并通过 |
| 开启自测 | bench_safe Debug、`LOG_TEST=1`、禁热构建及ELF配置检查通过 |
| 非法配置 | bench_safe启用加热、robot启用遥控诊断均被拒绝 |
| 旧Makefile | `make -n -B`检查自有bsp_log对象有全部八项、DSP对象没有附加该组策略 |
| CMake增量 | 新增／删除／移动源码、多层目录、仅移动头目录及排除模板均通过，与全新配置一致 |

Windows沿用既有支持子集，排除 `production_init_resource_failures`：该MinGW测试配置缺少 `__real_malloc`、`__real_calloc`、`__real_free` 链接入口，WSL全量仍执行它。没有为本轮扩大排除集合。

TODO格式、差异空白及本轮新增／修改的本地文件和标题链接检查通过。

主机入口为 `cmake -S tests/host -B build/host-Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug`，随后构建并运行CTest；Release使用独立目录。Sanitizer使用 `-fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie` 和链接选项 `-fsanitize=address,undefined -no-pie`，只构建日志两目标，执行 `ctest -R production_log_`，环境为 `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1`、`UBSAN_OPTIONS=halt_on_error=1`。

## 固件资源与证据位置

构建技能脚本在导入阶段因 `No module named 'tool_config'` 无法启动，随后使用仓库现有CMake／Ninja入口，没有修改技能安装。增量检查的隔离输入增加了配置时使用的Makefile、格式检查脚本和日志硬件替身头，共复制889份输入，基线编译162个路径。

基线与本轮均在同一Linux源码路径、同一ARM GCC 10.3.1和相同选项下构建，避免路径文字影响尺寸。六种配置均保留 Cortex-M7、`fpv5-d16`、hard-float，未使用 `-Ofast` 或 `-ffast-math`。Flash按 `text+data`，静态RAM按 `data+bss`：

| 配置 | 基线／本轮Flash | 变化 | 基线／本轮RAM | 变化 |
|---|---:|---:|---:|---:|
| bench_safe Debug | 174548 B | 0 B | 104316 B | 0 B |
| bench_safe RelWithDebInfo | 172240 B | 0 B | 104268 B | 0 B |
| robot Debug | 204072 B | 0 B | 113216 B | 0 B |
| robot RelWithDebInfo | 199868 B | 0 B | 113160 B | 0 B |
| rcdiag Debug | 176728 B | 0 B | 107484 B | 0 B |
| rcdiag RelWithDebInfo | 174328 B | 0 B | 107436 B | 0 B |

日志位于WSL `/home/mc02/build/log-format/` 及Windows `build/log-format/windows-Debug/`、`windows-Release/`。WSL目录保存主机、Sanitizer、六种基线与当前固件、额外自测、Makefile命令、非法配置和 `extra-evidence.json`；构建相关修改文件已逐项比较SHA-256，与交付工作区一致。增量证据：`/home/mc02/build/log-format/regeneration/run-20261020-135810-0ey78d5z`。

## 边界与复盘

只关闭LOG-04。MC02-0066异步日志、MC02-0004等级配置和MISC-06的OLED运行时容量问题继续开放。没有测试RTT查看器、探针连接、物理吞吐或板上耗时。

告警名称存在不代表它能拒绝目标错误，检查必须覆盖合法控制、具体诊断原因和实际目标命令。MinGW长度推断不足应直接记录，由必要的ARM／WSL检查补充；运行时长度处理和内存检查继续保留。

教学入口：[学习笔记10第11节（LOG-04历史）](../../学习笔记/10-可变参数与格式串安全.md#11-让编译器检查可变参数log-04)、[学习笔记18第11节（LOG-04历史）](../../学习笔记/18-一条日志的旅程.md#11-编译通过与文字完整是两件事log-04)、[日志模块说明](../../../bsp/log/bsp_log.md)。
