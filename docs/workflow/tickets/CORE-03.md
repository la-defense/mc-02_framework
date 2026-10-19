# CORE-03：BSPInit 单份定义与双源文件链接

## 基线与范围

- 基线：`8d43df5b52e9b35f657febcb936fa7f69f522244`；分支：`fix/bsp-init-definition`。
- 将 `BSPInit()` 的实现从公共头移到唯一源文件，保留调用位置、启动步骤、参数和 `void` 返回方式。本任务只用本机验证，不连接开发板。
- 分阶段启动、逐项失败上报和重复初始化保护继续由 MC02-0055、MC02-0087 跟踪；本次没有把参数载入结果改为整体启动结果。

## 实现与接口

- [bsp_init.h](../../../bsp/bsp_init.h) 只维护声明、包含保护和调用条件，不再自包含或间接提供底层头文件。
- [bsp_init.c](../../../bsp/bsp_init.c) 按原顺序执行七步：DWT（480 MHz）、看门狗（4000 ms）、日志、ADC、复位原因输出、崩溃记录、参数载入。`RobotInit()` 仍负责中断状态和调度器启动前的调用，任务创建完成后仍把看门狗等待时间改为 200 ms。
- `robot_task.h` 为自己使用的 DWT 函数直接包含声明；活动 CMake 源码收集自动纳入新文件，旧 Makefile 显式补齐来源。本轮不扩大为 BUILD-02。

## 主机与构建证据

2026-10-20 的检查结果：

| 检查 | 结果 |
|---|---|
| WSL Debug / Release 全量 CTest | 各 163/163 |
| Windows Debug / Release 支持子集 | 各 74/74 |
| 新增两个独立调用方 | 共同链接生产 `bsp_init.c`，各自进程调用一次，七步顺序、次数和参数通过 |
| 检查是否在 Release 生效 | 使用 `TEST_CHECK`，两个 CTest 均设置 10 秒超时 |
| bench_safe / robot / rcdiag × Debug / RelWithDebInfo | 六种禁热固件构建和 ELF profile 隔离检查通过 |
| 非法配置 | bench_safe 开热、robot 开 rcdiag 均按预期拒绝 |
| CMake 增量发现 | 隔离副本的增删、移动源码、移动仅含头文件的目录及排除模板检查全部通过；与全新配置的源码和包含目录一致 |
| 来源与符号 | 六种编译清单各包含一次 `bsp/bsp_init.c`；主机 Debug/Release 与六种 ELF 各有一个全局 `BSPInit` 定义 |
| 编译参数 | Cortex-M7、fpv5-d16、hard-float 保持原配置；固件编译清单没有 `-Ofast` / `-ffast-math` |

主机测试入口：[bsp_init_link_test.c](../../../tests/host/bsp_init_link_test.c)。两个调用源文件同时参与链接，其中一个重复包含公共头，验证单个源文件内的包含保护；不同进程分别执行两个调用方，不把重复启动当作支持的板上行为。底层替身不操作硬件，`ParamInit()` 模拟返回 0，入口仍按原方式结束。

复用当前配置匹配的 `build/bspuart-03u1/` 固件和 Windows 主机目录，以及 WSL 的 `/home/mc02/build/bspuart-03u1/host-{Debug,Release}`。本轮日志保存在忽略目录 `build/core-03/`。构建技能脚本缺少 `tool_config` 导入，未能执行探测；本轮使用现有 CMake/Ninja 工具直接完成构建，没有修改技能安装。

增量回归命令为 `py -3 tools/workflow/check_cmake_regeneration.py --work-dir build/core-03/regeneration --jobs 4`，证据目录为 `build/core-03/regeneration/run-20261020-000205-6d9tjy3f/`。初始来源清单含161个文件，已包含新生产实现。

复现主要检查：

```powershell
# 新增主机测试；Windows 需先将 MinGW 的 bin 目录加入 PATH。
cmake -S tests/host -B build/core-03/host -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/core-03/host --target bsp_init_link_test
ctest --test-dir build/core-03/host -R production_bsp_init_ --output-on-failure

# 固件示例；另两类配置及优化档位使用同一生产源码。
cmake -S . -B build/core-03/bench -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMC02_PROFILE=bench_safe -DMC02_ENABLE_HEATER=OFF
cmake --build build/core-03/bench
py -3 tools/workflow/check_firmware_profile.py build/core-03/bench/Basic_Framework_MC02_bench_safe.elf --profile bench_safe
```

## 固件容量

与基线已有的六种同配置产物比较，`arm-none-eabi-size` 的 `text+data` 计入程序存储，`data+bss` 计入静态 RAM：

| 配置 | 本轮 Flash | Flash 变化 | 本轮 RAM | RAM 变化 |
|---|---:|---:|---:|---:|
| bench_safe Debug | 168692 B | 0 B | 104628 B | 0 B |
| bench_safe RelWithDebInfo | 168600 B | +8 B | 104696 B | 0 B |
| robot Debug | 197632 B | 0 B | 113504 B | 0 B |
| robot RelWithDebInfo | 196152 B | +16 B | 113544 B | 0 B |
| rcdiag Debug | 170916 B | 0 B | 107804 B | 0 B |
| rcdiag RelWithDebInfo | 170736 B | +16 B | 107864 B | 0 B |

移动到独立源文件后，函数编译与链接位置发生变化；这些小幅体积差异不代表启动性能变化。本轮没有测量物理耗时或硬件初始化结果。

## 教学入口

- [学习笔记01：声明、定义和多源文件链接](../../学习笔记/01-编译优化入门.md#12-头文件保护为什么挡不住链接时的多重定义core-03)
- [学习笔记索引](../../学习笔记/README.md)
