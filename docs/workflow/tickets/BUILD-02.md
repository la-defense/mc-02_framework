# BUILD-02：唯一 CMake 入口与双板角色构建

任务状态只维护在 [.Doc/TODO.md](../../../.Doc/TODO.md)。本轮从 `b61f418ef105269f87e3af2f6145a024fa1f77f5` 建立 `chore/cmake-only-build`；只做 WSL 主机测试和 ARM 固件构建，不连接硬件、不运行烧录或调试会话。

## 构建接口

- 固件入口为根目录 `CMakePresets.json`，CMake 3.22.1 兼容预设固定使用 Ninja；主机测试使用独立的 `tests/host/CMakePresets.json`。全部输出放在 `build/wsl/<预设名>/`。
- `bench-debug` 是编辑器默认构建，配置为 `bench_safe` 且加热关闭。`*-debug` 为 Debug，`*-rel` 为 RelWithDebInfo。robot 角色使用 `MC02_BOARD_ROLE=single|chassis|gimbal`，视觉传输使用 `MC02_VISION_TRANSPORT=usb|uart`。
- `cmake/arm-none-eabi-toolchain.cmake` 集中选择 ARM GCC、汇编器、objcopy、objdump、归档器与 size 工具。`cmake/mc02_firmware_sources.cmake` 登记平台、BSP、公共运行依赖和所选应用源码；未登记模块不自动参加固件编译。头目录来自已选源码所在目录和少量确实共用的接口目录。
- 单板选择 RobotCMD、底盘、云台和发射；底盘板只选底盘应用；云台板选择 RobotCMD、云台和发射。裁判 UI 与视觉传输单独配置。现有 CANComm 及控制代码格式沿用，但本轮构建不证明双板实车通信可用。
- VSCode 保持 Windows 窗口，以参数数组调用 `wsl.exe`。DWARF 源路径映射为 Windows 工作区路径；launch 配置分别指向匹配的 bench_safe、单板、底盘板或云台板 ELF。Ozone 公共入口不固定探针编号和个人工具安装路径。
- 删除根 Makefile 与旧 Makefile 编辑器入口；保留上游旧教程作为历史资料。CI 复用 CMake 预设和同一 profile 检查，本轮未触发远端工作流。

## 本机验证

验证环境为 WSL Ubuntu 22.04、CMake 3.22.1、Ninja 1.10.1、主机 GCC 11.4.0、ARM GNU GCC 10.3.1。

| 验证 | 结果 |
|---|---|
| 主机 CTest Debug／Release | 各 288/288 通过 |
| Python 工具回归 | 60 项运行，59 项通过、1 项按条件跳过 |
| 格式编译正反例 | ARM GCC 16 组通过；实际编译命令含全部格式选项，自有 C 98 项、第三方 C 53 项分开核对 |
| 主固件矩阵 | bench_safe、rcdiag、robot single/chassis/gimbal 共 10 种 Debug／RelWithDebInfo 构建成功 |
| 附加固件 | refdiag Debug／RelWithDebInfo、single 与 chassis UI 关闭、single UART 视觉、普通日志＋日志自测配置均构建成功 |
| Profile 与源码选择 | 十种主配置、refdiag 两档、UI 关闭两种角色、UART 视觉及日志自测均通过；编译数据库的 C／汇编输入与显式清单、固定 DSP／启动输入完全一致，角色排除、Ninja 链接输入和 ELF 符号检查通过 |
| 配置拒绝 | 裸板启热、裸板双板角色、未知角色、云台板开启裁判 UI、额外冲突角色／视觉宏及裁判／遥控诊断冲突均在配置阶段拒绝 |
| 缺失依赖拒绝 | 隔离副本中的已登记源码缺失和已登记头目录缺失均在 CMake 配置阶段失败，并报告缺失路径 |
| CMake 增量重生成 | 10 种变更场景全部通过；未登记源码／模板不进入编译，登记、移除、移动源码及头文件目录后，已有构建目录与干净配置的源码／头目录集合相同，删除项不在 Ninja 链接输入中 |
| 编译 ABI | 153 个 robot-single Debug C 编译命令全部使用 `-mfloat-abi=hard -mfpu=fpv5-d16`，未出现 `-ffast-math` |
| Windows 调试源码路径 | bench ELF 的 DWARF 源名包含 `D:/RoboMasterRelated/OpenSourseProject/mc-02/...`，可匹配 Windows 工作区 |

Debug 主配置的 ARM `size` 结果（字节，`text/data/bss`，不把这些合计误当单一物理 RAM 区）如下；`RelWithDebInfo` 同列格式记录在 ELF 中，可用相同命令复现。日志工作区 `.log_workspace` 为 14,656 字节 DTCM，配置间一致。

| 配置 | text | data | bss |
|---|---:|---:|---:|
| bench_safe | 200,364 | 3,076 | 123,536 |
| robot single | 230,572 | 3,888 | 131,392 |
| robot chassis | 222,672 | 3,836 | 130,872 |
| robot gimbal | 217,380 | 3,888 | 127,296 |

相同工具链下，chassis 相比 single 少 7,900 字节 text，gimbal 少 13,192 字节 text；DTCM 工作区相同。`--gc-sections` 会从 ELF 清除无引用节，因此源码清单、编译数据库、链接输入和最终符号是不同层级的证据。

另按 ELF 已分配节的 VMA／LMA 汇总各内存区域（字节；FLASH 使用 LMA，RAM 使用 VMA，未计入链接器额外对齐填充）：

| Debug 配置 | FLASH | 主 RAM | DTCM | RAM_D2／RAM_D3 |
|---|---:|---:|---:|---:|
| bench_safe | 203,440 | 111,948 | 14,656 | 0／0 |
| rcdiag | 205,548 | 115,124 | 14,656 | 0／0 |
| robot single | 234,460 | 120,616 | 14,656 | 0／0 |
| robot chassis | 226,508 | 120,044 | 14,656 | 0／0 |
| robot gimbal | 221,268 | 116,520 | 14,656 | 0／0 |

这些是当前构建产物的已分配 ELF 节合计；链接器按节边界报告时会因对齐略有增加。RelWithDebInfo 的 `text/data/bss` 可用同一 `arm-none-eabi-size` 命令复算。

`check_exercises.py --build --generator Ninja` 通过，4/4 项检查完成，学习笔记本地链接有效。CMake 增量证据位于 `build/wsl/cmake-regeneration-v3/run-20261022-132622-g9wckr71/`。本轮未运行 VSCode 调试、Ozone、下载任务、硬件或远端 CI；ELF 路径映射和 task/launch 结构只作静态检查，不代表硬件联调已验证。

## 复盘

显式源码登记把“目录里存在一个 `.c`”和“当前固件需要编译它”分开，漏掉的依赖会在干净构建中暴露，而不是被全树搜索暂时掩盖。验证同时查看生成输入和最终 ELF，避免把“编过”误当作“被选用”，也避免把链接器删除的死节误判成模块缺失。
