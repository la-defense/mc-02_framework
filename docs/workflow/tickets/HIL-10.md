# HIL-10：运行态 SWD 快照不得停核

状态仅由 [.Doc/TODO.md](../../../.Doc/TODO.md) 管理。

## 已确认的问题

旧 `probe_snapshot_live.py` 通过 GDB `target extended-remote` 在线附着并读取全局变量。实机测试中，即使 `gdb-attach` 和 `gdb-detach` 回调为空、关闭 GDB memory map，并设置 `interrupt-on-connect off`，目标仍报告 `halted due to debug-request`。因此旧路径不能用于运行态观测。

在同一 CMSIS-DAP 链路上，独立 OpenOCD Telnet 会话先执行 `targets`，再读取 `0x24000000`，最后再次执行 `targets`，前后均报告 `running`。串口被动采集也持续收到 CRC 正确的状态帧。该实测支持改用 Telnet 只读命令，但不代表任何地址或 OpenOCD 配置都天然安全。

## 实施要求

- GDB 只离线读取 ELF 符号表以解析 RAM 变量地址，不连接目标。
- 在线读取只允许 `targets`、对链接脚本 RAM 区域内的指定 ELF 符号地址执行单字节 `mdb` 或单字 `mdw`；地址和读取宽度必须同时匹配解析出的符号清单。
- 每次快照在读内存前后检查 `mc02.cpu0` 为 `running`；若连接前或读取后不满足，立即报错退出。不得自动发送 `resume`、`halt`、`reset` 或任何写内存命令。
- OpenOCD 配置仅监听 `127.0.0.1`，禁用 GDB/Tcl 端口，移除 STM32H7 `examine-end` 回调中的 DBGMCU 写入，并使用已实测的 400 kHz SWD 频率。
- 主机测试使用假 OpenOCD Telnet 服务，验证命令白名单、目标状态检查、超时、符号/内存解析和复位/计数回退分类。
- 裸板验收连续采样时核心前后保持运行、COM14 状态帧持续且没有复位。仅被动接收串口数据，不发送控制帧。

## 边界

本任务修复快照工具的停核风险。历史长时间 CMSIS-DAP USB I/O 错误若无原始日志，不推断其根因；工具报告计数回退时继续区分已观察到的复位证据和原因未明的回退。

## 实施与验证结果

- `probe_snapshot_live.py` 使用离线 GDB 解析 ELF 地址，在线仅连接 `127.0.0.1` 的 Telnet 端口；每个字段读取后立即再次检查目标状态，绝不发送恢复命令。
- OpenOCD 配置清空 `examine-end` 回调，关闭 GDB/Tcl 服务，并在加载 STM32H7 脚本后将 SWD 频率设为 400 kHz。
- 主机回归：`python -m unittest discover -s tests/tools -p 'test_*.py'`，11 项通过；覆盖命令白名单、目标状态检查、超时、地址/内存解析与计数变化分类。
- 产物：`cmake -S . -B build/hil10-snapshot -G Ninja -DCMAKE_BUILD_TYPE=Debug -DMC02_PROFILE=bench_safe` 后 `cmake --build build/hil10-snapshot --parallel 4` 成功。仅编译，没有烧录。
- 裸板实测：CMSIS-DAP 识别到 STM32H723/Cortex-M7；OpenOCD 显示 400 kHz、GDB/Tcl 端口禁用。运行 5 次快照，`uwTick` 连续递增、IWDG 标志为 0、视觉 CRC 计数保持 8；每次 RAM 字段读取前后均报告 `mc02.cpu0 running`。同期被动采集 COM14 5 秒，收到 1000 个 CRC 正确的 43 字节 `SP` 状态帧，均为 `mode=0`。没有发送串口数据、停核、复位或烧录。

## 2026-10-07 后续加固与复测

- 快照字段由单一 `SnapshotField` 表定义符号表达式、OpenOCD 命令和读取宽度；解析及读取入口都拒绝缺失字段、额外字段、非对齐地址及 STM32H723 链接脚本 RAM 区域外地址。Telnet 只接受 `targets`，或与 ELF 符号地址及宽度完全匹配的单值 RAM 读取。
- 主机回归增至 14/14，新增 RAM 边界、精确符号白名单、读取宽度及 IWDG/tick 独立分类覆盖。当前 `bench_safe` ELF 的离线 GDB 符号解析成功。
- CMSIS-DAP 以 400 kHz 重启 OpenOCD 后进行 3 轮快照：`uwTick` 为 1434371→1434701→1435020，IWDG 标志保持 0，视觉 CRC 计数保持 8；每个字段读取前后均报告目标运行中。没有烧录、串口发送、停核、复位或 RAM 写入。
- 限制：这证明当前配置下短时快照链路工作且未观察到停核/复位；历史长时 USB I/O 错误仍需其原始日志分析。构建日志中有既存 `volatile` 限定和未使用变量警告，与本任务无关。
