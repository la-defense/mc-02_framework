# MC02-0145 验证记录

本记录只描述软件检查；任务状态以 [TODO](../../../../.Doc/TODO.md) 为准。

## 僵死现场

2026-11-15 20:00:28 同时启动六条 `wsl.exe`（`cmake --build /home/mc02/build/sentinel-sanitize` 两条、`verify.py --firmware` 两条、`verify.py --host` 两条），到 20:32 全部没有返回。用 `wsl --system -u root` 从系统发行版旁路观察：

- 负载 0.02，内存 539 MB 已用（共 7943 MB），swap 未使用，pty 占用 10／4096，进程 71 个，线程 114 个。没有资源耗尽的迹象。
- `dmesg` 最后一条停在开机后 266 秒，即 20:00:29；此前有一次发行版重启（t=184 卸载，t=208 重新挂载）。没有 OOM、hung task 或 soft lockup 记录。
- 八个 `Relay` 的栈是 `unix_stream_read_generic`，等的是 `/run/WSL/418_interop` 一类 socket；会话已分到伪终端，Windows 侧对应的 `wslhost.exe` 存在但不推进。
- 构建目录最新文件时间是 19:58，晚于它的命令没有产生任何输出。
- `wsl --shutdown` 后重新进入 Ubuntu-22.04 用时 3.7 秒，会话恢复。

## 改动检查

- `tests/tools/test_verify.py` 12 项通过，新增 6 项覆盖探测判定和超时报告。
- 工具回归 187 项通过，1 项跳过。
- `python tools/workflow/verify.py --fast` 在 Windows 入口完成探测、转交和三个阶段，用时 25.0 秒，退出码 0。
- TODO 契约与文档链接检查通过。

## 边界

- 僵死原因没有确定。内核没有留下崩溃或卡住记录，只能确认会话建立停在 interop 交接。11-04 曾把同类现象归给自编译内核，本次运行的是官方内核（`#1 … Tue Oct 6 19:13:07 UTC 2026`，gcc 13.2.0），那个结论不成立。
- 探测只覆盖 `verify.py`，直接调用 `wsl.exe` 的入口仍然没有时限。
- 超时终止的是 Windows 侧进程，虚拟机里的命令是否结束未验证。
