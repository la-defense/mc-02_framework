# WORKFLOW-20261028 证据

对应 [工作流票据](../WORKFLOW-20261028.md)。所有命令在 WSL（Ubuntu 22.04）中执行，源码基线为提交 `fbb0b58`，未连接开发板，未烧录。

## 环境

- CMake 3.22.1，Ninja 1.10.1
- `arm-none-eabi-gcc` 10.3.1 20210621
- Python 3.10.12，PyYAML 5.4.1（仅用于本地解析工作流）
- 主机编译器：GCC 11.4.0
- 本机没有 ccache；`sudo` 需要密码，因此未安装

## 命令与结果

```sh
# 1. Python 工具回归
python3 -m unittest discover -s tests/tools -p 'test_*.py'
# Ran 159 tests，OK（4 项按既有硬件条件跳过）

# 2. 文档链接和 TODO 摘要
python3 tools/workflow/check_docs_links.py
# 153 个文档、2063 条链接，缺失 0，提醒 8

python3 tools/workflow/check_todo.py .Doc/TODO.md --summary
# 未完成 71，已完成 106；状态 needs-info 17、needs-triage 19、
# ready-for-agent 25、ready-for-human 10

# 3. 预设清单与 CMakePresets
python3 tools/workflow/presets.py --check-cmake
# 预设清单与 CMakePresets 一致

# 4. I²C 专项（主机）
cmake -S tests/host -B /tmp/mc02-host -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/mc02-host --target iic_registration_test --parallel 12
ctest --test-dir /tmp/mc02-host -R iic --output-on-failure
# 4/4 通过

# 5. bench_safe ARM 构建与源码隔离
cmake --preset bench-debug -B /tmp/mc02-arm
cmake --build /tmp/mc02-arm --parallel 12
python3 tools/workflow/check_firmware_profile.py \
  /tmp/mc02-arm/Basic_Framework_MC02_bench_safe.elf \
  --profile bench_safe --role single --build-dir /tmp/mc02-arm
# 161 步通过；PASS

# 6. 工作流 YAML 解析
python3 -c "import yaml; yaml.safe_load(open('.github/workflows/c-cpp.yml'))"
# yaml ok 5.4.1

# 7. 完整主机回归和固件构建
time python3 tools/workflow/verify.py --host --build-root /tmp/mc02-host-verify
# 退出 0；Debug/Release 各 421 个 CTest

time python3 tools/workflow/verify.py --firmware --presets bench-debug --build-root /tmp/mc02-fw-verify
# [firmware] 通过，用时 43.7s；profile 检查 PASS

# 8. 归档（本工作树由 Windows Git 创建，WSL 需要显式 GIT_DIR）
GIT_DIR=/mnt/d/RoboMasterRelated/OpenSourseProject/mc-02/.git/worktrees/mc-02-workflow \
GIT_WORK_TREE=/mnt/d/RoboMasterRelated/OpenSourseProject/mc-02-workflow \
python3 tools/workflow/verify.py --archive --presets bench-debug --build-root /tmp/mc02-fw-verify
# [archive] 通过，用时 28.4s
# ZIP: mc02-bench-debug-fbb0b580c58a-d38c1c1d...-a88277b2.zip
# SHA-256: 82469609bb3ca008ea6895e234ff4b98a5aeed778e654828421145a2d1e57bb8
```

`GIT_DIR` 只用于这个由 Windows Git 创建、带绝对 `gitdir` 路径的工作树；普通 checkout 不需要。它不影响归档内容，只让 WSL 里的 `git` 找到同一个提交。

## 源分支增量反馈

2026-10-28 的源分支通过更新时间戳触发增量重建，没有改动 `bsp/iic/bsp_iic.c` 的源码内容：主机重新构建受影响目标并运行421项 CTest，用时26.8秒；ARM 重新编译一个对象、链接并检查源码隔离，用时4.9秒。这是构建增量反馈的历史数据，整合分支当前复验见[工作流票据](../WORKFLOW-20261028.md#整合后本机验证)。

## 链接检查耗时

| 版本 | 命令 | 耗时 |
|---|---|---|
| 逐条 `Path.resolve()` | `time python3 tools/workflow/check_docs_links.py` | 约 10.0s（`real`） |
| 目录缓存 + 文本路径比较 | 同上 | 约 0.9s（`real`） |

两者检查同一批 2000 余条链接，缺失均为 0。耗时差异主要来自 `/mnt/d` 上逐段 `resolve()` 的文件系统查询；缓存按目录读取一次文件名，路径包含关系改为文本比较。

## 未覆盖

- 没有运行完整十种 ARM 预设和归档（只跑 `bench-debug`）。
- 没有在 GitHub Actions 上运行新工作流，也没有推送。
- 没有安装 ccache，缓存命中率和编译时间未测。
- 没有上板，未验证 10 位 I²C 从机的电平和波形。
