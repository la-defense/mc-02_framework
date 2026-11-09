# 文档类文件盘点（2026-11-09）

## 0. 本轮范围

只读盘点，**没有删除、移动或重写任何现有文件**。本轮只新增这份报告和逐文件清单，供用户决定后续动作。

- 盘点基线：`e9993f8`（`docs/review-gate`）。
- tracked 状态按该提交统计；untracked 状态按主工作树 `D:\RoboMasterRelated\OpenSourseProject\mc-02` 统计，因为盘点 worktree 不含本地厂商资料、缓存和未跟踪文件。
- 文档候选：`.md`、`.txt`、`.rst`、`.adoc`、`.pdf`、`.html`、`.docx`、`.xlsx`、`.pptx`。
- 已剔除：`CMakeLists.txt`、`*_config.txt`、`LICENSE.txt`、`requirements-filter-plots.txt`、缓存 README；这些是构建配置/许可/缓存，不是文档。
- 不计入：源码、头文件、构建产物、`.git`、`build/`、`tools/referee_sim/`、根目录 `资料/`。
- 4 个 CSV 证据文件和 2 个无扩展名 `LICENSE`（根目录、FreeRTOS 源目录）不在本轮范围。
- 完整逐文件清单：[documentation-inventory-20261109.tsv](documentation-inventory-20261109.tsv)。
- 图片资产不在 358 行清单内；PDF 和课程 HTML 已包含在 305 个项目文档中，§7 只是按格式重列，不重复计数。

## 1. 结论摘要

| 分类 | 数量 | Git | 建议 |
|---|---:|---|---|
| 项目维护文档，已跟踪 | 255 | 已跟踪 | 保留 |
| 项目文档，需复核后保留 | 50 | 已跟踪 | 保留，核对过时内容 |
| 厂商/本地资料 | 53 | 未跟踪 | 不进 Git；整目录补 ignore，不移动 |
| 文档图片资产 | 70 | 已跟踪 | 按引用保留；先审计再删 |
| 已跟踪 PDF | 5 | 已跟踪 | 保留；可评估 Git LFS |
| 课程 HTML | 10 | 已跟踪 | 保留，作为离线课程产物 |

项目维护文档合计 **305** 个：255 个直接保留，50 个需要逐项复核。另有 53 个厂商文档只在本地使用（`.Doc/资料` 工作文件约 659 MB，另有约 128 MB 的独立 `.git`）。

## 2. 分类规则

- **需要 + 进 Git**：项目自己维护、会被代码/流程/教学引用的文档。
- **需要 + 只本地**：未跟踪的厂商手册、协议 PDF、原理图、3D 模型、厂商例程；体积大或有版权，不随仓库分发。
- **已入库例外**：已经跟踪的 5 个 PDF 保留现状；本规则只约束未跟踪新增资料。
- **需要复核**：模块说明、历史审查、个人环境说明；先核对是否仍与当前代码/流程一致，再决定保留、归档或删除。
- **不需要 + 不进 Git**：构建产物、缓存、误生成文件、编辑器临时文件。
- **不属于文档**：`CMakeLists.txt`、`*_config.txt`、`LICENSE.txt` 等构建/纯文本许可文件；`LICENSE.md` 作为上游包文档随包保留。

## 3. 需要并进 Git 的文档

这些文件已经跟踪，默认保留；具体逐文件见 TSV。

| 类别 | 数量 | 位置/说明 |
|---|---:|---|
| 项目入口与任务状态 | 4 | `README.md`、`AGENTS.md`、`GLOSSARY.md`、`.Doc/TODO.md` |
| 工作流规则 | 4 | `docs/agents/` |
| 工作流记录 | 99 | `docs/workflow/specs/`、`tickets/`、`maps/`，含 2 个根文件 |
| 教学文档 | 29 | `docs/学习笔记/` |
| 课程与练习 | 28 | `docs/课程/自瞄链路/`，已剔除 1 个 CMakeLists；含 10 个 HTML |
| 仓库技能 | 61 | `.agents/skills/` |
| 上游包文档 | 25 | `Drivers/`、`Middlewares/` 的 HowTo/Abstract/readme/许可/来源说明；已剔除 CMake/config 与 LICENSE.txt |
| 硬件证据 | 3 | `docs/硬件定位/` |
| ADR | 1 | `docs/adr/` |
| 审查报告 | 1 | `docs/代码审查报告_MC-02.md` |

## 4. 需要复核后保留的 50 个

这些文件已跟踪，但可能有过时内容、历史定位或环境特定信息。建议逐个核对，不直接删除。

| 文件 | 类别 | 建议 |
|---|---|---|
| `docs/6S上电安全审查_2026-10-04.md` | docs-other | 历史硬件安全审查，保留归档 |
| `docs/README.md` | docs-other | docs 目录索引，保留 |
| `docs/development/VSCode与Ozone调试指南.md` | docs-other | 开发环境指南，保留并核对版本 |
| `docs/history/seasky/README.md` | docs-other | 历史参考，保留归档 |
| `docs/history/seasky/湖南大学RoboMaster电控组通信协议.md` | docs-other | 历史协议参考，保留归档 |
| `docs/工程复审_2026-09-30.md` | docs-other | 历史工程复审，保留归档 |
| `docs/本机WSL自瞄联调部署记录.md` | docs-other | 本机环境记录，保留但标注环境特定 |
| `application/APP层应用编写指引.md` | module-doc | 模块说明，保留并核对是否过时 |
| `application/application.md` | module-doc | 模块说明，保留并核对是否过时 |
| `application/chassis/chassis.md` | module-doc | 模块说明，保留并核对是否过时 |
| `application/cmd/robot_cmd.md` | module-doc | 模块说明，保留并核对是否过时 |
| `application/gimbal/gimbal.md` | module-doc | 模块说明，保留并核对是否过时 |
| `application/shoot/shoot.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/adc/bsp_adc.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/bsp.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/can/bsp_can.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/dwt/bsp_dwt.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/flash/bsp_flash.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/gpio/bsp_gpio.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/iic/bsp_iic.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/log/bsp_log.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/pwm/bsp_pwm.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/spi/bsp_spi.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/usart/bsp_usart.md` | module-doc | 模块说明，保留并核对是否过时 |
| `bsp/usb/bsp_usb.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/BMI088/bmi088-datasheet.pdf` | module-doc | 厂商数据手册，保留；可评估 Git LFS |
| `modules/BMI088/bmi088.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/TFminiPlus/tfminiplus.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/alarm/buzzer.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/algorithm/algorithm.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/can_comm/can_comm.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/daemon/daemon.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/encoder/encoder.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/imu/ins_task.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/lcd/README.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/master_machine/master_process.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/message_center/message_center.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/module.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/motor/DJImotor/dji_motor.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/motor/HTmotor/HT04.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/motor/HTmotor/驱动器硬件说明.pdf` | module-doc | 厂商硬件说明，保留；可评估 Git LFS |
| `modules/motor/LKmotor/LK-TECH电机CAN协议说明V2_3.pdf` | module-doc | 厂商协议，保留；可评估 Git LFS |
| `modules/motor/LKmotor/LK_motor.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/motor/servo_motor/servo_motor.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/oled/oled.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/referee/referee.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/remote/remote.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/standard_cmd/standard_cmd.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/super_cap/super_cap.md` | module-doc | 模块说明，保留并核对是否过时 |
| `modules/unicomm/unicomm.md` | module-doc | 模块说明，保留并核对是否过时 |

## 5. 只本地保留、不进 Git 的 53 个

全部位于 `.Doc/资料/`。该目录是**独立 Git 仓库**（4,943 个工作文件、约 659 MB，另有约 128 MB 的 `.git`），父仓库只看到一条未跟踪目录。下面是按文档后缀筛出的 53 个文件；不是整个目录只有 53 个文件。

| 子目录/文件 | 数量 | 处理 |
|---|---:|---|
| `例程/` | 38 | 厂商 Cube/CMSIS 例程中真正的 readme/HowTo/Abstract；本地参考，不进 Git |
| `说明书/` | 6 | 芯片/模块手册；本地参考，不进 Git |
| `2D图纸/` | 5 | 原理图/屏规格书；本地参考，不进 Git |
| `README*.md` | 3 | 资料目录说明；如需要，整理成 `docs/资料索引.md` 后再跟踪 |
| RoboMaster 通信协议 PDF | 1 | 本地参考，不进 Git |

建议：在 `.gitignore` 增加 `/.Doc/资料/`，整目录保持本地。不要直接 `git add`；也不建议把约 659 MB 工作文件、含独立 `.git` 的目录移动到根目录 `资料/`，除非用户明确确认。根目录 `资料/` 目前只有 ignore 规则，目录本身不存在。

## 6. 不需要、不应进 Git 的文件

| 文件/目录 | 现状 | 建议 |
|---|---|---|
| `.pytest_cache/README.md` | 由 `.pytest_cache/.gitignore` 忽略 | 无需入库；清理缓存即可 |
| `__pycache__/`（3 处） | untracked 缓存 | 加入 `.gitignore`，不进 Git |
| `" 2"` | untracked，`cmake --list-presets` 输出 | 确认后删除；不要提交 |
| `/build/`、`/build-perf/`、`/build-test/`、`/build-audit/` | `.gitignore` 已逐条列出 | 保持；没有通用 `build-*/` 规则 |
| `tools/referee_sim/` | 已忽略的独立仓库 | 保持 |
| 根目录 `资料/` | 只有 ignore 规则，目录尚未创建 | 如迁移厂商资料，会命中 `/资料/` 规则 |

已跟踪文档中，本轮没有判定任何一份为“不需要”；50 份只是“需要复核”。

## 7. 文档资产

- **70 个已跟踪图片**：`.assets/` 61 个、`docs/` 6 个、`modules/` 3 个；包含被 `docs/development/VSCode与Ozone调试指南.md` 引用的 1 个 `.webp`。
- **5 个已跟踪 PDF**：`docs/硬件定位/` 2 个、`modules/BMI088/` 1 个、`modules/motor/HTmotor/` 1 个、`modules/motor/LKmotor/` 1 个；已包含在 305 个项目文档中。
- **10 个课程 HTML**：`docs/课程/自瞄链路/lessons/` 5 个、`reference/` 5 个；已包含在 305 个项目文档中。
- 建议：图片先做引用审计，再决定是否删除未引用项；PDF 保留现状，后续可评估 Git LFS。本轮不删任何资产。

## 8. 建议的后续动作

**以下是建议，本轮均未执行；任何删除或移动都必须经用户确认后另开任务。**

1. 仅建议补 `.gitignore`：`/.Doc/资料/`、`__pycache__/`；不改动现有文件。
2. 逐项复核 50 个模块/历史文档，标记“保留 / 归档 / 更新 / 删除”。
3. 对 61 个 `.assets` 图片做引用审计；没有引用的再讨论删除。
4. 未跟踪的厂商资料只本地保留；已入库的 5 个 PDF 和课程 HTML 按现状保留。
5. 确认 `" 2"` 来源后再删除；不要直接提交。

## 9. 逐文件清单

[documentation-inventory-20261109.tsv](documentation-inventory-20261109.tsv) 包含 358 行数据，每行一个真正的文档候选，列为：

```text
path    category    git    decision    reason
```

其中 `git` 为 `tracked`/`untracked`，`decision` 为 `keep`、`keep-review` 或 `local-only`。本报告自身不计入 358 行；提交后仓库会多出这份报告和清单。
