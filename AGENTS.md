# Repository guide

This is the MC-02 STM32H723 firmware repository. Start here, then follow the task-specific references below.

- Task state lives only in [.Doc/TODO.md](.Doc/TODO.md). Each open item has one stable ID, one category, one triage state, an acceptance condition, and optionally a linked specification or ticket detail.
- Tracker behavior, statuses, cross-repository work, and local skill adaptations are defined in [docs/agents/issue-tracker.md](docs/agents/issue-tracker.md).
- Firmware vocabulary and design decisions live in [GLOSSARY.md](GLOSSARY.md) and [docs/adr/](docs/adr/).
- Learning material for both firmware and sp_vision_25 is indexed in [docs/学习笔记/README.md](docs/学习笔记/README.md). Use the course workspace in docs/课程/自瞄链路 when a new concept needs a short lesson or practice.
- Hardware safety boundary: MC-02 bench HIL is bare-board only. Keep all generated vision commands fire-disabled; do not use a workflow that actuates the robot.
- For changes, use the installed engineering skills where their trigger applies. Keep a task on one branch and commit its finished work before code review.
- Git 提交的标题和正文统一使用中文；代码标识符、任务编号、产品名及必要的技术术语保留原文。

The skills are installed in this repository only. When a task touches sp_vision_25, use this repository's workflow and learning references while switching the code worktree to the vision repository.
