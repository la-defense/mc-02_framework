# Repository guide

This is the MC-02 STM32H723 firmware repository. Start here, then follow the task-specific references below.

- Task state lives only in [.Doc/TODO.md](.Doc/TODO.md). Each open item has one stable ID, one category, one triage state, an acceptance condition, and optionally a linked specification or ticket detail.
- 开发流程、TODO 状态、相关改动共用分支、统一验证阶段及跨仓库交付，以 [docs/agents/issue-tracker.md](docs/agents/issue-tracker.md) 为准。
- Firmware vocabulary and design decisions live in [GLOSSARY.md](GLOSSARY.md) and [docs/adr/](docs/adr/).
- 编写或修改自有代码、测试、脚本、构建配置及配套说明前，先阅读并遵循 [student-code-comments](.agents/skills/student-code-comments/SKILL.md)。
- 每次改动必须提交至少一份新增或实质更新的教学文档，入口为 [docs/学习笔记/README.md](docs/学习笔记/README.md)。已有主题更新原篇；teach 按需使用，课程工作区为 docs/课程/自瞄链路。
- Hardware safety boundary: MC-02 bench HIL is bare-board only. Keep all generated vision commands fire-disabled; do not use a workflow that actuates the robot.
- 普通开发完成验证、教学文档和 TODO 后提交，并简短复盘交付；默认流程取消代码审查、双代理复审、推送及 PR 环节。skills 按本地适配规则使用。
- Git 提交使用 fix、feat、docs、chore 等原文类型前缀；摘要和正文用中文。代码标识符、任务编号、产品名及必要技术术语保留原文。

The skills are installed in this repository only. When a task touches sp_vision_25, use this repository's workflow and learning references while switching the code worktree to the vision repository.
