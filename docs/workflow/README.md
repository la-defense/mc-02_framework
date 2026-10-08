# Engineering workflow records

The repository TODO is the only task-state source. Keep larger specifications, per-task acceptance detail, decision maps, incoming triage notes, and rejected-request rationale under the paths in docs/agents/issue-tracker.md.

开发流程以 [issue-tracker.md](../agents/issue-tracker.md) 为准（2026-10-08 起）：相关改动共用分支，测试、构建和必要的上板调试合并为验证阶段；每次改动提交教学文档，完成本地 Git 提交并简短复盘交付。默认流程已取消代码审查、双代理复审、推送及 PR。跨仓库交付使用稳定 TODO 编号和关联提交 SHA。

## Local skill adaptations

Project-specific changes to the installed Matt Pocock skills are recorded in [skill-overrides.md](skill-overrides.md). The skills-lock file continues to identify the upstream source version; compare local changes before updating those skills.
