# Engineering workflow records

The repository TODO is the only task-state source. Keep larger specifications, per-task acceptance detail, decision maps, incoming triage notes, and rejected-request rationale under the paths in docs/agents/issue-tracker.md.

开发流程以 [issue-tracker.md](../agents/issue-tracker.md) 为准（2026-10-08 起）：相关改动共用分支，测试、构建和必要的上板调试合并为验证阶段；每次改动提交教学文档，完成本地 Git 提交并简短复盘交付。默认流程已取消代码审查、双代理复审、推送及 PR。跨仓库交付使用稳定 TODO 编号和关联提交 SHA。

## 本地工具

- `python3 tools/workflow/verify.py --fast`：TODO 契约、文档链接和 Python 工具回归；`--host`、`--firmware`、`--all` 按改动扩展。
- `check_docs_links.py`：检查根说明、`.Doc` 和 `docs` 的相对链接；独立仓库和厂商资料在 `docs_links_allowlist.json` 里说明。
- `check_todo.py --summary`：未完成项的编号查重、状态和正文检查，并输出按状态／分类／章节的分布。
- `presets.py --check-cmake`：核对十种官方预设与 `CMakePresets.json`，防止 CI、脚本和文档各自维护一份清单。
- 证据摘要放 `docs/workflow/tickets/<ID>-evidence/README.md`；`build/` 和 `/tmp` 只放原始日志与产物。

## Local skill adaptations

Project-specific changes to the installed Matt Pocock skills are recorded in [skill-overrides.md](skill-overrides.md). The skills-lock file continues to identify the upstream source version; compare local changes before updating those skills.
