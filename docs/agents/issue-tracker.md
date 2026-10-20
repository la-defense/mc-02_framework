# Task tracker: local TODO

## Source of truth

The unchecked item in .Doc/TODO.md is the only authoritative task state. Do not keep a second status field in a specification, ticket, map, PR description, or learning document.

Each active entry has this shape:

    - [ ] **ID** [bug|enhancement] [needs-triage|needs-info|ready-for-agent|ready-for-human] Title. 验收：observable completion condition.

Completed implementation uses [x]. A rejected item uses [x] [wontfix] and a short reason. A task's stable ID remains unchanged when its title or detail changes. Firmware keeps existing R25/HIL/APP IDs; new firmware entries use MC02-####. Vision entries use VIS25-####.

## Detail files

- Specs: docs/workflow/specs/<feature>.md
- Ticket details: docs/workflow/tickets/<ID>.md
- Decision maps: docs/workflow/maps/<effort>.md
- Triage inbox and rejected requests: docs/workflow/inbox/ and docs/workflow/out-of-scope/

These files hold rationale, dependencies, evidence, and acceptance detail. They may link to a TODO item but never own its status. When one task spans both repositories, create one linked TODO item in each repository and keep each item's state in its own TODO.

## 开发流程（2026-10-08 起）

1. 明确目标和验收条件，检查相关源码、Git 状态与 TODO。复用已确认的要求和授权，只询问会影响范围、行为或安全的缺失信息。
2. 大功能或跨模块改造写规格并按依赖拆 TODO；简单修复直接在 TODO 记录验收条件，长说明才放票据文件。
3. 一组相关改动共用一个分支，允许多个关联 TODO 项在同一分支交付。新分支选择能满足真实依赖的基线，不为每个子任务强制创建叠加分支。
4. 定位并实现，统一执行下面的验证阶段。
5. 每次改动至少新增或实质更新一份教学文档；已有主题更新原篇，文档引用任务编号、相关代码和验证依据，并更新学习笔记索引。教学文档与固件改动同次提交；视觉改动的教学文档仍集中在 MC-02，两仓库分别提交并关联任务编号与代码 SHA。
6. 验收满足后更新 TODO，并提交已完成的改动。提交格式如 `fix: 修复视觉失联后旧指令残留`；fix、feat、docs、chore 等类型保留原文，摘要和正文用中文。
7. 交付提交 SHA、改动说明、验证结果、教学文档入口及遗留问题，并简短复盘。正式 retro 用于重大故障、阶段结束或用户明确要求。

默认开发流程取消代码审查、双代理复审、自动推送和 PR 创建；交付不依赖远端工具。只有用户另行明确要求审查或远端操作时，才执行对应独立任务。

## 统一验证阶段

- 将测试先行、构建、回归和必要的上板调试作为同一个验证阶段，按改动选择验证方式。验证可以与实现交替进行。
- 本仓库本机软件验收使用 WSL 主机测试和 ARM 固件构建；Windows 原生构建不作为验收要求。安全逻辑、协议和复杂故障在生产接口上增加必要的行为回归，适用时用 TDD；简单重构复用现有测试，不为字段布局或实现写法新增测试。
- 只有验收涉及板上行为、确实需要上板调试时才连接硬件。沿用用户已授权的操作范围，确认供电、连接与准确固件 SHA；烧录前备份并校验。
- MC-02 上板验证保持裸板、禁止开火和执行器动作。主机测试与寄存器观察不能代替电平、VIN 精度或温升等物理验收。
- 相关检查通过后结束验证；只有新改动、失败或具体未解决风险才扩大或重复检查。

## Skills 的选择

- triage / wayfinder 在 TODO 中管理任务，较长依据留在 docs/workflow。
- to-spec / to-tickets 只服务需要规格和依赖拆分的大功能或跨模块改造。
- implement / implement-spec 按上述流程实施，相关任务共用分支，不接入审查或 PR 链条。
- tdd 是统一验证阶段的可选方法；teach 是按需教学工具。每次提交教学文档独立于是否调用 teach，掌握记录只依据用户展示的理解。
- code-review / pr 保留为用户明确要求时才使用的独立工具，不由实现、路由或复盘自动调用。
