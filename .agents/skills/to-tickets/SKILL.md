---
name: to-tickets
description: Break a reviewed specification into dependency-ordered TODO tasks with concrete acceptance conditions and optional local detail files.
disable-model-invocation: true
---

# To Tickets

Read the spec, the target repository TODO, `GLOSSARY.md`, and related ADRs. Split the work into independently verifiable vertical slices. Prefer one task per branch and PR; cross-repository work gets one linked TODO item in each repository.

## Process

1. Reuse settled requirements and decisions. Ask only where scope, behavior, safety, or task dependency remains materially unclear.
2. Draft the task graph in dependency order. Every task needs a stable ID, category, state, `验收：` condition, and explicit blockers where relevant.
3. Present the proposed task titles, outputs, acceptance conditions, and blocking edges. Refine only when the current request leaves a material choice open.
4. Record status in the repository TODO. Store longer context in `docs/workflow/tickets/<ID>.md` and link it from the TODO entry. Detail files may repeat acceptance rationale but never the status.
5. Keep the frontier visible: an item is ready when all listed blockers are complete and its TODO state is `ready-for-agent`.

Do not create external issues, `.scratch` ticket copies, or a second status field. Preserve existing task IDs and completed history.
