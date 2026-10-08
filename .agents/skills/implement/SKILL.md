---
name: implement
description: "Implement a piece of work based on a spec or set of tickets."
disable-model-invocation: true
---

Implement the work described by the user in the spec or tickets.

Read `docs/agents/issue-tracker.md` for the current development flow. A group of related changes shares a branch. Small changes use TODO acceptance directly; large features or cross-module changes use a linked specification.

Use one validation phase for tests, builds, regressions, and necessary board debugging. Run pure software checks locally; connect hardware only for acceptance that requires board behavior. Use TDD at agreed production interfaces where it provides a meaningful regression. Reuse existing checks for simple refactoring and stop once required checks pass.

Every change must include a new or substantively updated teaching document, indexed under `docs/学习笔记/README.md`; teach is on demand. Update TODO, commit the finished change with the original type prefix and a Chinese summary/body, and deliver the SHA, verification evidence, teaching link, remaining issues, and a brief reflection. The default flow ends here; review, repeated agent reviews, push, and PR are not delivery steps.
