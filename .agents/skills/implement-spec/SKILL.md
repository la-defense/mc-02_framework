---
name: implement-spec
description: Implement a local specification as dependency-ordered TODO tasks with shared branches and local validation.
disable-model-invocation: true
---

# Implement Spec

Implement the spec linked from the repository TODO. Read its ticket-detail files and respect dependencies. Follow `docs/agents/issue-tracker.md`; related tasks share a branch. Cross-repository changes use reciprocal TODO links and separate commits.

## Steps

1. Read the specification, TODO entries, blocker details, repository guide, glossary, and relevant ADRs.
2. Work from the ready frontier. Combine tests, builds, regressions, and necessary board debugging in one validation phase. Pure software is checked locally; connect hardware only when acceptance requires board behavior and within existing user authorization. Keep board work bare-board and fire-disabled. Use TDD where a production interface needs a meaningful regression.
3. For each task, update the TODO only when acceptance is met. Save rationale and test evidence in the detail file or task report, never a second status tracker.
4. Add or substantively update at least one teaching document for every change and update its index. Cross-repository teaching remains in MC-02 and references the vision task and code SHA. Use teach on demand; a document alone is not evidence of mastery.
5. Commit finished changes using original type prefixes such as fix/feat/docs/chore, followed by a Chinese summary and body.
6. Deliver commit SHAs, verification evidence, remaining blockers, teaching links, and a brief reflection. Automatic code review, agent re-review, push, and PR are removed from this flow.
