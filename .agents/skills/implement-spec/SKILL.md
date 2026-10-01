---
name: implement-spec
description: Implement a local specification as dependency-ordered, testable TODO tasks on a task branch.
disable-model-invocation: true
---

# Implement Spec

Implement the spec linked from the repository TODO. Read all ticket-detail files and respect the task graph. Each independently deliverable task has a task branch and PR; a cross-repository task has a separate commit and PR in both repositories, with reciprocal TODO links.

## Steps

1. Read the specification, TODO entries, blocker details, repository guide, glossary, and relevant ADRs.
2. Work from the ready frontier. Use test-first work where a stable seam exists. Keep hardware-facing work fire-disabled and simulator-only unless the corresponding manually gated HIL is explicitly being run.
3. For each task, update the TODO only when acceptance is met. Save rationale and test evidence in the detail file or task report, never a second status tracker.
4. Commit the complete task on its task branch before code review. Run `code-review` on the committed diff from the branch merge base. Address findings in a follow-up commit and review again.
5. When remote access is available, open or update a PR and link its report. If remote access is unavailable, leave the branch ready to push and state what external step remains.
6. Close out with commit SHAs, test evidence, remaining blockers, and updated learning notes. A written lesson is not evidence that the user has mastered it.
