---
name: implement
description: "Implement a piece of work based on a spec or set of tickets."
disable-model-invocation: true
---

Implement the work described by the user in the spec or tickets.

Use /tdd where possible, at pre-agreed seams.

Run typechecking regularly, single test files regularly, and the full test suite once at the end.

Commit the complete task change on its task branch before review. Then use /code-review against the branch merge base. Review fixes go in a follow-up commit and must be reviewed again. Close out with commit SHA, verification evidence, and learning-note updates.
