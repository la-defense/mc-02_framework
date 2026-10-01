# Engineering workflow records

The repository TODO is the only task-state source. Keep larger specifications, per-task acceptance detail, decision maps, incoming triage notes, and rejected-request rationale under the paths in docs/agents/issue-tracker.md.

Use one task branch and PR. Commit the finished task before review. A cross-repository task has one commit/PR in each affected repository, linked by stable TODO IDs. On task completion, explain code changes, checks, and any learning-note update.

## Local skill adaptations

Project-specific changes to the installed Matt Pocock skills are recorded in [skill-overrides.md](skill-overrides.md). The skills-lock file continues to identify the upstream source version; compare local changes before updating those skills.
