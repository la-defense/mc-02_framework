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

## Skill operations

- Triage changes category/state and adds an acceptance condition directly in TODO.
- To-spec writes a feature specification in docs/workflow/specs.
- To-tickets creates TODO entries in dependency order and optional detail files in docs/workflow/tickets.
- Wayfinder keeps its decision map in docs/workflow/maps and records open/complete status only in TODO.
- Code review resolves its originating spec/ticket from the task ID, branch name, or PR description; use the merge base as fixed point.
- One task uses one task branch and one PR. Commits are made before review; review fixes use a follow-up commit and are reviewed again.
