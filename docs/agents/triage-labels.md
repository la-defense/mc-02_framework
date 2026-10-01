# TODO category and state vocabulary

| Role | Allowed value | Meaning |
|---|---|---|
| Category | bug | Existing behavior is broken, unsafe, or inconsistent with its contract |
| Category | enhancement | New capability, maintenance, performance, or refactoring |
| State | needs-triage | Scope, priority, or evidence still needs maintainer review |
| State | needs-info | A missing fact prevents safe specification |
| State | ready-for-agent | Acceptance is testable and work can start |
| State | ready-for-human | Requires physical access, product judgment, or an external decision |
| State | wontfix | Closed with a short reason in TODO |

Every open item has exactly one category and one active state. A completed item is checked and does not carry an active state.
