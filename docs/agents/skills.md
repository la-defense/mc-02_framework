# Skill routing

Single source for choosing a skill. The skill bodies live in [.agents/skills/](../../.agents/skills/); this file only says which task takes which skill and how the skill is triggered. Do not restate skill instructions here, and do not add a second copy of this table to another document.

## How a skill reaches the agent

- The harness scans `<repo>/.agents/skills` before `~/.agents/skills`, so a skill installed in this repository overrides a global skill of the same name.
- A skill whose header lacks `disable-model-invocation` is offered to the model, which may load it when its `description` matches the task.
- `disable-model-invocation: true` keeps a skill out of the model catalog. The only entry point is the `/` menu or a hand-typed `/name`. A routing rule alone cannot reach such a skill; the flag has to be dropped first.
- Loading a skill injects its body for that one task. Nothing in this document is always-on context, and no skill is worth copying into AGENTS.md.

## Routing

| Task | Skill | Trigger |
|---|---|---|
| Write or edit repository source, tests, scripts, build configuration, or their documentation | [student-code-comments](../../.agents/skills/student-code-comments/SKILL.md) | Mandatory. Read before editing. |
| A defect that reproduces only under specific conditions, or a performance regression | `diagnosing-bugs` | Automatic |
| A protocol, register, datasheet, or SDK behaviour that needs a primary source | `research` | Automatic. Record the findings under `docs/workflow/`. |
| A large, foggy effort where the destination is clear but the route is not | `wayfinder` | Automatic |
| An agreed large feature that needs a specification | `to-spec` | Automatic |
| Splitting an agreed specification into dependency-ordered TODO items | `to-tickets` | Automatic |
| An incoming defect or enhancement that has no triage state yet | `triage` | Automatic |
| Implementing an agreed specification | `implement`, `implement-spec` | Explicit request |
| A behaviour regression inside the validation phase | `tdd` | Optional method, see the validation phase |
| A short concept lesson | `teach` | On demand |
| Delivery retrospection | `retro` | Major failure, phase end, or explicit request |
| Reviewing a committed diff, or writing PR text | `code-review`, `pr` | Explicit request only, never automatic |
| Polishing an article or a note | `humanizer-zh`, `no-ai-slop`, `writing-beats`, `writing-fragments`, `writing-shape` | User asks |
| Writing or revising a skill, AGENTS.md, or another agent-facing document | `writing-for-agents` | Automatic |
| Handing the session to another agent | `handoff`, `claude-handoff` | Explicit request |

`implement`, `implement-spec`, `retro`, `handoff`, `claude-handoff`, `grill-me`, `grill-with-docs`, `to-questionnaire`, `wait-what`, and the `writing-*` drafts stay manual: they need a deliberate request, and none of them may run on its own.

## Boundaries that outrank any skill

- Task state stays in [.Doc/TODO.md](../../.Doc/TODO.md). No skill may keep a second status field, and no skill may create external issues.
- The default flow has no automatic code review, dual-agent review, push, or PR; see [issue-tracker.md](issue-tracker.md). `code-review` and `pr` run only when the user asks for that work as its own task.
- Bare-board bench HIL only; generated vision commands stay fire-disabled.
- Every change ships a teaching document in [docs/学习笔记/](../../docs/学习笔记/README.md). That obligation is independent of whether `teach` is invoked.

## Local adaptations and upstream identity

Routing is defined here. Diffs from the upstream skill bodies are recorded in [docs/workflow/skill-overrides.md](../workflow/skill-overrides.md), and upstream identity is pinned in [skills-lock.json](../../skills-lock.json). A skill update must be reviewed against both files before it replaces a project copy.
