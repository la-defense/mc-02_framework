# Local task detail template

Save substantial task context under `docs/workflow/tickets/<ID>.md` and link it from TODO. Do not copy task status here.

```md
# <ID>: <short task title>

**Category:** bug / enhancement
**Current behavior:** what happens now, with a code path or reproduction when available.
**Desired behavior:** observable behavior after completion.
**Key interfaces:** domain types or behavioral contracts to find; avoid line numbers.
**Acceptance:** independent tests or observations that prove completion.
**Blocked by:** TODO IDs that must complete first, or None.
**Out of scope:** adjacent behavior not included in this task.
```
