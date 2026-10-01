# Issue tracker: local TODO

Task state is stored only in the target repository's TODO file: `.Doc/TODO.md` for MC-02 and `TODO.md` for sp_vision_25.

## Conventions

- An open TODO item has a stable ID, category, active state, and concrete acceptance condition.
- Specs live in `docs/workflow/specs/`; optional ticket detail lives in `docs/workflow/tickets/<ID>.md`.
- Decision maps live in `docs/workflow/maps/` and link to TODO IDs without copying status.
- Rejected-request rationale lives in `docs/workflow/out-of-scope/`.
- Cross-repository tasks have reciprocal TODO links, with status stored locally in each repository.

When asked to find a task, search TODO by ID or domain phrase and follow its linked detail. Never create `.scratch` files, external issues, or a second status field.
