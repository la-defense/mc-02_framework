---
name: triage
description: Groom an unclassified defect or improvement request into the firmware or vision repository's local TODO contract.
disable-model-invocation: true
---

# Triage

Use triage for raw incoming work that is not already a specified TODO task. Existing task state is read from the current repository TODO: `.Doc/TODO.md` for MC-02, or `TODO.md` for sp_vision_25. Read the related `GLOSSARY.md`, ADRs, and prior decisions before classifying.

## Roles

Categories:

- `bug`: current behavior violates a safety or correctness contract.
- `enhancement`: new capability, maintenance, performance work, or refactoring.

States:

- `needs-triage`: priority, scope, or evidence needs review.
- `needs-info`: a missing fact prevents a safe acceptance condition.
- `ready-for-agent`: behavior and acceptance are testable without physical access.
- `ready-for-human`: physical access, a product judgment, or an external decision is required.

## Process

1. Reproduce a reported bug when a safe host or read-only path exists. Search for existing code and tests; record the code path or command used.
2. Check `docs/workflow/out-of-scope/` for a prior decision.
3. Write a concise task brief at `docs/workflow/tickets/<ID>.md` when rationale, dependencies, or reproduction detail exceeds one TODO line.
4. Add or update the unchecked TODO entry with a stable ID, one category, one state, a concrete `验收：` condition, and a link to the detail file if present.
5. For work crossing MC-02 and sp_vision_25, add one linked TODO item in each repository. Keep their states local to their respective TODOs.

The TODO is the only status source. Specs, ticket details, maps, PRs, and learning material may link to an item but never copy its state. Do not create an external issue, post a comment, or reject a request without explicit user direction.

Mark work `ready-for-human` when physical verification or an external decision is a gate. Use `needs-info` only when a specific missing answer blocks a safe, testable specification; continue independent work meanwhile.
