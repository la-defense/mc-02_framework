---
name: wayfinder
description: Map a large, uncertain engineering effort into local decision tasks when the destination is clear but the route is not.
disable-model-invocation: true
---

# Wayfinder

Use this for an effort too uncertain to spec in one conversation. For a well-scoped feature, use `grill-with-docs → to-spec → to-tickets` instead.

## Source of truth

Task status lives only in the repository TODO: `.Doc/TODO.md` in MC-02 and `TODO.md` in sp_vision_25. Store the durable decision map in `docs/workflow/maps/<effort>.md`; detailed decision briefs belong in `docs/workflow/tickets/<ID>.md`. Never put a second status field in the map.

## Process

1. Read the relevant TODO, `GLOSSARY.md`, ADRs, and existing workflow map.
2. State the destination, what is known, what remains uncertain, and which decisions could change the route.
3. Ask focused questions only for decisions that cannot be resolved from available code or authoritative sources. Resolve one decision at a time.
4. For each open decision, add a TODO item with a stable ID, category, state, and observable acceptance condition. Put context and blockers in its ticket-detail file; mark ready only when the decision can be made.
5. Update the map with decision rationale and links to TODO IDs. Keep implementation tasks in TODO as soon as the answers make them specifiable.
6. When the route is clear, link the resulting specification and dependency-ordered tasks; the map then becomes a navigation aid, not a task tracker.

For cross-repository decisions, create a reciprocal item in both TODO files. Do not post external issues or comments. Each independently deliverable task uses its task branch, is committed before code review, and is delivered through a PR when remote access is available.
