# ADR-0001: TODO is the task-state source of truth

## Status

Accepted

## Decision

Each repository keeps its own hand-edited TODO. Specifications, ticket detail, PRs, learning notes, and HIL reports may describe a task but do not maintain a second status. Cross-repository work has linked IDs, one in each repository.

## Consequences

Agents must read the repository tracker configuration before triage, planning, implementation, and review. A small checker validates IDs, categories, states, and acceptance conditions. Task state remains reviewable in normal Git history.
