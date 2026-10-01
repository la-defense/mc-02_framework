---
name: setup-matt-pocock-skills
description: Reconfigure the project's installed engineering skills only when the user asks to change its TODO tracker, triage vocabulary, or domain-document layout.
disable-model-invocation: true
---

# Project Skill Configuration

This repository already uses a local TODO tracker and single-context domain docs. Read `docs/agents/issue-tracker.md`, `docs/agents/triage-labels.md`, `docs/agents/domain.md`, and `docs/workflow/skill-overrides.md` before changing configuration.

## Current configuration

- MC-02 task state: `.Doc/TODO.md`.
- sp_vision_25 task state: its repository-root `TODO.md`.
- Specifications and ticket details: `docs/workflow/specs/` and `docs/workflow/tickets/`; these never own task status.
- Domain vocabulary and decisions: repository-root `GLOSSARY.md` and `docs/adr/`.
- Agent skills: installed only in MC-02. Start the session here, then switch to the vision worktree for vision code.

## Change procedure

When the user explicitly changes one of these choices, update the relevant `docs/agents/` file, skill override, and repository guide together. Preserve all TODO history and IDs. Do not re-run a setup interview for choices already recorded here.
