# Project-specific skill adaptations

The installed skills are pinned to mattpocock/skills in skills-lock.json. These local changes adapt tracker paths, C/C++ exercises, and teaching triggers to this repository. Do not replace them during a skill update without reviewing the diff.

| Skill | Project adaptation |
|---|---|
| ask-matt | Routes task tracking through each repository TODO and includes the teaching / learning-note path. |
| setup-matt-pocock-skills | Reads the committed local tracker and domain configuration without asking already-settled setup questions. |
| to-spec / to-tickets | Large features and cross-module changes get specs and dependency-ordered TODO items; simple changes use TODO acceptance directly. Related changes share a branch. |
| triage / wayfinder | Uses TODO states and docs/workflow notes instead of GitHub issue labels, .scratch, or .out-of-scope. |
| implement / implement-spec | Follow docs/agents/issue-tracker.md: shared branches, one validation phase, mandatory teaching material with every change, local commit, and brief delivery/retro. No automatic review, push, or PR. |
| code-review / pr | User-invoked only; removed from the default workflow and from implementation/router handoffs. |
| tdd | Optional method inside the shared validation phase; behavior tests at agreed production interfaces, with existing tests used for simple refactoring. |
| teach | On-demand lessons in docs/课程/自瞄链路; teaching documents are mandatory for every change even when teach is not invoked. |
| retro | Brief reflection accompanies delivery; a formal retrospective is reserved for major failures, phase completion, or an explicit request. |
| scaffold-exercises | Uses host C/C++, CMake/CTest, and local link checks instead of TypeScript and pnpm. |

The writing skills remain interactive for substantial co-written articles. Routine task learning notes are drafted from project evidence and trusted sources.
