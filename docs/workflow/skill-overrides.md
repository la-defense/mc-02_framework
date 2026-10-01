# Project-specific skill adaptations

The installed skills are pinned to mattpocock/skills in skills-lock.json. These local changes adapt tracker paths, C/C++ exercises, and teaching triggers to this repository. Do not replace them during a skill update without reviewing the diff.

| Skill | Project adaptation |
|---|---|
| ask-matt | Routes task tracking through each repository TODO and includes the teaching / learning-note path. |
| setup-matt-pocock-skills | Reads the committed local tracker and domain configuration without asking already-settled setup questions. |
| to-spec / to-tickets / implement-spec | Specs and ticket details live under docs/workflow; active state remains in TODO; a feature uses one branch and PR. |
| triage / wayfinder | Uses TODO states and docs/workflow notes instead of GitHub issue labels, .scratch, or .out-of-scope. |
| implement / code-review | Commit before review; find the spec by task ID and review the committed branch against its merge base. |
| teach | Enables the agreed just-in-time trigger and uses docs/课程/自瞄链路 as the teaching workspace. |
| scaffold-exercises | Uses host C/C++, CMake/CTest, and local link checks instead of TypeScript and pnpm. |

The writing skills remain interactive for substantial co-written articles. Routine task learning notes are drafted from project evidence and trusted sources.
