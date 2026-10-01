---
name: scaffold-exercises
description: Create host-side C/C++ exercises for MC-02 and sp_vision_25 concepts, with problem, solution, and explainer material validated by local-link checks and CMake/CTest.
---

# Scaffold Exercises

Use the shared course at `docs/课程/自瞄链路/exercises/`. Exercises model the SP protocol, CDC state, vision safety, and camera/serial bridge with host data only. They never call HAL, OpenVINO, a camera SDK, or an actuator.

## Structure

Create one numbered exercise directory per concept:

```text
exercises/01-sp-frame-crc/
  problem/    # learner starter and task
  solution/   # runnable host implementation and tests
  explainer/  # concept explanation and links to the lesson/reference
```

Each directory has a nonempty `readme.md`. Problem starters may contain `TODO` markers; the solution must be complete. Use portable C++17, simulated inputs, no external test framework, and observable assertions.

## Workflow

1. Read `MISSION.md`, `RESOURCES.md`, the linked lesson, and the exercise tree.
2. Create or update one `problem / solution / explainer` set per requested concept.
3. Register runnable solution tests in the course `CMakeLists.txt` and add them to CTest.
4. Link each exercise to its lesson, reference page, and relevant code or test in the correct repository.
5. Run `python tools/workflow/check_exercises.py`; run it with `--build` to configure, build, and run CTest in a temporary build directory.
6. Confirm examples use simulated data and contain no hardware-control entry points.

## Done

The exercise is complete when all three readmes exist, local Markdown/HTML links resolve, its solution test passes through CTest, and the starter/explainer links point to the relevant lesson and project evidence.
