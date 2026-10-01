#!/usr/bin/env python3
"""Validate the local TODO item contract for either project repository."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ITEM = re.compile(
    r"^\s*- \[(?P<done>[ xX])\] \*\*(?P<id>[A-Z][A-Z0-9-]+)\*\* "
    r"\[(?P<category>bug|enhancement)\]"
    r"(?: \[(?P<state>needs-triage|needs-info|ready-for-agent|ready-for-human|wontfix)\])?"
    r" (?P<body>.+)$"
)


def validate(path: Path) -> list[str]:
    errors: list[str] = []
    seen: set[str] = set()
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        checkbox = re.match(r"^\s*- \[(?P<done>[ xX])\]", line)
        if not checkbox or checkbox.group("done").lower() == "x":
            continue
        match = ITEM.match(line)
        if not match:
            errors.append(f"{path}:{number}: checklist item lacks the required ID/category/state format")
            continue
        task_id = match.group("id")
        if task_id in seen:
            errors.append(f"{path}:{number}: duplicate task ID {task_id}")
        seen.add(task_id)
        state = match.group("state")
        body = match.group("body")
        if state in (None, "wontfix"):
            errors.append(f"{path}:{number}: open item needs an active state")
        if "验收：" not in body:
            errors.append(f"{path}:{number}: item lacks an acceptance condition")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("todo", nargs="?", default=".Doc/TODO.md")
    args = parser.parse_args()
    path = Path(args.todo)
    errors = validate(path)
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"{path}: TODO contract valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
