#!/usr/bin/env python3
"""Check course exercise structure and offline links; optionally build and run CTest."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[2]
COURSE = ROOT / "docs" / "课程" / "自瞄链路"
EXERCISES = COURSE / "exercises"
VARIANTS = ("problem", "solution", "explainer")
MARKDOWN_LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)]+)\)")
HTML_LINK = re.compile(r"(?:href|src)\s*=\s*['\"]([^'\"]+)['\"]", re.IGNORECASE)


def check_structure() -> list[str]:
    errors: list[str] = []
    if not (COURSE / "MISSION.md").is_file():
        errors.append("missing course MISSION.md")
    exercise_dirs = sorted(path for path in EXERCISES.iterdir() if path.is_dir())
    if not exercise_dirs:
        errors.append("no exercise directories found")
    for exercise in exercise_dirs:
        for variant in VARIANTS:
            readme = exercise / variant / "readme.md"
            if not readme.is_file() or not readme.read_text(encoding="utf-8").strip():
                errors.append(f"{readme.relative_to(ROOT)}: missing or empty readme.md")
        for source in (exercise / "problem").glob("*.[ch]pp"):
            if "TODO" not in source.read_text(encoding="utf-8"):
                errors.append(f"{source.relative_to(ROOT)}: problem starter needs a TODO marker")
    return errors


def check_links() -> list[str]:
    errors: list[str] = []
    learning_notes = ROOT / "docs" / "学习笔记"
    documents = (
        *COURSE.rglob("*.md"),
        *COURSE.rglob("*.html"),
        learning_notes / "README.md",
        learning_notes / "20-SP固定帧与CRC校验.md",
        learning_notes / "21-从TODO到可重复验证的工程闭环.md",
        learning_notes / "22-失联安全与有界等待.md",
    )
    for document in sorted(documents):
        content = document.read_text(encoding="utf-8")
        targets = [match.group(1) for match in MARKDOWN_LINK.finditer(content)]
        targets.extend(match.group(1) for match in HTML_LINK.finditer(content))
        for raw_target in targets:
            target = raw_target.strip().split()[0].strip("<>")
            parsed = urlsplit(target)
            if parsed.scheme or target.startswith("//"):
                continue
            relative_path = unquote(parsed.path)
            if not relative_path:
                continue
            resolved = (document.parent / relative_path).resolve()
            if not resolved.exists():
                errors.append(
                    f"{document.relative_to(ROOT)}: broken local link {raw_target!r}"
                )
    return errors


def build_and_test(generator: str | None) -> int:
    with tempfile.TemporaryDirectory(prefix="mc02-learning-exercises-") as build_dir:
        env = os.environ.copy()
        selected_generator = generator
        configured_generator = env.get("CMAKE_GENERATOR")
        generator_tools = {
            "Ninja": "ninja",
            "NMake Makefiles": "nmake",
            "NMake Makefiles JOM": "jom",
            "MinGW Makefiles": "mingw32-make",
            "Unix Makefiles": "make",
        }
        if selected_generator is None and configured_generator:
            tool = generator_tools.get(configured_generator)
            if tool is None or shutil.which(tool):
                selected_generator = configured_generator
            else:
                # A shell can inherit a stale generator from a developer prompt.
                env.pop("CMAKE_GENERATOR", None)
        if selected_generator is None and shutil.which("ninja"):
            selected_generator = "Ninja"

        configure = ["cmake", "-S", str(EXERCISES), "-B", build_dir]
        if selected_generator:
            configure.extend(["-G", selected_generator])
        subprocess.run(configure, check=True, env=env)
        subprocess.run(["cmake", "--build", build_dir, "--parallel"], check=True, env=env)
        subprocess.run(
            ["ctest", "--test-dir", build_dir, "--output-on-failure"], check=True, env=env
        )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="configure, build, and run CTest")
    parser.add_argument("--generator", help="optional CMake generator override")
    args = parser.parse_args()

    errors = check_structure() + check_links()
    if errors:
        print("Exercise checks failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    exercise_count = sum(
        all((path / variant / "readme.md").is_file() for variant in VARIANTS)
        for path in EXERCISES.iterdir()
        if path.is_dir()
    )
    print(f"Exercise structure and offline links are valid ({exercise_count} exercises)")
    if args.build:
        return build_and_test(args.generator)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.returncode)
