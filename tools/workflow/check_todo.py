#!/usr/bin/env python3
"""校验仓库 TODO 契约，并可输出按状态和章节统计的摘要。

检查内容：
- 每个勾选条目都有稳定编号和 bug／enhancement 分类；
- 未完成条目必须有允许的分诊状态和 `验收：` 条件；
- 编号在整个文件内不能重复（已完成条目也参与查重）；
- 未完成条目正文过长、或只把证据指向被忽略的 `build/` 时给出提醒。

用法：
    python3 tools/workflow/check_todo.py .Doc/TODO.md
    python3 tools/workflow/check_todo.py .Doc/TODO.md --summary
    python3 tools/workflow/check_todo.py .Doc/TODO.md --strict

退出状态：格式错误返回 1；`--strict` 时提醒也返回 1。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

ITEM = re.compile(
    r"^\s*- \[(?P<done>[ xX])\] \*\*(?P<id>[A-Z][A-Z0-9-]+)\*\* "
    r"\[(?P<category>bug|enhancement)\]"
    r"(?: \[(?P<state>[a-z-]+)\])?"
    r" (?P<body>.+)$"
)
HEADING = re.compile(r"^##\s+(?P<title>.+?)\s*$")
CHECKBOX = re.compile(r"^\s*- \[(?P<done>[ xX])\]")
DONE_ID = re.compile(r"^\s*- \[[xX]\] (?:\[[a-z-]+\] )?\*\*(?P<id>[A-Z][A-Z0-9-]+)\*\*")
ALLOWED_STATES = ("needs-triage", "needs-info", "ready-for-agent", "ready-for-human", "wontfix")
BUILD_REFERENCE = re.compile(r"build/[A-Za-z0-9_.\-/]+")
DEFAULT_MAX_BODY = 1200


@dataclass
class Item:
    """TODO 中的一条任务记录。"""

    line: int
    task_id: str
    done: bool
    category: str
    state: str | None
    body: str
    section: str


@dataclass
class Report:
    """保存解析结果、错误和提醒。"""

    items: list[Item] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def open_items(self) -> list[Item]:
        """返回所有未完成条目。"""
        return [item for item in self.items if not item.done]

    def summary(self) -> dict[str, object]:
        """按状态、分类和最近章节统计未完成条目。"""
        open_items = self.open_items
        states = Counter(item.state for item in open_items)
        categories = Counter(item.category for item in open_items)
        sections = Counter(item.section for item in open_items)
        return {
            "open": len(open_items),
            "done": len(self.items) - len(open_items),
            "states": dict(states),
            "categories": dict(categories),
            "sections": dict(sections),
        }


def parse(path: Path) -> Report:
    """解析 TODO 文件；格式问题写入报告，不中断解析。"""
    report = Report()
    section = "(文件开头)"
    seen_ids: dict[str, int] = {}
    text = path.read_text(encoding="utf-8")
    for number, line in enumerate(text.splitlines(), 1):
        heading = HEADING.match(line)
        if heading:
            section = heading.group("title")
            continue
        if not CHECKBOX.match(line):
            continue
        match = ITEM.match(line)
        done = CHECKBOX.match(line).group("done").lower() == "x"
        if not match:
            # 历史清单里存在没有编号的已勾选项，也有分类写在编号之前的旧任务；
            # 已完成的记录只做编号查重，不按当前契约阻断。
            if done:
                loose = DONE_ID.match(line)
                if loose:
                    task_id = loose.group("id")
                    if task_id in seen_ids:
                        report.errors.append(
                            f"{path}:{number}: 任务编号重复 {task_id}"
                            f"（首次出现在第 {seen_ids[task_id]} 行）"
                        )
                    else:
                        seen_ids[task_id] = number
                continue
            report.errors.append(
                f"{path}:{number}: 清单项缺少编号、分类或状态格式"
            )
            continue
        task_id = match.group("id")
        if task_id in seen_ids:
            report.errors.append(
                f"{path}:{number}: 任务编号重复 {task_id}（首次出现在第 {seen_ids[task_id]} 行）"
            )
        else:
            seen_ids[task_id] = number
        report.items.append(Item(
            line=number,
            task_id=task_id,
            done=match.group("done").lower() == "x",
            category=match.group("category"),
            state=match.group("state"),
            body=match.group("body"),
            section=section,
        ))
    return report


def validate(report: Report, path: Path, max_body: int) -> None:
    """检查未完成条目的状态、验收条件、正文长度和证据位置。"""
    for item in report.open_items:
        if item.state is None or item.state == "wontfix":
            report.errors.append(f"{path}:{item.line}: 未完成项需要有效的分诊状态")
        elif item.state not in ALLOWED_STATES:
            report.errors.append(f"{path}:{item.line}: 未知状态 {item.state}")
        if "验收：" not in item.body:
            report.errors.append(f"{path}:{item.line}: 缺少 `验收：` 条件")
        if len(item.body) > max_body:
            report.warnings.append(
                f"{path}:{item.line}: 正文 {len(item.body)} 字，超过 {max_body}；"
                "详细证据宜移到票据文件"
            )
        build_refs = BUILD_REFERENCE.findall(item.body)
        if build_refs:
            report.warnings.append(
                f"{path}:{item.line}: 证据指向被忽略的 build/（{build_refs[0]}），"
                "请把结论或摘要提交到 docs/workflow/tickets/<ID>-evidence/"
            )


def print_summary(summary: dict[str, object]) -> None:
    """打印便于分诊的简短摘要。"""
    print(f"未完成 {summary['open']}，已完成 {summary['done']}")
    states = summary["states"]
    if states:
        print("状态：" + "，".join(f"{name} {count}" for name, count in sorted(states.items())))
    categories = summary["categories"]
    if categories:
        print("分类：" + "，".join(f"{name} {count}" for name, count in sorted(categories.items())))
    sections = summary["sections"]
    if sections:
        print("按章节：")
        for title, count in sorted(sections.items(), key=lambda pair: -pair[1]):
            print(f"  {count:>3}  {title}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="校验 TODO 契约")
    parser.add_argument("todo", nargs="?", type=Path, default=Path(".Doc/TODO.md"))
    parser.add_argument("--summary", action="store_true", help="输出状态和章节统计")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出摘要")
    parser.add_argument("--strict", action="store_true", help="把提醒也视为错误")
    parser.add_argument("--max-body", type=int, default=DEFAULT_MAX_BODY,
                        help="未完成条目正文的提醒长度")
    args = parser.parse_args(argv)
    path = args.todo
    try:
        report = parse(path)
    except OSError as error:
        print(f"读取 TODO 失败：{error}", file=sys.stderr)
        return 1
    validate(report, path, args.max_body)

    summary = report.summary()
    if args.json:
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    elif args.summary:
        print_summary(summary)
    if report.errors:
        print("\n".join(report.errors), file=sys.stderr)
    if report.warnings:
        print("\n".join(report.warnings), file=sys.stderr)
    if report.errors:
        return 1
    if args.strict and report.warnings:
        return 1
    if not args.json and not args.summary:
        print(f"{path}: TODO 契约有效（提醒 {len(report.warnings)} 条）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
