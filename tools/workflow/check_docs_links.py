#!/usr/bin/env python3
"""检查仓库文档里的相对链接是否指向存在的文件。

默认扫描根目录说明、`.Doc/` 和 `docs/` 下的 Markdown。外部网址、页内锚点、
代码块和行内代码不参与检查。指向 `build/` 的链接只给出提醒：该目录被 Git
忽略，适合放原始日志，不适合作为唯一的验证证据。

用法：
    python3 tools/workflow/check_docs_links.py
    python3 tools/workflow/check_docs_links.py --deny-build-references
    python3 tools/workflow/check_docs_links.py --root . --json

退出状态：存在缺失文件时返回 1；使用 `--deny-build-references` 时，出现
`build/` 链接也返回 1。读取失败会把原因写到标准错误并返回 1。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import unquote

# 只匹配行内链接和图片；链接目标可以用尖括号包住含空格路径。
LINK = re.compile(r"!?\[[^\]]*\]\(\s*(<[^>\n]+>|[^)\s]+)(?:\s+[\"'][^)]*[\"'])?\s*\)")
FENCE = re.compile(r"```.*?```", re.DOTALL)
INLINE_CODE = re.compile(r"`[^`\n]*`")
SCHEME = re.compile(r"^[a-zA-Z][a-zA-Z0-9+.-]*:")
BUILD_PATH = re.compile(r"(^|/)build(/|$)")


@dataclass
class Result:
    """保存一次检查的文件数、链接数和问题。"""

    files_checked: int = 0
    links_checked: int = 0
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def markdown_files(root: Path) -> list[Path]:
    """列出根说明、`.Doc` 和 `docs` 下要检查的 Markdown 文件。"""
    candidates = [root / "README.md", root / "AGENTS.md", root / "GLOSSARY.md"]
    candidates.extend(sorted((root / ".Doc").glob("*.md")))
    candidates.extend(sorted((root / "docs").rglob("*.md")))
    return [path for path in candidates if path.is_file()]


def strip_code(text: str) -> str:
    """去掉围栏代码和行内代码，避免把示例里的链接当成坏链接。"""
    return INLINE_CODE.sub("", FENCE.sub("", text))


def iter_links(text: str) -> list[tuple[int, str]]:
    """返回链接所在的 1 基行号和原始目标；搜索前先去掉代码片段。"""
    cleaned = strip_code(text)
    found: list[tuple[int, str]] = []
    for match in LINK.finditer(cleaned):
        target = match.group(1).strip()
        if target.startswith("<") and target.endswith(">"):
            target = target[1:-1]
        line = cleaned.count("\n", 0, match.start()) + 1
        found.append((line, target))
    return found


def load_allowlist(path: Path) -> list[dict[str, str]]:
    """读取允许缺失的仓库路径前缀；文件不存在时返回空表。"""
    if not path.is_file():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    entries = data.get("prefixes", [])
    if not isinstance(entries, list):
        raise ValueError("allowlist 的 prefixes 必须是列表")
    cleaned: list[dict[str, str]] = []
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("prefix"), str):
            raise ValueError("allowlist 每项需要 prefix 字符串")
        cleaned.append({"prefix": entry["prefix"], "reason": str(entry.get("reason", ""))})
    return cleaned


def allowed_reason(relative_path: str, allowlist: list[dict[str, str]]) -> str | None:
    """返回命中的允许前缀说明，未命中时返回 None。"""
    for entry in allowlist:
        if relative_path.startswith(entry["prefix"]):
            return entry["reason"]
    return None


class PathCache:
    """按目录缓存文件名，避免为每条链接单独查询文件系统。"""

    def __init__(self) -> None:
        self._entries: dict[Path, set[str]] = {}

    def exists(self, path: Path) -> bool:
        """判断路径是否存在；同名目录和文件都算存在。"""
        parent = path.parent
        if parent not in self._entries:
            try:
                self._entries[parent] = set(os.listdir(parent))
            except OSError:
                self._entries[parent] = set()
        return path.name in self._entries[parent]


def check_file(path: Path, root: Path, result: Result,
               allowlist: list[dict[str, str]], cache: PathCache) -> None:
    """检查单个文件中的相对链接，并把问题追加到结果。"""
    text = path.read_text(encoding="utf-8")
    relative_file = path.relative_to(root).as_posix()
    for line, target in iter_links(text):
        if not target or target.startswith("#") or SCHEME.match(target) or target.startswith("//"):
            continue
        raw_target = target.split("#", 1)[0].split("?", 1)[0]
        if not raw_target:
            continue
        result.links_checked += 1
        decoded = unquote(raw_target)
        # 绝对路径和仓库外路径不参与相对链接检查，避免把本机配置写进仓库。
        if decoded.startswith("/"):
            result.warnings.append(f"{relative_file}:{line}: 绝对路径链接 {target}")
            continue
        # 用路径文本比较代替逐条 resolve；在 WSL 的 /mnt/d 上 resolve 会为每段路径查询文件系统。
        candidate = Path(os.path.normpath(str(path.parent / decoded)))
        try:
            inside = candidate.relative_to(Path(os.path.normpath(str(root))))
        except ValueError:
            result.warnings.append(f"{relative_file}:{line}: 指向仓库外的链接 {target}")
            continue
        inside_name = inside.as_posix()
        reason = allowed_reason(inside_name, allowlist)
        if reason is not None:
            result.warnings.append(
                f"{relative_file}:{line}: 引用仓库外资料 {target}（{reason}）"
            )
            continue
        if BUILD_PATH.search(inside_name):
            result.warnings.append(
                f"{relative_file}:{line}: 链接指向被忽略的 build/：{target}"
            )
            continue
        if not cache.exists(candidate):
            result.errors.append(f"{relative_file}:{line}: 链接目标不存在：{target}")


def check(root: Path, allowlist: list[dict[str, str]] | None = None) -> Result:
    """检查整个仓库的文档链接。"""
    result = Result()
    rules = allowlist or []
    cache = PathCache()
    for path in markdown_files(root):
        result.files_checked += 1
        check_file(path, root, result, rules, cache)
    return result


def print_report(result: Result, max_print: int, deny_build: bool) -> None:
    """打印摘要和问题列表；问题太多时只显示前若干条。"""
    print(
        f"文档 {result.files_checked} 个，链接 {result.links_checked} 条；"
        f"缺失 {len(result.errors)}，提醒 {len(result.warnings)}"
    )
    labels = (("错误", result.errors), ("提醒", result.warnings))
    shown = 0
    for label, items in labels:
        if not items:
            continue
        print(f"\n{label}：")
        for item in items:
            if shown >= max_print:
                print(f"……其余 {len(result.errors) + len(result.warnings) - shown} 条未显示")
                return
            print(f"  {item}")
            shown += 1
    if deny_build and any("build/" in item for item in result.warnings):
        print("\n已启用 --deny-build-references：build/ 链接视为错误。", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="检查文档相对链接")
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--deny-build-references", action="store_true",
                        help="把指向 build/ 的链接也视为错误")
    parser.add_argument("--json", action="store_true", help="输出 JSON 摘要")
    parser.add_argument("--max-print", type=int, default=50, help="最多显示多少条问题")
    parser.add_argument("--allowlist", type=Path,
                        default=Path(__file__).resolve().with_name("docs_links_allowlist.json"),
                        help="允许缺失的仓库外资料前缀")
    parser.add_argument("--no-allowlist", action="store_true", help="不读取允许缺失清单")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    try:
        allowlist = [] if args.no_allowlist else load_allowlist(args.allowlist)
        result = check(root, allowlist)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"读取文档失败：{error}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps({
            "files_checked": result.files_checked,
            "links_checked": result.links_checked,
            "errors": result.errors,
            "warnings": result.warnings,
        }, ensure_ascii=False, indent=2))
    else:
        print_report(result, args.max_print, args.deny_build_references)

    if result.errors:
        return 1
    if args.deny_build_references and any("build/" in item for item in result.warnings):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
