#!/usr/bin/env python3
# check_docs.py
# Why: the docs site must list every function a team calls; this fails when the code gains one
#      the reference or the at-a-glance page doesn't mention, or when the changelogs drift apart.
# Owns: the API coverage and changelog mirror checks. Standard library only. Run from anywhere.
"""Checks that the documentation covers the public API. Exit code 0 = OK, 1 = problems found."""

import re
import subprocess
import sys
from pathlib import Path

CHASSIS_HEADER = "cardLib/include/cardLib/chassis/chassis.hpp"
PRESETS_HEADER = "cardLib/include/cardLib/chassis/drive_presets.hpp"
CHASSIS_PAGE = "docs/reference/chassis.md"
CONFIG_PAGE = "docs/reference/config.md"
GLANCE_PAGE = "docs/reference/at-a-glance.md"
CHANGELOG = "CHANGELOG.md"
CHANGELOG_PAGE = "docs/project/changelog.md"

# Team-facing drive functions that live outside drive_presets.hpp but belong with the presets.
EXTRA_PRESET_FUNCTIONS = ["requireSpeedMatchWithin"]
MIRROR_SENTENCE = re.compile(r"^This file mirrors .*$")


def repo_root() -> Path:
    here = Path(__file__).resolve().parent
    try:
        out = subprocess.run(["git", "rev-parse", "--show-toplevel"], cwd=here,
                             capture_output=True, text=True, check=True)
        return Path(out.stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        return here.parent


def read(root: Path, relative: str) -> str:
    return (root / relative).read_text(encoding="utf-8")


def strip_comments(code: str) -> str:
    code = re.sub(r"/\*.*?\*/", " ", code, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", " ", code)


def top_level_calls(code: str) -> list:
    """Names directly before a '(' that sits outside any (), {} or <> nesting."""
    names = []
    paren = brace = 0
    for i, ch in enumerate(code):
        if ch == "(":
            if paren == 0 and brace == 0:
                match = re.search(r"([A-Za-z_]\w*)\s*$", code[:i])
                if match:
                    names.append(match.group(1))
            paren += 1
        elif ch == ")":
            paren -= 1
        elif ch == "{" and paren == 0:
            brace += 1
        elif ch == "}" and paren == 0:
            brace -= 1
    return names


def chassis_public_functions(header: str) -> list:
    code = strip_comments(header)
    start = re.search(r"\bclass\s+Chassis\s*\{", code)
    if not start:
        raise ValueError("class Chassis not found")
    body = code[start.end():]
    public = re.search(r"\bpublic\s*:(.*?)\b(?:private|protected)\s*:", body, flags=re.DOTALL)
    if not public:
        raise ValueError("no public: ... private: section in class Chassis")
    section = re.sub(r"\)(\s*const)?\s*\{[^{}]*\}", r")\1;", public.group(1))
    names = []
    for statement in section.split(";"):
        if "= delete" in statement or "operator" in statement:
            continue
        names.extend(top_level_calls(statement))
    return sorted(set(names))


def preset_functions(header: str) -> list:
    """Namespace-scope constexpr functions, skipping anything inside a struct body."""
    code = strip_comments(header)
    code = re.sub(r"\bstruct\s+\w+\s*\{.*?\n\};", " ", code, flags=re.DOTALL)
    found = re.findall(r"^constexpr\s+[\w:<>]+\s+(\w+)\s*\(", code, flags=re.MULTILINE)
    return sorted(set(found) | set(EXTRA_PRESET_FUNCTIONS))


def code_spans(markdown: str) -> str:
    return "\n".join(re.findall(r"`([^`\n]+)`", markdown))


def missing_from(names: list, markdown: str) -> list:
    spans = code_spans(markdown)
    return [n for n in names if not re.search(r"\b" + re.escape(n) + r"\(", spans)]


def glance_rows_without_links(markdown: str) -> list:
    rows = [line for line in markdown.splitlines() if line.startswith("| `")]
    return [row for row in rows if "](" not in row]


def changelog_problems(changelog: str, page: str) -> list:
    def normalise(text: str) -> list:
        lines = [line.rstrip() for line in text.replace("\r\n", "\n").split("\n")]
        return [line for line in lines if not MIRROR_SENTENCE.match(line)]

    a, b = normalise(changelog), normalise(page)
    while a and not a[-1]:
        a.pop()
    while b and not b[-1]:
        b.pop()
    if a == b:
        return []
    for number, (left, right) in enumerate(zip(a, b), start=1):
        if left != right:
            return [f"first difference near line {number}:\n    {CHANGELOG}: {left!r}\n"
                    f"    {CHANGELOG_PAGE}: {right!r}"]
    return [f"lengths differ: {len(a)} vs {len(b)} lines (ignoring the mirrors sentence)"]


def main() -> int:
    root = repo_root()
    problems = []

    chassis = chassis_public_functions(read(root, CHASSIS_HEADER))
    presets = preset_functions(read(root, PRESETS_HEADER))
    chassis_page = read(root, CHASSIS_PAGE)
    config_page = read(root, CONFIG_PAGE)
    glance = read(root, GLANCE_PAGE)
    print(f"Chassis public functions ({len(chassis)}): {', '.join(chassis)}")
    print(f"Preset functions ({len(presets)}): {', '.join(presets)}")

    checks = [
        (chassis, chassis_page, CHASSIS_PAGE),
        (chassis, glance, GLANCE_PAGE),
        (presets, config_page, CONFIG_PAGE),
        (presets, glance, GLANCE_PAGE),
    ]
    for names, page, path in checks:
        for name in missing_from(names, page):
            problems.append(f"{path}: no entry for {name}()")
    for row in glance_rows_without_links(glance):
        problems.append(f"{GLANCE_PAGE}: row has no link to its full entry: {row[:70]}")
    for problem in changelog_problems(read(root, CHANGELOG), read(root, CHANGELOG_PAGE)):
        problems.append(f"{CHANGELOG_PAGE} doesn't mirror {CHANGELOG}: {problem}")

    for problem in problems:
        print(f"PROBLEM: {problem}")
    print(f"check_docs: {len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
