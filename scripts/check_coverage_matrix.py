#!/usr/bin/env python3

# Copyright 2026 Robomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Docs-consistency check for the coverage matrices.

The matrices under docs/roadmap/coverage/ are the normative scope declaration
for v0.0.1, and they are only worth something if something enforces their own
rules. This script (the p5-s6 exit criterion names it) checks three:

  1. **Every sprint id a matrix names must exist as a sprint heading in
     docs/roadmap/roadmap.md.** This is the rule that actually rots: renaming
     or dropping a sprint silently leaves matrix rows pointing at nothing.
     Applies to every coverage matrix.

  2. **Every In row names the sprint that implements it, and a Post or Excl
     row names none** — the rule stated in the XML matrix's own preamble.
     Applies only to matrices with a single, unambiguous status column; the
     DSL matrix scores Check and Exec separately (a row may be Check=In and
     Exec=Post while legitimately naming the sprints that got it that far), so
     only rule 1 applies there.

  3. **Every In row's Tests cell either names the test suites that prove it
     or carries a planned/deferred marker with an owner** (a sprint id or a
     `#issue` reference), and every test file named anywhere in a Tests
     column exists under `core/tests/` or `python/tests/`. This is the rule
     the P5 pillar exit criterion states ("a named test" per In row) and the
     one that keeps the Status column honest: "In" declares scope, the Tests
     cell shows whether the row has actually been delivered. Applies to
     single-status matrices that have a Tests column.

Both matrices are parsed by their table headers rather than by column index,
so a table that grows a column does not silently stop being checked.

Usage:  python scripts/check_coverage_matrix.py

Prints every offending row with its file and line number, and exits non-zero
on failure.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ROADMAP = ROOT / "docs" / "roadmap" / "roadmap.md"
COVERAGE_DIR = ROOT / "docs" / "roadmap" / "coverage"

# "### p5-s6 — Global & infrastructure actions"
SPRINT_HEADING = re.compile(r"^#+\s+(p\d+-s\d+)\b")
# A sprint id anywhere in a cell.
SPRINT_ID = re.compile(r"\bp\d+-s\d+\b")
# A markdown table separator: |---|---|
SEPARATOR = re.compile(r"^\|[\s:|-]+\|$")
# A backticked test-suite file name in a Tests cell.
TEST_FILE = re.compile(r"`([A-Za-z0-9_]+_test\.cpp|test_[A-Za-z0-9_]+\.py)`")
# An issue reference in a planned/deferred marker.
ISSUE_REF = re.compile(r"#\d+")

STATUSES = {"In", "Post", "Excl"}
# Column headers that hold a coverage status, and the one that holds sprints.
STATUS_HEADERS = {"status", "check", "exec"}
SPRINT_HEADERS = {"sprint", "sprint(s)", "sprints"}
TESTS_HEADERS = {"tests", "test", "test suites"}
# Where a named test file must exist.
TEST_DIRS = (Path("core") / "tests", Path("python") / "tests")


def roadmap_sprints() -> set[str]:
    """Every sprint id that has a heading in the roadmap."""
    return {
        match.group(1)
        for match in (
            SPRINT_HEADING.match(line)
            for line in ROADMAP.read_text(encoding="utf-8").splitlines()
        )
        if match
    }


def cells_of(line: str) -> list[str]:
    return [cell.strip() for cell in line.strip().strip("|").split("|")]


def coverage_rows(path: Path):
    """Yields (line_number, element, statuses, sprint_cell, tests_cell,
    single_status), tests_cell being None where the table has no Tests column.

    Walks the file table by table: a header row followed by a separator opens a
    table, and the header names decide which columns carry a status and which
    carries the sprints. Tables without both are skipped, which is how the
    prose tables in these documents stay out of the check.
    """
    lines = path.read_text(encoding="utf-8").splitlines()
    status_columns: list[int] = []
    sprint_column: int | None = None
    tests_column: int | None = None

    for index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped.startswith("|"):
            # a table cannot span prose
            status_columns, sprint_column, tests_column = [], None, None
            continue
        if SEPARATOR.match(stripped):
            continue

        cells = cells_of(stripped)
        is_header = index + 1 < len(lines) and SEPARATOR.match(lines[index + 1].strip())
        if is_header:
            lowered = [cell.lower() for cell in cells]
            status_columns = [i for i, name in enumerate(lowered) if name in STATUS_HEADERS]
            sprint_column = next(
                (i for i, name in enumerate(lowered) if name in SPRINT_HEADERS), None
            )
            tests_column = next(
                (i for i, name in enumerate(lowered) if name in TESTS_HEADERS), None
            )
            continue

        if not status_columns or sprint_column is None:
            continue
        if sprint_column >= len(cells) or max(status_columns) >= len(cells):
            continue
        statuses = [cells[i] for i in status_columns]
        tests_cell = (
            cells[tests_column]
            if tests_column is not None and tests_column < len(cells)
            else None
        )
        yield (
            index + 1,
            cells[0],
            statuses,
            cells[sprint_column],
            tests_cell,
            len(status_columns) == 1,
        )


def main() -> int:
    if not ROADMAP.is_file():
        print(f"error: {ROADMAP} not found", file=sys.stderr)
        return 2
    sprints = roadmap_sprints()
    if not sprints:
        print(f"error: no sprint headings found in {ROADMAP}", file=sys.stderr)
        return 2

    problems: list[str] = []
    checked = 0
    for path in sorted(COVERAGE_DIR.glob("*.md")):
        relative = path.relative_to(ROOT)
        rows = coverage_rows(path)
        for number, element, statuses, sprint_cell, tests_cell, single_status in rows:
            checked += 1
            where = f"{relative}:{number}: row '{element}'"

            # Rule 1 — every sprint named must exist.
            named = SPRINT_ID.findall(sprint_cell)
            for sprint in named:
                if sprint not in sprints:
                    problems.append(
                        f"{where} names sprint '{sprint}', which has no heading in "
                        f"docs/roadmap/roadmap.md"
                    )

            # Rule 3 (file-existence half) — every named test file must exist,
            # whatever the row's status.
            for test_file in TEST_FILE.findall(tests_cell or ""):
                if not any((ROOT / d / test_file).is_file() for d in TEST_DIRS):
                    problems.append(
                        f"{where} names test file '{test_file}', which exists in "
                        f"none of {', '.join(str(d) for d in TEST_DIRS)}"
                    )

            # Rules 2 and 3 — only where the status is unambiguous.
            if not single_status:
                continue
            status = statuses[0]
            if status not in STATUSES:
                continue
            if status == "In" and not named:
                problems.append(
                    f"{where} is In-v0.0.1 but names no sprint "
                    f"(sprint cell is {sprint_cell!r})"
                )
            elif status != "In" and named:
                problems.append(
                    f"{where} is {status}, so it must not name a sprint "
                    f"(it names {', '.join(named)})"
                )

            # Rule 3 (delivery half) — an In row's Tests cell must name a
            # suite or carry an owned planned/deferred marker.
            if status != "In" or tests_cell is None:
                continue
            if TEST_FILE.search(tests_cell):
                continue
            lowered_tests = tests_cell.lower()
            if "planned" in lowered_tests or "deferred" in lowered_tests:
                if SPRINT_ID.search(tests_cell) or ISSUE_REF.search(tests_cell):
                    continue
                problems.append(
                    f"{where} is In-v0.0.1 with a planned/deferred Tests marker "
                    f"that names no owning sprint or issue "
                    f"(tests cell is {tests_cell!r})"
                )
                continue
            problems.append(
                f"{where} is In-v0.0.1 but its Tests cell names no test suite "
                f"and no planned/deferred owner (tests cell is {tests_cell!r})"
            )

    if problems:
        print(f"coverage matrix check FAILED ({len(problems)} problem(s)):", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1

    print(f"coverage matrix OK ({checked} rows checked against {len(sprints)} sprints)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
