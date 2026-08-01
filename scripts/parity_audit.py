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

"""Audits the C++ / C / Python parity of the engine surface.

Scena ships three ways to drive the same engine, and the pillar's exit criterion
is that none of them silently lags the others. This script extracts the public
`scena::Engine` methods from the C++ header and checks each one against:

- the C ABI, by looking for its `scn_engine_*` entry point in `capi/capi.h`;
- the Python bindings, by looking for its `.def(...)` / `.def_prop_ro(...)` in
  `python/src/bindings.cpp`.

A method that is absent from a binding must appear in `EXCLUSIONS` below with a
reason. Anything else is a **gap** and the script exits non-zero — that is what
makes the audit a test rather than a report (`python/tests/test_parity.py` runs
it).

The extraction is deliberately textual. A real one would need a C++ parser and a
build of the bindings; the point here is to notice when someone adds a method and
forgets two of the three surfaces, and a grep notices that reliably.

Usage:
    python scripts/parity_audit.py            # print the table, exit 0/1
    python scripts/parity_audit.py --markdown # emit the table as Markdown
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ENGINE_HEADER = REPO / "core" / "include" / "scena" / "engine.h"
CAPI_HEADER = REPO / "capi" / "include" / "scena" / "capi.h"
BINDINGS = REPO / "python" / "src" / "bindings.cpp"

#: C++ Engine methods that deliberately do not reach one or both bindings, with
#: the reason. Keeping the reason here rather than in a comment is what lets the
#: audit distinguish "decided" from "forgotten".
EXCLUSIONS: dict[str, str] = {
    # Construction and destruction are not methods a binding mirrors one-for-one.
    "Engine": "constructor",
    "~Engine": "destructor",
    "operator=": "assignment is deleted",
    # C sees engine state through typed accessors; the IR objects themselves stay
    # in C++ and Python.
    "route_of": "returns an ir::Route*; C sees it through scn_engine_entity_route_* accessors",
    "assigned_controller_of": (
        "returns an ir::Controller*; C sees name and type through "
        "scn_engine_entity_controller_*"
    ),
    "environment": "returns an ir::Environment&; C has no struct for the whole environment",
    "prepend_diagnostics": "internal to the C ABI's load-and-init path; not a host operation",
    "set_gateway": "C installs callbacks with scn_engine_set_callbacks instead",
    "gateway": "C installs callbacks with scn_engine_set_callbacks instead",
}

#: C++ method name -> the C entry point that implements it, where a plain
#: `scn_engine_<name>` guess would miss.
C_ALIASES: dict[str, str] = {
    "state": "scn_engine_get_state",
    "variable": "scn_engine_get_variable",
    "user_defined_value": "scn_engine_get_user_defined_value",
    "date_time": "scn_engine_get_date_time",
    "default_lane_width": "scn_engine_get_default_lane_width",
    "time": "scn_engine_get_time",
    "storyboard_element_state": "scn_engine_element_state",
    "storyboard_element_transition": "scn_engine_element_transition",
    "entity_ids": "scn_engine_entity_id_at",
    "diagnostics": "scn_engine_diagnostic_at",
    "visibility_of": "scn_engine_entity_visibility",
    "controller_activation_of": "scn_engine_entity_controller_activation",
}

#: C++ method name -> the Python name, where they differ.
PY_ALIASES: dict[str, str] = {
    "time": "time",
    "initialized": "initialized",
    "default_lane_width": "default_lane_width",
}


@dataclass
class Row:
    method: str
    in_c: bool
    in_python: bool
    excluded: str | None

    @property
    def ok(self) -> bool:
        return self.excluded is not None or (self.in_c and self.in_python)


def public_engine_methods(text: str) -> list[str]:
    """Method names in the public section of class Engine."""
    start = text.index("class Engine {")
    end = text.index("\nprivate:", start)
    public = text[start:end]
    names: list[str] = []
    # `Status init(...)`, `[[nodiscard]] std::optional<EntityState> state(...)`,
    # `void clear_diagnostics() noexcept;` — take the identifier before '('.
    # \b before the capture stops the lazy prefix from eating into the
    # identifier ("explicit Engine(" must yield "Engine", not "ngine").
    for match in re.finditer(r"^\s{4}(?:\[\[nodiscard\]\]\s*)?[\w:<>,&*\s]+?\b(\w+)\s*\(",
                             public, re.M):
        name = match.group(1)
        if name in {"if", "for", "while", "return", "explicit"}:
            continue
        if name not in names:
            names.append(name)
    return names


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--markdown", action="store_true", help="emit the table as Markdown")
    args = parser.parse_args()

    engine_text = ENGINE_HEADER.read_text(encoding="utf-8")
    capi_text = CAPI_HEADER.read_text(encoding="utf-8")
    bindings_text = BINDINGS.read_text(encoding="utf-8")

    rows: list[Row] = []
    for method in public_engine_methods(engine_text):
        c_symbol = C_ALIASES.get(method, f"scn_engine_{method}")
        py_name = PY_ALIASES.get(method, method)
        rows.append(
            Row(
                method=method,
                in_c=c_symbol in capi_text,
                in_python=f'"{py_name}"' in bindings_text,
                excluded=EXCLUSIONS.get(method),
            )
        )

    gaps = [row for row in rows if not row.ok]

    if args.markdown:
        print("| `scena::Engine` method | C ABI | Python | note |")
        print("|---|---|---|---|")
        for row in rows:
            note = row.excluded or ""
            print(
                f"| `{row.method}` | {'yes' if row.in_c else 'no'} "
                f"| {'yes' if row.in_python else 'no'} | {note} |"
            )
    else:
        width = max(len(row.method) for row in rows)
        for row in rows:
            mark = "ok " if row.ok else "GAP"
            note = f"  ({row.excluded})" if row.excluded else ""
            print(
                f"{mark} {row.method.ljust(width)}  "
                f"C:{'y' if row.in_c else 'n'}  Py:{'y' if row.in_python else 'n'}{note}"
            )

    print(f"\n{len(rows)} methods, {len(gaps)} gap(s)", file=sys.stderr)
    for row in gaps:
        print(
            f"GAP: Engine::{row.method} is missing from "
            f"{'the C ABI' if not row.in_c else ''}"
            f"{' and ' if not row.in_c and not row.in_python else ''}"
            f"{'the Python bindings' if not row.in_python else ''}"
            " — bind it, or add it to EXCLUSIONS with a reason.",
            file=sys.stderr,
        )
    return 1 if gaps else 0


if __name__ == "__main__":
    sys.exit(main())
