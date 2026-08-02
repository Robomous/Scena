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

"""Audits the C++ / C / Python parity of the engine and frontend surfaces.

Scena ships three ways to drive the same engine, and the pillar's exit criterion
is that none of them silently lags the others. This script extracts the public
`scena::Engine` methods from the C++ header and checks each one against:

- the C ABI, by looking for its `scn_engine_*` entry point in `capi/capi.h`;
- the Python bindings, by looking for its `.def(...)` / `.def_prop_ro(...)` in
  `python/src/bindings.cpp`.

It then does the same for the **frontend entry points** — the free functions a
host calls to get a scenario in, from `scena::xml` and `scena::dsl`. They are
not `Engine` methods, but they are just as much part of the surface a binding
can silently lag: p7-s5 added DSL checking to all three, and nothing would have
noticed if it had reached only two.

A method or entry point that is absent from a binding must appear in the
matching exclusion table below with a reason. Anything else is a **gap** and the
script exits non-zero — that is what makes the audit a test rather than a report
(`python/tests/test_parity.py` runs it).

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
XML_LOADER_HEADER = REPO / "frontends" / "xml" / "include" / "scena" / "xml" / "loader.h"
DSL_LOAD_HEADER = REPO / "frontends" / "dsl" / "include" / "scena" / "dsl" / "load.h"
DSL_LOWER_HEADER = REPO / "frontends" / "dsl" / "include" / "scena" / "dsl" / "lower.h"

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

#: The frontend headers whose namespace-scope entry points are audited, and the
#: namespace each one lives in (used only to name the row).
FRONTEND_HEADERS: dict[str, tuple[Path, ...]] = {
    "xml": (XML_LOADER_HEADER,),
    "dsl": (DSL_LOAD_HEADER, DSL_LOWER_HEADER),
}

#: `<namespace>::<function>` -> the C entry point that implements it. The names
#: differ more than the engine's do, because C has to say which frontend.
FRONTEND_C_ALIASES: dict[str, str] = {
    "xml::load_file": "scn_engine_load_xml_file",
    "xml::load_string": "scn_engine_load_xml_string",
    "dsl::check_file": "scn_check_dsl_file",
    "dsl::check_source": "scn_check_dsl_string",
}

#: `<namespace>::<function>` -> the Python name.
FRONTEND_PY_ALIASES: dict[str, str] = {
    "xml::load_file": "load_file",
    "xml::load_string": "load_string",
    "dsl::check_file": "check_dsl_file",
    "dsl::check_source": "check_dsl_string",
}

#: Frontend entry points that deliberately do not reach one or both bindings.
FRONTEND_EXCLUSIONS: dict[str, str] = {
    # Validation without a scenario is a C++ diagnostic-only path; the bindings
    # expose loading, which validates on the way through.
    "xml::validate_file": "loading validates; a validate-only pass has no binding consumer",
    "xml::validate_string": "loading validates; a validate-only pass has no binding consumer",
    # load_* stops after parsing and import resolution and hands back ASTs that
    # only C++ can walk. check_* is the same work plus resolution, and is what
    # a host actually wants.
    "dsl::load_file": "the lower half of check_file; its result is an AST no binding can carry",
    "dsl::load_source": "the lower half of check_source; same reason",
    # Lowering produces an ir::Scenario, which no binding carries today: the C
    # ABI and Python reach a scenario through the loaders, which build and
    # initialize an engine in one call. The DSL execution surface follows the
    # same shape in p8-s4 (#47), where the two frontends are compared directly.
    "dsl::lower": "the IR it produces has no binding carrier yet; p8-s4 (#47)",
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


def frontend_entry_points(text: str) -> list[str]:
    """Namespace-scope `Status <name>(` declarations in a frontend header.

    Same deliberately textual extraction as the engine side: a member function
    is indented, so anchoring at column 0 is what separates an entry point from
    a method on a class the header also declares.
    """
    names: list[str] = []
    for match in re.finditer(r"^(?:\[\[nodiscard\]\]\s+)?Status\s+(\w+)\s*\(", text, re.M):
        name = match.group(1)
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

    for name_space, headers in FRONTEND_HEADERS.items():
        functions: list[str] = []
        for header in headers:
            for function in frontend_entry_points(header.read_text(encoding="utf-8")):
                if function not in functions:
                    functions.append(function)
        for function in functions:
            qualified = f"{name_space}::{function}"
            c_symbol = FRONTEND_C_ALIASES.get(qualified, "")
            py_name = FRONTEND_PY_ALIASES.get(qualified, "")
            rows.append(
                Row(
                    method=qualified,
                    in_c=bool(c_symbol) and c_symbol in capi_text,
                    in_python=bool(py_name) and f'"{py_name}"' in bindings_text,
                    excluded=FRONTEND_EXCLUSIONS.get(qualified),
                )
            )

    gaps = [row for row in rows if not row.ok]

    if args.markdown:
        print("| `scena::Engine` method or frontend entry point | C ABI | Python | note |")
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

    print(f"\n{len(rows)} entry points, {len(gaps)} gap(s)", file=sys.stderr)
    for row in gaps:
        # A frontend row already carries its namespace; an Engine method does not.
        qualified = row.method if "::" in row.method else f"Engine::{row.method}"
        print(
            f"GAP: {qualified} is missing from "
            f"{'the C ABI' if not row.in_c else ''}"
            f"{' and ' if not row.in_c and not row.in_python else ''}"
            f"{'the Python bindings' if not row.in_python else ''}"
            " — bind it, or add it to EXCLUSIONS with a reason.",
            file=sys.stderr,
        )
    return 1 if gaps else 0


if __name__ == "__main__":
    sys.exit(main())
