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

"""Checking OpenSCENARIO DSL sources from Python (p7-s5).

The same work `scena-check` does on the command line, reachable from a build
script or an editor integration that wants the findings as objects rather than
as text.

Checking is not running: DSL execution is P8. A source that checks clean here is
one the frontend understood — its types resolve, its expressions type, its
imports were found — not one that will necessarily execute.

Three things this shows:

- a clean source, and the two counts that say how much the checker covered;
- a source with a defect, and what a diagnostic carries (severity, file, line,
  and a message citing the specification section — DSL diagnostics leave
  `rule_id` empty because the standard defines no `asam.net:` rule ids);
- an import resolved through a search path, the DSL's answer to "where do my
  other files live" (§7.7.5.1.2).
"""

import tempfile
from pathlib import Path

import scena as scn

# `length` and `speed` are physical types from the bundled standard library,
# which a check makes available without an import (§7.7.5.2) — that is what
# gives a literal like `30kph` a type at all.
BEACON = """\
struct marker:
    x: length
    y: length

actor beacon:
    reach: length
    top_speed: speed
"""

MISSPELLED = """\
struct marker:
    x: lenght
"""


def report(label: str, result: "scn.DslCheck") -> None:
    print(f"{label}: {result.status}")
    for diagnostic in result.diagnostics:
        where = diagnostic.location
        print(f"  {diagnostic.severity} {where.file}:{where.line}:{where.column}: "
              f"{diagnostic.message}")


def main() -> None:
    clean = scn.check_dsl_string(BEACON, "beacon.osc")
    report("beacon.osc", clean)
    print(f"  {clean.type_count} types across {clean.file_count} files")
    assert clean, "the example source should check clean"

    # A failing check is not an exception: the findings are the result, so the
    # call returns them the same way it returns a success.
    broken = scn.check_dsl_string(MISSPELLED, "misspelled.osc")
    report("misspelled.osc", broken)
    assert not broken, "a misspelled type should be an error"

    # Imports resolve against the search paths, in the order given: a reference
    # `shapes.basic` is looked up as <dir>/shapes/basic.osc.
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / "shapes").mkdir()
        (root / "shapes" / "basic.osc").write_text(BEACON, encoding="utf-8")
        top = root / "top.osc"
        top.write_text("import shapes.basic\n\nstruct pair:\n    a: marker\n", encoding="utf-8")

        missing = scn.check_dsl_file(top)
        report("top.osc (no search path)", missing)
        assert not missing, "the import cannot be resolved without a search path"

        found = scn.check_dsl_file(top, search_paths=[root])
        report("top.osc (with search path)", found)
        print(f"  {found.type_count} types across {found.file_count} files")
        assert found, "the import resolves once the search path is given"


if __name__ == "__main__":
    main()
