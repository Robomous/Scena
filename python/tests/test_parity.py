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

"""The C++ / C / Python parity audit, as a test (p6-s3).

`scripts/parity_audit.py` exits non-zero when a public `scena::Engine` method or
a frontend entry point is missing from the C ABI or the Python bindings without
a recorded reason. Running it here is what turns the pillar's "parity audit
gap-free" exit criterion into something CI enforces rather than something a
human remembers to check.
"""

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
AUDIT = REPO / "scripts" / "parity_audit.py"


def test_the_audit_script_exists() -> None:
    assert AUDIT.is_file(), AUDIT


def test_the_parity_audit_reports_no_gaps() -> None:
    result = subprocess.run(
        [sys.executable, str(AUDIT)], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, (
        "the C++/C/Python parity audit found gaps:\n" + result.stderr + "\n" + result.stdout
    )
    assert "gap(s)" in result.stderr


def test_the_audit_emits_a_markdown_table() -> None:
    # The table is what goes into a release note or a docs page, so the
    # formatting matters enough to pin.
    result = subprocess.run(
        [sys.executable, str(AUDIT), "--markdown"], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    assert lines[0].startswith("| `scena::Engine` method or frontend entry point |")
    assert lines[1].startswith("|---|")
    assert any("`init`" in line for line in lines)
    assert any("`step`" in line for line in lines)
    # The frontend entry points share the table: p7-s5's DSL check is bound in
    # all three surfaces, and this is where that stays visible.
    assert any("`dsl::check_file`" in line for line in lines)
    assert any("`xml::load_file`" in line for line in lines)


def test_the_audit_notices_an_unbound_method(tmp_path) -> None:
    """A gap really is detected — the audit is not vacuously green.

    Runs the audit against a doctored copy of engine.h with a method nothing
    binds, and expects it to fail. Without this, a broken extractor would report
    "0 gaps" forever.
    """
    import shutil

    sandbox = tmp_path / "repo"
    for part in (
        "core/include/scena",
        "capi/include/scena",
        "python/src",
        "scripts",
        "frontends/xml/include/scena/xml",
        "frontends/dsl/include/scena/dsl",
    ):
        (sandbox / part).mkdir(parents=True, exist_ok=True)
    shutil.copy(REPO / "scripts" / "parity_audit.py", sandbox / "scripts" / "parity_audit.py")
    shutil.copy(REPO / "capi" / "include" / "scena" / "capi.h",
                sandbox / "capi/include/scena/capi.h")
    shutil.copy(REPO / "python" / "src" / "bindings.cpp", sandbox / "python/src/bindings.cpp")
    # The audit reads the frontend headers too; without them it would fail on a
    # missing file and this test would pass for the wrong reason.
    shutil.copy(REPO / "frontends/xml/include/scena/xml/loader.h",
                sandbox / "frontends/xml/include/scena/xml/loader.h")
    shutil.copy(REPO / "frontends/dsl/include/scena/dsl/load.h",
                sandbox / "frontends/dsl/include/scena/dsl/load.h")
    shutil.copy(REPO / "frontends/dsl/include/scena/dsl/lower.h",
                sandbox / "frontends/dsl/include/scena/dsl/lower.h")

    header = (REPO / "core" / "include" / "scena" / "engine.h").read_text(encoding="utf-8")
    doctored = header.replace(
        "    Status init(ir::Scenario scenario);",
        "    Status init(ir::Scenario scenario);\n"
        "    [[nodiscard]] int totally_unbound_thing() const;",
        1,
    )
    assert doctored != header
    (sandbox / "core/include/scena/engine.h").write_text(doctored, encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(sandbox / "scripts" / "parity_audit.py")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "totally_unbound_thing" in result.stderr
