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

"""Checking OpenSCENARIO DSL from Python (p7-s5).

Checking is not running: DSL execution is P8. A source that checks clean here is
one the frontend understood, and these tests pin what the binding reports about
it — the status, the findings, and how far the checker got.
"""

import scena as scn

# `length` and `speed` come from the bundled standard library, so a source that
# uses them also proves the implicit import reached the check (§7.7.5.2).
CLEAN = """\
struct marker:
    x: length
    y: length

actor beacon:
    reach: length
    top_speed: speed
"""

UNKNOWN_TYPE = """\
struct marker:
    x: no_such_type
"""


def test_a_clean_source_checks_ok() -> None:
    result = scn.check_dsl_string(CLEAN)
    assert result.status == scn.Status.Ok
    assert result.diagnostics == []
    assert bool(result) is True


def test_the_check_counts_what_it_covered() -> None:
    result = scn.check_dsl_string(CLEAN)
    # The source itself plus the standard library it was given implicitly.
    assert result.file_count > 1
    # Every type the source declares plus everything the library brought.
    assert result.type_count > 2


def test_an_unknown_type_is_an_error_that_cites_its_section() -> None:
    result = scn.check_dsl_string(UNKNOWN_TYPE, "marker.osc")
    assert result.status != scn.Status.Ok
    assert bool(result) is False
    assert len(result.diagnostics) == 1
    diagnostic = result.diagnostics[0]
    assert diagnostic.severity == scn.Severity.Error
    assert "no_such_type" in diagnostic.message
    # The DSL standard defines no `asam.net:` rule ids, so a DSL diagnostic
    # cites its section in the message and leaves rule_id empty.
    assert "§" in diagnostic.message
    assert diagnostic.rule_id == ""


def test_a_diagnostic_names_the_file_it_came_from() -> None:
    # A checked program spans every file its root imported, so a line number
    # alone locates nothing.
    result = scn.check_dsl_string(UNKNOWN_TYPE, "marker.osc")
    diagnostic = result.diagnostics[0]
    assert diagnostic.location.file == "marker.osc"
    assert diagnostic.location.line == 2


def test_the_origin_defaults_when_none_is_given() -> None:
    result = scn.check_dsl_string(UNKNOWN_TYPE)
    assert result.diagnostics[0].location.file == "<string>"


def test_without_the_standard_library_its_types_are_gone() -> None:
    # Turning the library off is what makes a file checkable in isolation —
    # and `length` stops naming anything.
    result = scn.check_dsl_string(CLEAN, implicit_standard_library=False)
    assert result.status != scn.Status.Ok
    assert result.file_count == 1
    assert any("length" in diagnostic.message for diagnostic in result.diagnostics)


def test_a_file_is_checked_from_disk(tmp_path) -> None:
    source = tmp_path / "marker.osc"
    source.write_text(CLEAN, encoding="utf-8")
    result = scn.check_dsl_file(source)
    assert result.status == scn.Status.Ok
    assert result.diagnostics == []


def test_an_unreadable_path_is_host_misuse(tmp_path) -> None:
    # InvalidArgument is the Status model's host-misuse code: the path was not
    # something we could read at all, which is a different failure from a defect
    # in the content.
    result = scn.check_dsl_file(tmp_path / "absent.osc")
    assert result.status == scn.Status.InvalidArgument
    assert len(result.diagnostics) == 1
    assert result.file_count == 0


def test_an_import_is_followed_through_a_search_path(tmp_path) -> None:
    library = tmp_path / "lib"
    (library / "shapes").mkdir(parents=True)
    (library / "shapes" / "basic.osc").write_text(
        "struct marker:\n    x: length\n", encoding="utf-8"
    )
    source = tmp_path / "top.osc"
    source.write_text("import shapes.basic\n\nstruct pair:\n    a: marker\n", encoding="utf-8")

    without = scn.check_dsl_file(source)
    assert without.status != scn.Status.Ok

    result = scn.check_dsl_file(source, search_paths=[library])
    assert result.status == scn.Status.Ok, [d.message for d in result.diagnostics]
    # The root, the imported module, and the standard library.
    assert result.file_count == without.file_count + 1


def test_the_repr_says_what_the_check_found() -> None:
    text = repr(scn.check_dsl_string(CLEAN))
    assert "DslCheck(" in text
    assert "status=" in text
    assert "file_count=" in text
