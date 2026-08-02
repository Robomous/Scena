# `scena-check` — checking OpenSCENARIO DSL

`scena-check` loads an OpenSCENARIO DSL file, follows its imports, and reports
what the checker finds. It does not execute anything: DSL execution is a later
milestone, and a file that checks clean here is one the frontend understood, not
one that will necessarily run.

```
scena-check my_scenario.osc
```

```
my_scenario.osc: ok, 412 types across 3 files
```

## Options

| Option | Meaning |
| --- | --- |
| `-I`, `--search-path <dir>` | A directory to resolve module imports against. Repeatable; searched in the order given. |
| `--no-standard-library` | Do not load the bundled `osc.standard` library. |
| `--strict` | Exit non-zero when there are warnings, not only errors. |
| `--quiet` | Do not print diagnostics. |
| `-h`, `--help` | Usage text. |

## Exit codes

| Code | Meaning |
| --- | --- |
| `0` | The file checked. There may still have been warnings — see `--strict`. |
| `2` | The command line was wrong. |
| `3` | The source did not check: it has errors, an import did not resolve, or `--strict` was given and there were warnings. |
| `4` | The input could not be read. |

`3` and `4` are deliberately different. Scena's status model separates a defect
in the content from the host handing the library something it cannot use, and
the exit codes follow that line: a file full of type errors is your scenario's
problem, an unreadable path is your invocation's. A build script can act on the
difference.

## Diagnostics

One diagnostic per line, in the same shape [`scena-run`](scena-run.md) prints, so
a script that parses one tool's output can parse the other's:

```
error: my_scenario.osc:14:9: unknown type 'vehicel' (§7.7.4.2)
warning: my_scenario.osc:31:5: this constraint needs a solver; v0.0.1 resolves fixed values only (§7.3.11, ADR-0004)
```

The file is always named, including for a diagnostic that came from an imported
file rather than from the one you passed — a program spans every file its root
imported, so a line number on its own would not locate anything.

Every diagnostic cites the section of the standard it comes from. Unlike the XML
frontend's, a DSL diagnostic carries no bracketed rule identifier: the DSL
standard defines no `asam.net:` rule ids, so the section reference in the message
is the citation.

Error recovery is contractual. A malformed declaration does not cost its
siblings, so one run reports as much as it can rather than stopping at the first
problem.

## Imports

Both of the reference forms in the standard work.

A **file reference** is resolved relative to the file that wrote it:

```
import "shared/common.osc"
```

A **module reference** maps `a.b.c` to `a/b/c.osc` and is looked for under each
`--search-path`, in the order you gave them:

```
import shared.common
```

```
scena-check main.osc -I ./lib -I ./vendor/lib
```

A file referenced twice is imported once, keyed on its canonical path. A diamond
therefore declares its shared types once, and an import cycle terminates instead
of erroring.

References beginning with `osc` are reserved by the standard and never reach the
search path; an unknown one is reported rather than looked up as a module of
yours.

## The standard library

The `osc.standard` library is bundled — it is not files on disk, so there is
nothing to install or point at. Its physical types are available without an
import, because they are what gives a literal like `30kph` a type at all; the
domain model needs an explicit import:

```
import osc.standard.all
namespace demo use std, stdtypes

scenario overtake:
    ego: vehicle
    target: lane
    keep(target.lane_type == lane_type!driving)
    keep(target.width == 3.5m)
```

`--no-standard-library` turns even the implicit part off, which is useful when
checking a file that is meant to stand alone.

## Checking from C and Python

The CLI is a thin consumer of the same entry point the bindings expose, so an
editor plugin or a build script can have the findings as objects instead of
parsing text. Checking needs no engine — it is a frontend service.

In Python, one call returns a `DslCheck`:

```python
import scena as scn

result = scn.check_dsl_file("overtake.osc", search_paths=["lib"])
if not result:
    for diagnostic in result.diagnostics:
        where = diagnostic.location
        print(f"{where.file}:{where.line}:{where.column}: {diagnostic.message}")
else:
    print(f"ok, {result.type_count} types across {result.file_count} files")
```

`DslCheck` carries `status`, `diagnostics`, `type_count` and `file_count`, and
is falsy unless the status is `Ok`. `check_dsl_string(source, origin)` checks a
source in memory; `origin` names it in diagnostics and anchors its relative
imports, and need not exist on disk. Both take `search_paths` and
`implicit_standard_library`, the two `LoadOptions` fields. The XML loaders
return `(status, scenario)` because the scenario is the payload; a check has no
payload — the findings are the result — so it returns one named object.

In C, the result is an opaque handle you destroy when done:

```c
scn_dsl_check_options options = {0};
options.implicit_standard_library = 1;   /* the zero struct turns it OFF */

scn_dsl_check* check = NULL;
scn_status status = scn_check_dsl_file("overtake.osc", &options, &check);

size_t count = 0;
scn_dsl_check_diagnostic_count(check, &count);
for (size_t i = 0; i < count; ++i) {
    scn_diagnostic diagnostic;
    scn_dsl_check_diagnostic_at(check, i, &diagnostic);
    fprintf(stderr, "%s:%d:%d: %s\n", diagnostic.file, diagnostic.line,
            diagnostic.column, diagnostic.message);
}
scn_dsl_check_destroy(check);
```

A failing check still produces a handle — that is exactly the case whose
diagnostics you want — so destroy it whatever the status. The strings a
diagnostic borrows stay valid until `scn_dsl_check_destroy`; unlike the engine's
diagnostics, nothing else invalidates them, because a check result never
changes. Only a NULL argument returns without a handle.

## What "checked clean" covers

The checker resolves names, types every expression, and validates constraints
and coverage declarations against the standard's rules. Constraints it cannot
resolve to fixed values are reported as warnings rather than errors — solving
them needs a constraint solver, which is out of scope for v0.0.1 — so a clean
exit means "well-formed and understood", not "solved".

Run with `--strict` in CI if you want those warnings to fail the build.
