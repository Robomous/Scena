# OpenSCENARIO XML versions and validation

Scena targets ASAM OpenSCENARIO XML **1.0 through 1.3**. This chapter states
what that means for a file you already have: which revisions load, what
happens to constructs a later revision deprecated, and what the file-level
validation pass checks before you run anything.

## Which revisions load

| Declared `revMajor.revMinor` | Result |
|---|---|
| 1.0, 1.1, 1.2, 1.3 | Accepted |
| 1.4 and later 1.x | Rejected, `Status::UnsupportedFeature` |
| any other major | Rejected, `Status::ValidationError` |
| missing / not an integer | Rejected, `Status::ParseError` |

Both rejections cite `asam.net:xosc:1.0.0:xml.valid_schema`, and the detected
version is reported in `Document::version` either way.

## One set of semantics for the whole range

Within 1.0–1.3 the runtime does **not** branch on the revision. Version 1.3
corrected the calculation specifications of the `Position` sub-classes, and
§5 says those corrected semantics are the ones an implementation should
apply to every version — so Scena runs one set of position and orientation
semantics for the whole range.

The practical consequence is testable, and it is tested: the same scenario
written for 1.0, 1.1, 1.2 and 1.3 loads to an equivalent IR and simulates
**bit-identically**.

## Migration table

Every construct below is **accepted and executed**. What changes with the
declared revision is whether Scena tells you a successor exists: a document
written before the successor existed is not doing anything wrong, so there
is nothing to report.

| Construct | Successor | Reported from | Mapping |
|---|---|---|---|
| `ActivateControllerAction` directly under `PrivateAction` | `ControllerAction/ActivateControllerAction` | 1.1 | Same action, same lateral/longitudinal flags |
| `GeoPosition` `latitude`/`longitude`/`height` | `latitudeDeg`/`longitudeDeg`/`altitude` | 1.1 | Same fields |
| `alongRoute` on a distance condition | `coordinateSystem` + `relativeDistanceType` | 1.1 | Stored as given; the newer attributes win when both appear |
| `ParameterSetAction` / `ParameterModifyAction` | `VariableSetAction` / `VariableModifyAction` | 1.2 | Executed against a runtime parameter overlay so a 1.0/1.1 file's `ParameterCondition` observes the change |
| `ReachPositionCondition` | `DistanceCondition` | 1.2 | Executed as a 2D tolerance circle |
| `curvatureDot` on a clothoid | `curvaturePrime` | 1.4 | Same field |
| Event priority `overwrite` | `override` | 1.3 | One IR value: the standard's descriptions of the two literals are word for word identical |
| `Rule` operators `greaterOrEqual`, `lessOrEqual`, `notEqualTo` | — | before 1.2 | Read with their documented meaning; the introduction version cannot be confirmed from the reference text, so this warns rather than rejects |

Deprecations are reported as `Status::DeprecatedFeature` warnings. A warning
never fails a load — that is the kernel's severity contract, and it is what
"accepted, deprecated" in the coverage matrix means.

## File-level validation

Some defects are only visible once the whole document has been read. The
element readers check what is in front of them (an attribute's type, a
required child, a choice with one alternative); this pass checks the rest,
and it runs as part of every load — a caller cannot forget it.

**Referential integrity**

| Check | Rule |
|---|---|
| Entity references in actions, actors and triggering entities name a declared entity or selection | `reference_control.references_to_scenario_object` |
| A `StoryboardElementStateCondition` names an element that exists **and is of the type it claims** | `reference_control.resolvable_storyboard_element_ref` |
| Variable references name a declared variable | `reference_control.resolvable_variable_reference` |
| Traffic signal controller references name a declared controller | `reference_control.traffic_signal_controller_references` |
| Parameter references resolve in scope | `parameters.parameter_declaration_parameter_scope` |
| Catalog references resolve | `reference_control.catalog_reference_resolvability` |

**Naming** — duplicate names among siblings (two entities, stories, acts,
maneuver groups, maneuvers or events with the same name at the same level)
cite `naming.unique_element_names_on_same_level`.

**Unused declarations** — a declared parameter or variable that nothing
references is reported as a **warning**. It is not a defect: §9.1 notes that
global parameters may be overwritten by external tools, so a knob nothing
references inside the file can be entirely deliberate. It is a warning
because the far more common cause is a typo in the reference.

### Checking without running

```cpp
scena::xml::Document document;
scena::DiagnosticSink sink;
const scena::Status status = scena::xml::validate_file("cut_in.xosc", document, sink);
```

`validate_file` / `validate_string` are `load_file` / `load_string` under a
name that says what you are asking — useful for a linter or a CI check that
should not execute anything. The validation is part of loading either way.

## See also

- [Loading scenarios](loading-scenarios.md) — the loader API and diagnostics.
- [Coverage matrix](../roadmap/coverage/osc-xml-coverage.md) — the normative
  scope declaration, including every "accepted, deprecated" row.
