# ADR-0021: Structural lowering of an OpenSCENARIO XML document (p4-s2)

## Status

Accepted (p4-s2, #25).

## Context

p4-s1 gave the XML frontend a document layer: a parsed, version-checked
document reported through the kernel's diagnostics. p4-s2 fills it — entities,
the storyboard hierarchy, init actions, triggers, and the action and condition
payloads for everything the runtime has landed. That is the point where the
frontend stops being a reader and starts being a compiler into the Scenario
IR, so the mapping decisions it makes are the ones every later sprint builds
on.

## Decision

- **One reader module per element family.** `read_entities`,
  `read_geometry` (positions, trajectories, routes), `read_conditions`,
  `read_actions` and `read_storyboard`, all written against the shared
  `ReadContext` and the `read_common` attribute/enumeration/choice helpers.
  Every reader reports through the same context, so paths, positions and
  severities are uniform whichever element failed, and p4-s3..s5 extend the
  same surface rather than inventing their own.
- **The ScenarioObject name is the entity identity.** The standard
  references entities by that one name everywhere — actions, conditions,
  actors — so `Entity::id` and `Entity::name` are both that name. Splitting
  them would invent an identity the document does not have.
- **RoadNetwork file references go to the host, not the IR.** `LogicFile`
  and `SceneGraphFile` land on the frontend's `Document`, because the engine
  reaches roads only through the `IRoadQuery` gateway (ADR-0003): the
  embedder builds a backend from those paths. Putting a file path into the
  Scenario IR would give the kernel a dependency on a concrete road
  representation, which the layering forbids. The `TrafficSignalController`s
  declared inside `RoadNetwork` *are* scenario content and lower into the IR.
  Paths are kept verbatim and unresolved — resolving them relative to the
  scenario file is the host's decision.
- **A private action lowers once per actor.** §8.3.3.3 applies a
  ManeuverGroup's private actions to every actor; the IR action carries a
  single entity id, so the loader instantiates one action per actor, in actor
  order. An event whose group has no actor is a content defect, not an
  action without a subject.
- **Declarations lower as literal text; references defer.** A
  `ParameterDeclaration`/`VariableDeclaration` whose value contains `$` is
  reported as deferred to p4-s3 rather than stored as the literal text it
  happens to be — a stored `${$speed * 2}` would silently compare unequal to
  everything. Declared types are p4-s3's too: the by-value conditions compare
  stringly today.
- **World-frame geometry only where the IR is world-frame.** Trajectory
  vertices, NURBS control points, clothoid starts and route waypoints are
  `WorldPosition` in the IR. A road- or entity-relative vertex would have to
  be resolved at load time, when no entity state exists, so those are
  reported rather than guessed. Action and condition *targets* take the full
  ten-variant `Position` and resolve at run time, as they always did.
- **Deprecated spellings are accepted and mapped, with a warning.** The
  1.0/1.1 `ActivateControllerAction` placement directly under
  `PrivateAction`, the `overwrite` event priority (a lexical synonym of
  `override`, ADR-0005), `ParameterAction`, `ReachPositionCondition`,
  `alongRoute`, the pre-1.1 `GeoPosition` attribute names, and
  `curvatureDot`. Each maps onto its successor and reports
  `Status::DeprecatedFeature` — the coverage matrix's "accepted, deprecated"
  promise.
- **Never silent, and every deferral names its owner.** An element outside
  the loaded subset is a `Warning`/`UnsupportedFeature` diagnostic that says
  where it went: `p4-s3` (expressions), `p4-s4` (catalogs, ObjectController,
  entity selections), `#62` (relative speed profiles), or "Post-v0.0.1" for
  what the coverage matrix excludes.
- **ManeuverGroup `maximumExecutionCount` is read and reported, not
  honoured.** The IR has no field for it and §8.4.4 re-arming needs an
  invented restart rule (ADR-0005, #52); a document asking for more than one
  execution is told the group executes once, rather than being silently
  executed once.
- **A condition that cannot be lowered fails its whole trigger.** Dropping
  one condition would weaken the AND of its group into something the document
  never asked for, so the trigger is rejected and the defect reported.

## Consequences

- GS-1 loads from its committed `.xosc` and runs through the C++ API — the
  sprint's exit criterion — and the loaded scenario steps bit-identically
  across repeated runs, so the frontend introduces no ordering or formatting
  dependence into the IR.
- The IR-dump snapshot in `xml_storyboard_test.cpp` is the structural
  contract: a lowering change that moves an action, renames an element or
  reorders actors shows up as a diff there.
- p4-s3 (expressions), p4-s4 (catalogs, controllers, selections) and p4-s5
  (version rules, validation) each have a precise starting point: the
  warnings this sprint emits are their work lists.
- The frontend's `Document` grew a `RoadNetwork` member. No C ABI or Python
  change: the XML frontend is not exposed through either surface yet (P6).
