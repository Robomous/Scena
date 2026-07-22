# ASAM OpenSCENARIO XML 1.0–1.3 — v0.0.1 coverage matrix

Normative scope declaration for the XML frontend and runtime. Section
numbers follow the ASAM OpenSCENARIO XML 1.4.0 text (the local reference
copy); Scena **targets 1.0–1.3** — 1.4-only constructs are rejected by
version detection (p4-s1) and listed here as Excluded.

Statuses: **In** = In-v0.0.1 (implemented + tested before release);
**Post** = Post-v0.0.1 (parsed to a structured `UnsupportedFeature`
warning, never silently dropped; implementation deferred); **Excl** =
Excluded (with reason; rejected or warned, never executed).

Rules of this document:
- Every **In** row names the sprint that implements it; the sprint's tests
  cover it. CI's docs-consistency check greps this file against the sprint
  list in `docs/roadmap/roadmap.md`.
- Deprecated-in-1.x constructs that valid 1.0–1.3 files may contain are
  **In** with an "accepted, deprecated" note — Scena loads them and maps
  them to their successors.
- Introduction-version notes marked *(verify)* cannot be confirmed from the
  local reference text alone and need the per-version XSDs from the gated
  ASAM bundle (open question OQ-4 in the roadmap summary).

## Actions — private

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| SpeedAction | §7.4.1.4 | In | p2-s2 | Kernel landed: all TransitionDynamics shapes (linear/cubic/sinusoidal/step) × dimensions (time/rate/distance), default longitudinal controller, position-mode + hard Performance clamp (max speed + per-shape peak accel). Absolute target only — relative target deferred; `followingMode=follow` jerk deferred (ADR-0011). XML lowering deferred (P4/p5-s4) |
| SpeedProfileAction | §7.4.1.4 | In | p2-s2 | Kernel landed (≥1.2): position-mode piecewise-linear entry series, omitted entry time ⇒ performance-limited. entityRef-relative profile + DynamicConstraints/jerk (`followingMode=follow`) deferred (ADR-0011). XML lowering deferred (P4/p5-s4) |
| LongitudinalDistanceAction | §7.4.1.4 | In | p5-s5 | distance/timeGap modes, freespace, `continuous` keeping |
| LaneChangeAction | §7.4.1.4 | In | p5-s4 | Absolute/relative target lane, offset carryover; 1.4 lane-layer awareness excluded |
| LaneOffsetAction | §7.4.1.4 | In | p5-s4 | Incl. `continuous` variant |
| LateralDistanceAction | §7.4.1.4 | In | p5-s4 | Shares p2-s3 lateral machinery |
| TeleportAction | §7.4.1.4 | In | p5-s4 | All In-scope position variants (see Positions) |
| SynchronizeAction | §7.4.1.4 | Post | — | Requires master-relative speed synthesis (steady-state solve); deferred to keep v0.0.1 honest |
| VisibilityAction | §7.4.1.4 | In | p5-s5 | State flags surfaced via state/gateway; no sensor semantics in engine |
| ControllerAction (wrapper) | §7.4.1.4, §6.6 | In | p5-s5 | Wrapper for the three below |
| AssignControllerAction | §6.6.3 | In | p5-s5 | Controller metadata handed to host via gateway |
| ActivateControllerAction | §6.6.4 | In | p5-s5 | Lateral/longitudinal domains toggle engine default controller; lighting/animation domains → Post (appearance). Deprecated direct-under-PrivateAction placement accepted |
| OverrideControllerValueAction (+ Throttle/Brake/Clutch/ParkingBrake/SteeringWheel/Gear) | §7.4.1.4 | Post | — | Pedal/wheel-level overrides are host-controller business; no engine vehicle-device model in v0.0.1 |
| AssignRouteAction | §6.8.2 | In | p5-s5 | Route + RouteStrategy over P3 routes |
| FollowTrajectoryAction | §6.9 | In | p5-s5 | timeReference none/timing × followingMode follow/position |
| AcquirePositionAction | §7.4.1.4 | In | p5-s5 | Implicit two-waypoint route |
| RandomRouteAction | §7.4.1.4 | Excl | — | Randomness violates the determinism contract; post-release only with host-seeded selection design *(verify introduction version)* |
| PreferredLaneLayerAction | §7.4.1.3 | Excl | — | 1.4-only; outside targeted versions |
| AnimationAction | §6.7 | Post | — | ≥1.2; appearance domain, no engine semantics in v0.0.1 |
| LightStateAction | §6.7 | Post | — | ≥1.2; appearance domain |
| ConnectTrailerAction / DisconnectTrailerAction | §7.2.2.6 | Post | — | Trailer/hitch model out of v0.0.1 entity scope *(verify introduction version)* |

## Actions — global

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| EnvironmentAction | §7.4.2 | In | p5-s6 | Environment state store + TimeOfDay clock only; no physics/visual coupling (documented simplification) |
| AddEntityAction | §7.4.2 | In | p5-s6 | Deterministic entity-table update |
| DeleteEntityAction | §7.4.2 | In | p5-s6 | Deterministic entity-table update |
| ParameterSetAction | class ref | In | p5-s6 | Accepted, deprecated; lowered onto the runtime variable store (1.0/1.1 compat) |
| ParameterModifyAction (+ ModifyRule, Add/MultiplyByValueRule) | class ref | In | p5-s6 | Accepted, deprecated; same lowering |
| VariableSetAction | §6.12 | In | p5-s6 | ≥1.2 |
| VariableModifyAction (+ rules) | §6.12 | In | p5-s6 | ≥1.2; numeric types only per rule C.2.6 |
| TrafficSignalControllerAction | §6.11 | In | p5-s6 | Phase model; 1.4 phase *semantics* (TrafficSignalSemantics) excluded |
| TrafficSignalStateAction | §6.11 | In | p5-s6 | Named signal observable state |
| TrafficSourceAction | §6.10 | Post | — | Ambient-traffic generation (with distributions) is a post-release feature family |
| TrafficSinkAction | §6.10 | Post | — | Same family |
| TrafficSwarmAction | §6.10 | Post | — | Same family; also inherently stochastic |
| TrafficAreaAction | §7.4.2 | Excl | — | ≥1.3 per rule C.7.23 **and** in the deferred traffic family; excluded to avoid a partially supported family |
| TrafficStopAction | §7.4.2 | Post | — | Meaningless without the traffic family |
| SetMonitorAction | §6.14 | Post | — | Monitor surface deferred with monitors *(verify introduction version)* |

## Actions — user defined

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| UserDefinedAction / CustomCommandAction | §7.4.3 | In | p5-s6 | Host callback through the gateway; no-op without a host (documented contract) |

## Conditions — by entity

All with `TriggeringEntities` any/all semantics (§7.6.5.1) — evaluated
per entity then reduced, before the trigger edge/delay machinery. The
kinematics/position set (p5-s2) landed cartesian-only under Scena's
scalar-velocity model; the interaction metrics (p5-s3) added the §6.4
distance matrix (euclidean / longitudinal / lateral, reference-point and
bounding-box freespace via a deterministic 2D OBB kernel), TimeHeadway,
TimeToCollision and Collision. **Road/lane/trajectory coordinate systems and
the road-topology predicates (EndOfRoad, Offroad, RelativeClearance) evaluate
to a deterministic false with an init-time UnsupportedFeature warning until
`IRoadQuery` lands (p3-s4)**; Collision matches an EntityRef target only
(ByObjectType deferred to p2-s1). Geometry is a minimal `BoundingBox`
forward-pull from p2-s1 (ADR-0009). `TriggeringEntities` holds plain entity
references; `EntitySelection` references are out of scope until the selection
IR lands (p4-s4).

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| AccelerationCondition | §7.6.5.1 | In | p5-s2 | Kernel landed: finite-difference acceleration, absent (⇒false) until two samples; optional direction (longitudinal signed, lateral/vertical 0 in the scalar model). XML lowering deferred (P4) |
| AngleCondition | §7.6.5.1 | Post | — | Introduction version unconfirmed in local text *(verify)*; low demand for v0.0.1 set |
| CollisionCondition | §7.6.5.1 | In | p5-s3 | Kernel landed: bounding-box (2D OBB) intersection, touching ⇒ collision (§6.4.7.2); absent box on either entity ⇒ false. EntityRef target only — ByObjectType categories landed p2-s1 (ADR-0010); wiring the target into this condition is a follow-up. XML lowering deferred (P4) |
| DistanceCondition | §7.6.5.1, §6.4 | In | p5-s3 | Kernel landed: euclidean (3D reference-point / 2D freespace) and longitudinal/lateral (entity or world axis); road/lane/trajectory CS ⇒ deterministic false + UnsupportedFeature warning (deferred p3-s4). Deprecated `alongRoute`/`cartesianDistance` warned and mapped. XML lowering deferred (P4) |
| EndOfRoadCondition | §7.6.5.1 | In | p5-s3 | IR + validation landed: requires road-network topology (p3-s4), so it evaluates deterministic false with a rule-cited UnsupportedFeature warning until then. XML lowering deferred (P4) |
| OffroadCondition | §7.6.5.1 | In | p5-s3 | IR + validation landed: road-deferred like EndOfRoad — deterministic false + UnsupportedFeature warning until p3-s4. XML lowering deferred (P4) |
| ReachPositionCondition | §7.6.5.1 | In | p5-s2 | Kernel landed: deprecated (1.2) ⇒ DeprecatedFeature warning, still evaluated; 2D horizontal tolerance circle (z ignored) against a minimal WorldPosition. Road Position variants + PositionResolver deferred (p2-s4/p3-s4); XML lowering deferred (P4) |
| RelativeAngleCondition | §7.6.5.1 | Post | — | Same rationale as AngleCondition *(verify)* |
| RelativeClearanceCondition | §7.6.5.1 | In | p5-s3 | IR + validation landed (freeSpace/oppositeLanes, distance window, entity refs, RelativeLaneRange): the checked area is in lane coordinates (§6.4.5), so it evaluates deterministic false + UnsupportedFeature warning until p3-s4. XML lowering deferred (P4) |
| RelativeDistanceCondition | §7.6.5.1, §6.4 | In | p5-s3 | Kernel landed: entity-to-entity distance, relativeDistanceType required, no alongRoute; freespace via bounding boxes; road/lane/trajectory CS ⇒ deterministic false + UnsupportedFeature warning (p3-s4). XML lowering deferred (P4) |
| RelativeSpeedCondition | §7.6.5.1 | In | p5-s2 | Kernel landed: spec formula `speed_rel = speed(trig) − speed(ref)` (signed); directional projection in the triggering frame via det_sincos; absent reference ⇒ false. XML lowering deferred (P4) |
| SpeedCondition | §7.6.5.1 | In | p5-s2 | Kernel landed: total = \|speed\|, optional directional projection. XML lowering deferred (P4) |
| StandStillCondition | §7.6.5.1 | In | p5-s2 | Kernel landed: contiguous time at speed exactly 0.0 (no invented ε), `>=` duration. XML lowering deferred (P4) |
| TimeHeadwayCondition | §7.6.5.1, §6.4 | In | p5-s3 | Kernel landed: distance ÷ the triggering entity's speed only (reference leading); stopped/reversing follower ⇒ false. Full distance matrix incl. freespace; deprecated `alongRoute` warned and mapped; road CS deferred false (p3-s4). XML lowering deferred (P4) |
| TimeToCollisionCondition | §7.6.5.1, §6.4 | In | p5-s3 | Kernel landed: distance ÷ closing speed (no acceleration), entity XOR position target; diverging/zero closing speed or coincident points ⇒ false. Road CS deferred false (p3-s4). XML lowering deferred (P4) |
| TraveledDistanceCondition | §7.6.5.1 | In | p5-s2 | Kernel landed: cumulative world-frame path length from init (not displacement), `>=` value. XML lowering deferred (P4) |

## Conditions — by value

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| ParameterCondition | §7.6.5.2 | In | p5-s1 | Kernel landed: shared `Rule` comparator, exact IEEE numeric compare, ordering only when both operands are scalar-convertible; immutable at runtime (§9.1); dangling `parameterRef` ⇒ init `SemanticError`. XML lowering deferred (P4) |
| VariableCondition | §7.6.5.2, §6.12 | In | p5-s1 | Kernel landed (≥1.2): runtime variable store seeded at init, host `set_variable`; strings equal/not-equal, numeric when scalar-convertible; dangling `variableRef` ⇒ init `SemanticError` (rule `reference_control.resolvable_variable_reference`). XML lowering deferred (P4) |
| SimulationTimeCondition | §7.6.5.2 | In | p5-s1 | Kernel landed: gained the spec's `rule` (default greaterOrEqual); time starts when the Storyboard enters running (§8.4.7); NaN value ⇒ init `ValidationError`. XML lowering deferred (P4) |
| StoryboardElementStateCondition | §7.6.5.2, §8.1–8.2 | In | p5-s1 | Kernel landed: level states + one-evaluation transition pulses; nameRef `::` resolution; zero/ambiguous ref ⇒ init `SemanticError` (rule `reference_control.resolvable_storyboard_element_ref`); `action` type unsupported until p5-s4 (warns). XML lowering deferred (P4) |
| TimeOfDayCondition | §7.6.5.2 | In | p5-s1 | Kernel landed: host-anchored simulated clock advancing with sim time, epoch-seconds compare (UTC offset honored), leap-year-correct; unset anchor ⇒ false + warn-once. Same anchor fed later by the EnvironmentAction clock (p5-s6). XML lowering deferred (P4) |
| TrafficSignalCondition | §7.6.5.2, §6.11 | In | p5-s6 | Lands with the signal actions |
| TrafficSignalControllerCondition | §7.6.5.2, §6.11 | In | p5-s6 | Phase reached; 1.4 semantics excluded |
| UserDefinedValueCondition | §7.6.5.2 | In | p5-s1 | Kernel landed: host-provided named values (`set_user_defined_value`, stageable pre-init); unset name ⇒ false + warn-once. XML lowering deferred (P4) |

## Storyboard & runtime semantics

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| Storyboard / Story / Act / ManeuverGroup / Maneuver / Event / Action nesting | §7.2.1, §7.3 | In | p1-s1 | |
| Element state machine (standby/running/complete; start/end/stop/skip transitions) | §8.1–8.3 | In | p1-s1, p1-s3 | Landed: skipTransition in p1-s3, emitted for both of its meanings (§8.2 priority skip and §8.4.2.1 exhausted execution count), told apart by the resulting state |
| Init phase (global + private init actions before simulation time) | §8.5 | In | p1-s1, p4-s2 | Non-instantaneous init actions carry into the storyboard |
| Trigger = OR(ConditionGroups); group = AND(Conditions); empty trigger false | §7.6.1 | In | p1-s2 | Landed: absent trigger (`nullopt`) distinct from empty (always false); empty ConditionGroup rejected at init (1..\* cardinality); no short-circuit evaluation |
| ConditionEdge none/rising/falling/risingOrFalling + first-evaluation corner cases | §7.6.2, §7.6.4 | In | p1-s2 | Landed: per-condition history; first check of an edge condition false, starting when the enclosing element enters standby |
| Condition delay (evaluates state of t−Δt; Δt ≥ 0) | §7.6.3 | In | p1-s2 | Rule-cited validation. Landed: delay applies post-edge (per `Condition.delay` class reference over §7.6.3 prose); sample-and-hold lookup of the most recent evaluation at or before t−Δt, exact comparisons |
| Start triggers (Act, Event only), stop triggers (Storyboard, Act) + inheritance | §7.6.1.1–7.6.1.2 | In | p1-s2 | Landed: stop checked before start at every level; stop applies from standby and running; execution-count clearing hooked in `stop_cascade` for p1-s3 |
| Event priority `override`/`skip`/`parallel` | §7.3.2, §8.4.2.2 | In | p1-s3 | Landed: scope is the Maneuver (§7.3.3); `override` stops running siblings only, clearing their remaining executions, through the same path as a stop trigger; `skip` is conditional on a running sibling (`Priority` class reference, which reconciles §7.3.2 with §8.4.2.2) and its skipTransition counts as an execution; priority applies to trigger-less events via the Act-inherited trigger (§8.4.2). Simultaneous triggers resolved in document order in a single pass — the standard gives no rule. Deprecated literal `overwrite` is a lexical synonym with no separate IR value; the frontend maps it at load time (p4-s2), which also applies the XSD default |
| maximumExecutionCount (Event) | §8.3.3.2, §8.4.2.1 | In | p1-s3 | Landed: executions = startTransitions + skipTransitions, spent sequentially; an endTransition re-arms to standby while executions remain and completes the event once exhausted; a standby event at its maximum completes with a skipTransition; condition history is *not* reset on re-arm, so an edge trigger must rise again; `0` accepted (schema-valid, §8.4.2.1 gives it a coherent reading) and negative rejected at init; stop triggers and `override` clear the remaining budget. A trigger-less event executes once — its Act-inherited trigger has already fired (§8.4.2) |
| maximumExecutionCount (ManeuverGroup) | §8.3.3.2, §8.4.4 | In | p4-s2 | Deferred from p1-s3 (ADR-0005, issue #52): §8.4.4 re-arms the group onto the Act-inherited start trigger, which is not re-evaluated while the act runs, so any behaviour requires an invented restart rule with determinism consequences — an ADR-level decision that belongs with the sprint loading the attribute |
| Action lifetime (instantaneous vs never-ending) | §7.4.1.2, §7.5.3 | In | p1-s3, p5-s4 | Landed: two-valued action outcome reported by the applier — complete on application, or ongoing and ended only by a stopTransition. Every shipped action is the §7.4.1.2 step-dynamic case. Finite-duration actions governed by transition dynamics arrive as an additive third outcome with p2-s2/p5-s4 (ADR-0005) |
| Action conflict resolution & completion reasons | §7.5 | In | p5-s4 | Deferred from p1-s3 (ADR-0005, issue #51): §7.5.1 conflicts require two actions competing for a domain, and every shipped action is the §7.4.1.2 step-dynamic case that assigns no control strategy, so the machinery would be unreachable until p2-s2. Covers continuous actions (§7.5.3), bulk/actor semantics (§7.5.4, §8.3.3.3) and stop-notification of ongoing actions (§7.5.2.1) |
| Storyboard stop trigger as sole completion | §8.4.7 | In | p1-s1 | |

## Document & reuse machinery

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| OpenScenario root / FileHeader (revMajor/revMinor 1.0–1.3) | class ref | In | p4-s1 | 1.4 rejected with diagnostic |
| ScenarioDefinition (incl. RoadNetwork reference) | class ref | In | p4-s2 | RoadNetwork logic file handed to road backend/host |
| ParameterDeclaration + ValueConstraint(Group) | §9.1 | In | p4-s3 | |
| Expressions `$param`, `${...}` | §9.2 | In | p4-s3 | ≥1.1; full operator whitelist + typing rules; 1.4-only constant `pi` rejected |
| VariableDeclaration | §6.12 | In | p4-s3 | ≥1.2; runtime store in kernel |
| MonitorDeclaration | §6.14 | Post | — | With SetMonitorAction *(verify version)* |
| Catalog / CatalogReference / CatalogLocations (vehicle, controller, pedestrian, miscObject, environment, maneuver, trajectory, route) | §9.4–9.6 | In | p4-s4 | 1.4-only trafficDistributionEntry catalog excluded |
| ParameterValueDistribution files (Deterministic/Stochastic) | §9.3 | Post | — | Parameter-variation workflow belongs with the post-release generation family |
| Entities: ScenarioObject + inline Vehicle/Pedestrian/MiscObject | §7.2.2 | In | p2-s1, p4-s2 | IR + bindings landed (p2-s1): ObjectType/category/Role enums (full 1.4 sets, 1.3→1.4 deprecations noted), `Performance`, axles and properties as data, bounding boxes, EntityObject variant + `object_type_of`/`bounding_box_of`/`performance_of`; full h/p/r `EntityState`; C-ABI typed builders + metadata accessors; Performance range validation (ADR-0010). XML lowering → p4-s2 |
| ExternalObjectReference | §7.2.2 | Post | — | Requires road-network object binding beyond v0.0.1 map scope |
| Trailer attributes on Vehicle | §7.2.2.6 | Post | — | With the trailer action family |
| EntitySelection / ByType / ByObjectType | §7.2.2.2–7.2.2.5 | In | p4-s4 | Homogeneity rules cited |
| ObjectController / Controller (+ properties) | §6.6 | In | p4-s4 | Domain typing; multiple controllers ≥1.2 accepted |
| Environment / Weather / TimeOfDay / RoadCondition (as data) | §7.4.2 | In | p4-s4, p5-s6 | Stored + queryable; no physics coupling |
| TrafficDefinition / TrafficDistribution | §6.10 | Post | — | With the traffic family |

## Positions, trajectories, routes

| Element | Section | Status | Sprint | Notes |
|---|---|---|---|---|
| WorldPosition | §6.3.8 | In | p2-s4 | ≥1.3 corrected calculations applied to all versions (per §5) |
| RelativeWorldPosition | §6.3.8 | In | p2-s4 | |
| RelativeObjectPosition | §6.3.8 | In | p2-s4 | |
| RoadPosition / RelativeRoadPosition | §6.3.8 | In | p2-s4, p3-s4 | Via `IRoadQuery` |
| LanePosition / RelativeLanePosition | §6.3.8 | In | p2-s4, p3-s4 | Via `IRoadQuery` |
| RoutePosition (all InRoutePosition variants) | §6.3.8 | In | p2-s4, p3-s4 | |
| TrajectoryPosition | §6.3.8 | In | p2-s5 | |
| GeoPosition | §6.3.8 | Post | — | Needs geodetic datum handling (rule-cited diagnostic when encountered) |
| Orientation + ReferenceContext | §6.3.8 | In | p2-s4 | |
| Trajectory + Polyline / Clothoid / Nurbs | §6.9 | In | p2-s5 | |
| ClothoidSpline | §6.9 | Post | — | Introduction version unconfirmed *(verify)*; clothoid single-segment covers v0.0.1 need |
| Motion class / Polyline Interpolation | §6.9 | Excl | — | 1.4-only |
| TimeReference / Timing / TrajectoryFollowingMode | §6.9.1–6.9.5 | In | p2-s5, p5-s5 | |
| Route / Waypoint / RouteStrategy / RouteRef | §6.8 | In | p3-s3, p5-s5 | RouteStrategy `random` → Excl (determinism); fastest/shortest/leastIntersections In |
| TrafficSignalController / Phase / TrafficSignalState | §6.11 | In | p5-s6 | 1.4 TrafficSignalSemantics / GroupState excluded |

## Version-handling decisions

- **1.0–1.3 accepted; 1.4 rejected** at `FileHeader` (p4-s1).
- **≥1.3 corrected position/orientation calculations applied uniformly**
  to all input versions, exactly as §5 prescribes; no per-version dual
  semantics.
- Deprecated constructs (see rows marked "accepted, deprecated") load with
  a warning diagnostic and map onto their successors; the strict
  no-deprecated schema variants are not enforced by Scena.
- Elements whose introduction version the local reference cannot confirm
  are marked *(verify)*; confirming them requires the per-version XSDs
  from the gated ASAM bundle (maintainer action, roadmap open question).
  Until verified, encountering them in a file that predates their
  (suspected) introduction yields the same structured warning as any
  Post/Excluded element — never a crash, never silence.

## Declared coverage summary (targets 1.0–1.3)

- Actions: **23 of 38** concrete action classes In (61%); 12 Post; 3 Excl
  (1.4-only or determinism-incompatible).
- Conditions: **21 of 24** condition classes In (88%); 3 Post.
- All storyboard/runtime semantics constructs: In (100%).
- Positions: 9 of 10 variants In (GeoPosition Post).
- Trajectory shapes: polyline, clothoid, NURBS In; ClothoidSpline Post;
  1.4 additions Excl.

Counting rule: concrete (non-abstract) classes from the Annex A action
tables and the §7.6.5 condition set; deprecated classes counted where
1.0–1.3 files may legally contain them.
