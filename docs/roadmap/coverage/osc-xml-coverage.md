# ASAM OpenSCENARIO XML 1.0–1.3 — v0.0.1 coverage matrix

Normative scope declaration for the XML frontend and runtime. Section
numbers follow the ASAM OpenSCENARIO XML 1.4.0 text (the local reference
copy); Scena **targets 1.0–1.3** — 1.4-only constructs are rejected by
version detection (p4-s1) and listed here as Excluded.

Statuses: **In** = In-v0.0.1 (implemented + tested before release);
**Post** = Post-v0.0.1 (parsed to a structured `UnsupportedFeature`
warning, never silently dropped; implementation deferred); **Excl** =
Excluded (with reason; rejected or warned, never executed).

**In marks scope commitment, not delivery.** The **Tests** column records
delivery state: a delivered row names the suites that prove it
(`core/tests/` and `python/tests/` file names); a row whose implementation
has not landed yet carries a *planned (sprint)* marker, and a row that was
descoped out of an otherwise-completed sprint carries a *deferred (#issue)*
marker naming the issue that now owns it.

Rules of this document:
- Every **In** row names the sprint that implements it, and its **Tests**
  cell either names the test suites that prove it or carries a
  planned/deferred marker with the owning sprint or issue. CI's
  docs-consistency check (`scripts/check_coverage_matrix.py`) enforces
  both against the sprint list in `docs/roadmap/roadmap.md` and verifies
  that every named test file exists in the tree.
- Deprecated-in-1.x constructs that valid 1.0–1.3 files may contain are
  **In** with an "accepted, deprecated" note — Scena loads them and maps
  them to their successors.
- Introduction-version notes marked *(verify)* cannot be confirmed from the
  local reference text alone and need the per-version XSDs from the gated
  ASAM bundle (open question OQ-4 in the roadmap summary).

## Actions — private

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| SpeedAction | §7.4.1.4 | In | p2-s2, p5-s4 | `longitudinal_test.cpp`, `action_longitudinal_test.cpp`, `test_longitudinal.py` | Kernel landed: all TransitionDynamics shapes (linear/cubic/sinusoidal/step) × dimensions (time/rate/distance), default longitudinal controller, position-mode + hard Performance clamp (max speed + per-shape peak accel) (p2-s2). Relative target (§RelativeTargetSpeed): delta/factor, one-shot + continuous tracking that never ends (§7.5.3); minimal single-domain supersession (p5-s4, ADR-0013). `followingMode=follow` jerk deferred (ADR-0011, #62). XML lowering deferred (P4/p5-s4) |
| SpeedProfileAction | §7.4.1.4 | In | p2-s2 | `longitudinal_test.cpp`, `test_longitudinal.py` | Kernel landed (≥1.2): position-mode piecewise-linear entry series, omitted entry time ⇒ performance-limited. entityRef-relative profile + DynamicConstraints/jerk (`followingMode=follow`) deferred (ADR-0011, #62). XML lowering deferred (P4/p5-s4) |
| LongitudinalDistanceAction | §7.4.1.4 | In | p5-s5 | `action_distance_test.cpp`, `test_distance_keeping.py` | Kernel landed: distance and timeGap targets, freespace (OBB) and reference-point gaps, `displacement` (any/leading/trailing), DynamicConstraints + Performance clamping, `continuous` keeping that never ends (§7.5.3); longitudinal-domain owner, so it supersedes and is superseded by SpeedAction/SpeedProfileAction (ADR-0014). Road-based coordinateSystem → p3-s4 (UnsupportedFeature warning + immediate completion); DynamicConstraints jerk rates stored but not clamped (#62). XML lowering deferred (P4) |
| LaneChangeAction | §7.4.1.4 | In | p2-s3 | `action_lateral_test.cpp`, `lateral_test.cpp`, `determinism_test.cpp`, `test_lateral.py` | Kernel landed: relative and absolute target lane, `targetLaneOffset`, all TransitionDynamics shapes × dimensions (Distance tied to the odometer, Step instantaneous), heading-blended lateral kinematics, lateral-domain supersession (ADR-0016). Flat-world lane widths (configurable `default_lane_width`) with a forward-pulled IRoadQuery lane-query path; absolute-lane/road-backed resolution remains with #23 (UnsupportedFeature warning + completion). 1.4 lane-layer awareness excluded (Scena targets 1.0–1.3). XML lowering deferred (P4) |
| LaneOffsetAction | §7.4.1.4 | In | p2-s3 | `action_lateral_test.cpp`, `lateral_test.cpp`, `test_lateral.py` | Kernel landed: absolute and relative offset targets, duration derived from `maxLateralAcc` per shape (k = 4 / π²/2 / 6; missing ⇒ 'inf' ⇒ instantaneous, as is Step), `continuous` keeping that never ends (§7.5.3) and re-corrects when the target moves (ADR-0016). A zero `maxLateralAcc` is rejected at init. XML lowering deferred (P4) |
| LateralDistanceAction | §7.4.1.4 | In | p2-s3 | `action_lateral_test.cpp`, `test_lateral.py` | Kernel landed: freespace (OBB) and reference-point lateral gaps, `displacement` (any/left/right), DynamicConstraints-limited or rigid keeping, `continuous` that never ends (§7.5.3); lateral-domain owner (ADR-0016). Lane coordinateSystem is undefined by §6.4.8.2.2 (warned at init + completion); road/trajectory coordinateSystem → p3-s4. Performance is deliberately not applied (its limits are longitudinal). XML lowering deferred (P4) |
| TeleportAction | §7.4.1.4 | In | p5-s4, p2-s4 | `action_teleport_test.cpp`, `control_ownership_test.cpp`, `test_private_actions.py` | Kernel landed: step (instantaneous) write of the entity's full pose (position + orientation) from any §6.3.8 Position resolved through the `PositionResolver`; self-contained variants resolve, road/route/geo/trajectory reported (p3-s4/p2-s5). A teleport on a host-controlled entity is a reported mode violation (ADR-0013, ADR-0017). XML lowering deferred (P4) |
| SynchronizeAction | §7.4.1.4 | Post | — | — | Requires master-relative speed synthesis (steady-state solve); deferred to keep v0.0.1 honest |
| VisibilityAction | §7.4.1.4 | In | p5-s5 | `action_controller_visibility_test.cpp` | Kernel landed: graphics/sensors/traffic flags stored per entity, readable via `Engine::visibility_of` and handed to the host through `ISimulatorGateway::on_visibility_changed`; no sensor semantics in the engine (ADR-0014). `sensorReferenceSet` (1.2+) → Post. XML lowering deferred (P4) |
| ControllerAction (wrapper) | §7.4.1.4, §6.6 | In | p5-s5 | `action_controller_visibility_test.cpp` | Wrapper for the three below |
| AssignControllerAction | §6.6.3 | In | p5-s5 | `action_controller_visibility_test.cpp` | Kernel landed: controller name, controllerType (1.3) and ordered properties stored per entity and handed to the host through `ISimulatorGateway::on_controller_assigned`; activateLateral/activateLongitudinal applied, activation outside the controllerType rejected (rule `scenario_logic.controller_activation`). Lighting/animation activation → Post (appearance). XML lowering deferred (P4) |
| ActivateControllerAction | §6.6.4 | In | p5-s5 | `action_controller_visibility_test.cpp` | Kernel landed: the lateral/longitudinal flags toggle Scena's engine default controller, deactivation retiring the domain's current owner and suppressing later actions on it (ADR-0014); lighting/animation domains → Post (appearance). Deprecated direct-under-PrivateAction placement accepted (a frontend concern, P4). XML lowering deferred (P4) |
| OverrideControllerValueAction (+ Throttle/Brake/Clutch/ParkingBrake/SteeringWheel/Gear) | §7.4.1.4 | Post | — | — | Pedal/wheel-level overrides are host-controller business; no engine vehicle-device model in v0.0.1 |
| AssignRouteAction | §6.8.2 | In | p5-s5 | `action_routing_test.cpp`, `test_routing_actions.py` | Assignment landed: Route/Waypoint/RouteStrategy over world-frame waypoints, installed per entity and readable via `Engine::route_of`, overwriting any prior route (§6.8.2) and completing immediately (Table 10) without touching a control domain (§7.4.1.4). Road semantics — waypoint-on-road prerequisites, RouteStrategy interpretation, route-following motion — → p3-s4. XML lowering deferred (P4) |
| FollowTrajectoryAction | §6.9 | In | p2-s5, p5-s5 | `action_routing_test.cpp`, `trajectory_test.cpp`, `determinism_test.cpp`, `test_trajectory.py`, `capi_test.cpp` | Polyline (p5-s5) plus Clothoid and NURBS shapes (p2-s5, ADR-0018): both drive through the arc-length evaluator, `timeReference` none (entity's own speed) and timing (linear time→arclength over the shape's start/end times), `initialDistanceOffset`, exact completion at the trajectory end. `followingMode=follow` accepted-as-position and closed trajectories accepted-as-open, both with an UnsupportedFeature warning (true `follow` steering deferred). ClothoidSpline, the Motion element and trajectory catalogs remain out (see below). XML lowering deferred (P4) |
| AcquirePositionAction | §7.4.1.4 | In | p5-s5, p2-s4 | `action_routing_test.cpp` | Assignment landed: the implicit two-waypoint route of §7.4.1.4 (position at apply time → target, strategy shortest), installed and completed immediately (Table 10). The target is any §6.3.8 Position resolved through the `PositionResolver` (p2-s4, ADR-0017); road semantics → p3-s4. XML lowering deferred (P4) |
| RandomRouteAction | §7.4.1.4 | Excl | — | — | Randomness violates the determinism contract; post-release only with host-seeded selection design *(verify introduction version)* |
| PreferredLaneLayerAction | §7.4.1.3 | Excl | — | — | 1.4-only; outside targeted versions |
| AnimationAction | §6.7 | Post | — | — | ≥1.2; appearance domain, no engine semantics in v0.0.1 |
| LightStateAction | §6.7 | Post | — | — | ≥1.2; appearance domain |
| ConnectTrailerAction / DisconnectTrailerAction | §7.2.2.6 | Post | — | — | Trailer/hitch model out of v0.0.1 entity scope *(verify introduction version)* |

## Actions — global

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| EnvironmentAction | §7.4.2 | In | p5-s6 | `action_global_test.cpp`, `test_global_actions.py` | Kernel landed: Environment/Weather/Sun/Fog/Precipitation/Wind/RoadCondition value types merged member-wise per §Environment ("a missing condition doesn't change"), readable via `Engine::environment`; TimeOfDay re-anchors the simulated clock and its `animation` flag freezes or advances it, feeding TimeOfDayCondition unchanged (ADR-0015). No physics or visual coupling — weather is stored, never applied (documented simplification). `RoadCondition.wetness` (1.2) + Properties, the DomeImage, and the CatalogReference form → Post/P4. XML lowering deferred (P4) |
| AddEntityAction | §7.4.2 | In | p5-s6, p2-s4 | `action_global_test.cpp` | Kernel landed: an `active` flag on the entity record (never an erase/insert, so iteration order and bookkeeping are untouched); a fresh state at any §6.3.8 Position resolved through the `PositionResolver` — full pose incl. orientation — with the observation baseline seeded as at init (p2-s4, ADR-0017). Adding an active entity is a no-op (§7.4.2). XML lowering deferred (P4) |
| DeleteEntityAction | §7.4.2 | In | p5-s6 | `action_global_test.cpp` | Kernel landed: clears all runtime motion/assignment/observation state, keeps the declared immutables; the entity is skipped by poll/refresh/integrate/publish, reports nothing to the host, and is absent from the by-entity conditions. Running private actions on it or on it as a reference stop (§7.5.2.2), surfacing as the owning event ending one evaluation later (ADR-0015). Deleting an inactive entity is a no-op. XML lowering deferred (P4) |
| ParameterSetAction | class ref | In | p5-s6 | `action_global_test.cpp` | Kernel landed: accepted, deprecated with 1.2, executed against a runtime overlay consulted ahead of the immutable §9.1 declaration so a 1.0/1.1 file's ParameterCondition observes it; warn-once `DeprecatedFeature` (ADR-0015). XML lowering deferred (P4) |
| ParameterModifyAction (+ ModifyRule, Add/MultiplyByValueRule) | class ref | In | p5-s6 | `action_global_test.cpp` | Kernel landed: same overlay and deprecation warning; add/multiplyByValue over the overlay, numeric values only per rule C.2.6. XML lowering deferred (P4) |
| VariableSetAction | §6.12 | In | p5-s6 | `action_global_test.cpp` | Kernel landed (≥1.2): writes the runtime variable store a VariableCondition and the host both read. Undeclared ref rejected at init (rule reference_control.resolvable_variable_reference). XML lowering deferred (P4) |
| VariableModifyAction (+ rules) | §6.12 | In | p5-s6 | `action_global_test.cpp` | Kernel landed (≥1.2): add/multiplyByValue as one fixed IEEE expression, stored back through the shortest round-tripping decimal so a modify chain is exact. Non-numeric current value ⇒ warn-once citing rule data_type.variable_modification_or_comparison_possible (C.2.6) and no-op; typed declarations → p4-s3. XML lowering deferred (P4) |
| TrafficSignalControllerAction | §6.11 | In | p5-s6 | `traffic_signal_test.cpp`, `test_traffic_signals.py` | Kernel landed: restarts a controller's cycle at a named phase (re-anchor + immediate tick, so same-evaluation conditions agree) and continues in declared order; both references validated at init (rule reference_control.traffic_signal_controller_action_references, C.7.11). 1.4 phase *semantics* (TrafficSignalSemantics) excluded. XML lowering deferred (P4) |
| TrafficSignalStateAction | §6.11 | In | p5-s6 | `traffic_signal_test.cpp` | Kernel landed: forces a named signal's observable state, which stands until the controlling cycle's next phase transition (actions-win precedence, the §11.12 bulb-failure shape, ADR-0015). Signal id free-form — rule C.7.14 needs a road network (p3-s4). XML lowering deferred (P4) |
| TrafficSourceAction | §6.10 | Post | — | — | Ambient-traffic generation (with distributions) is a post-release feature family |
| TrafficSinkAction | §6.10 | Post | — | — | Same family |
| TrafficSwarmAction | §6.10 | Post | — | — | Same family; also inherently stochastic |
| TrafficAreaAction | §7.4.2 | Excl | — | — | ≥1.3 per rule C.7.23 **and** in the deferred traffic family; excluded to avoid a partially supported family |
| TrafficStopAction | §7.4.2 | Post | — | — | Meaningless without the traffic family |
| SetMonitorAction | §6.14 | Post | — | — | Monitor surface deferred with monitors *(verify introduction version)* |

## Actions — user defined

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| UserDefinedAction / CustomCommandAction | §7.4.3 | In | p5-s6 | `action_global_test.cpp`, `test_global_actions.py` | Kernel landed: `type` and `content` handed verbatim to `ISimulatorGateway::on_custom_command` (defaulted virtual, an ADR-0003 amendment); a silent no-op without a gateway, per §7.4.3 executability — no diagnostic (ADR-0015). XML lowering deferred (P4) |

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

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| AccelerationCondition | §7.6.5.1 | In | p5-s2 | `condition_byentity_kinematics_test.cpp`, `test_byentity_conditions.py` | Kernel landed: finite-difference acceleration, absent (⇒false) until two samples; optional direction (longitudinal signed, lateral/vertical 0 in the scalar model). XML lowering deferred (P4) |
| AngleCondition | §7.6.5.1 | Post | — | — | Introduction version unconfirmed in local text *(verify)*; low demand for v0.0.1 set |
| CollisionCondition | §7.6.5.1 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | Kernel landed: bounding-box (2D OBB) intersection, touching ⇒ collision (§6.4.7.2); absent box on either entity ⇒ false. EntityRef target only — ByObjectType categories landed p2-s1 (ADR-0010); wiring the target into this condition is a follow-up. XML lowering deferred (P4) |
| DistanceCondition | §7.6.5.1, §6.4 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | Kernel landed: euclidean (3D reference-point / 2D freespace) and longitudinal/lateral (entity or world axis); road/lane/trajectory CS ⇒ deterministic false + UnsupportedFeature warning (deferred p3-s4). Deprecated `alongRoute`/`cartesianDistance` warned and mapped. XML lowering deferred (P4) |
| EndOfRoadCondition | §7.6.5.1 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | IR + validation landed: requires road-network topology (p3-s4), so it evaluates deterministic false with a rule-cited UnsupportedFeature warning until then. XML lowering deferred (P4) |
| OffroadCondition | §7.6.5.1 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | IR + validation landed: road-deferred like EndOfRoad — deterministic false + UnsupportedFeature warning until p3-s4. XML lowering deferred (P4) |
| ReachPositionCondition | §7.6.5.1 | In | p5-s2 | `condition_byentity_kinematics_test.cpp`, `test_byentity_conditions.py` | Kernel landed: deprecated (1.2) ⇒ DeprecatedFeature warning, still evaluated; 2D horizontal tolerance circle (z ignored) against a minimal WorldPosition. The `PositionResolver` landed in p2-s4 (ADR-0017) but was wired into the position **actions** only; generalizing the condition's target to any §6.3.8 variant follows when road targets are needed (p3-s4). XML lowering deferred (P4) |
| RelativeAngleCondition | §7.6.5.1 | Post | — | — | Same rationale as AngleCondition *(verify)* |
| RelativeClearanceCondition | §7.6.5.1 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | IR + validation landed (freeSpace/oppositeLanes, distance window, entity refs, RelativeLaneRange): the checked area is in lane coordinates (§6.4.5), so it evaluates deterministic false + UnsupportedFeature warning until p3-s4. XML lowering deferred (P4) |
| RelativeDistanceCondition | §7.6.5.1, §6.4 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | Kernel landed: entity-to-entity distance, relativeDistanceType required, no alongRoute; freespace via bounding boxes; road/lane/trajectory CS ⇒ deterministic false + UnsupportedFeature warning (p3-s4). XML lowering deferred (P4) |
| RelativeSpeedCondition | §7.6.5.1 | In | p5-s2 | `condition_byentity_kinematics_test.cpp`, `test_byentity_conditions.py` | Kernel landed: spec formula `speed_rel = speed(trig) − speed(ref)` (signed); directional projection in the triggering frame via det_sincos; absent reference ⇒ false. XML lowering deferred (P4) |
| SpeedCondition | §7.6.5.1 | In | p5-s2 | `condition_byentity_kinematics_test.cpp`, `test_byentity_conditions.py` | Kernel landed: total = \ | speed\ | , optional directional projection. XML lowering deferred (P4) |
| StandStillCondition | §7.6.5.1 | In | p5-s2 | `condition_byentity_kinematics_test.cpp`, `test_byentity_conditions.py` | Kernel landed: contiguous time at speed exactly 0.0 (no invented ε), `>=` duration. XML lowering deferred (P4) |
| TimeHeadwayCondition | §7.6.5.1, §6.4 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | Kernel landed: distance ÷ the triggering entity's speed only (reference leading); stopped/reversing follower ⇒ false. Full distance matrix incl. freespace; deprecated `alongRoute` warned and mapped; road CS deferred false (p3-s4). XML lowering deferred (P4) |
| TimeToCollisionCondition | §7.6.5.1, §6.4 | In | p5-s3 | `condition_byentity_interaction_test.cpp`, `test_interaction_conditions.py` | Kernel landed: distance ÷ closing speed (no acceleration), entity XOR position target; diverging/zero closing speed or coincident points ⇒ false. Road CS deferred false (p3-s4). XML lowering deferred (P4) |
| TraveledDistanceCondition | §7.6.5.1 | In | p5-s2 | `condition_byentity_kinematics_test.cpp`, `test_byentity_conditions.py` | Kernel landed: cumulative world-frame path length from init (not displacement), `>=` value. XML lowering deferred (P4) |

## Conditions — by value

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| ParameterCondition | §7.6.5.2 | In | p5-s1 | `condition_byvalue_test.cpp`, `test_byvalue_conditions.py` | Kernel landed: shared `Rule` comparator, exact IEEE numeric compare, ordering only when both operands are scalar-convertible; immutable at runtime (§9.1); dangling `parameterRef` ⇒ init `SemanticError`. XML lowering deferred (P4) |
| VariableCondition | §7.6.5.2, §6.12 | In | p5-s1 | `condition_byvalue_test.cpp`, `test_byvalue_conditions.py` | Kernel landed (≥1.2): runtime variable store seeded at init, host `set_variable`; strings equal/not-equal, numeric when scalar-convertible; dangling `variableRef` ⇒ init `SemanticError` (rule `reference_control.resolvable_variable_reference`). XML lowering deferred (P4) |
| SimulationTimeCondition | §7.6.5.2 | In | p5-s1 | `condition_byvalue_test.cpp`, `test_byvalue_conditions.py` | Kernel landed: gained the spec's `rule` (default greaterOrEqual); time starts when the Storyboard enters running (§8.4.7); NaN value ⇒ init `ValidationError`. XML lowering deferred (P4) |
| StoryboardElementStateCondition | §7.6.5.2, §8.1–8.2 | In | p5-s1 | `condition_byvalue_test.cpp`, `test_byvalue_conditions.py` | Kernel landed: level states + one-evaluation transition pulses; nameRef `::` resolution; zero/ambiguous ref ⇒ init `SemanticError` (rule `reference_control.resolvable_storyboard_element_ref`); `action` type unsupported until p5-s4 (warns). XML lowering deferred (P4) |
| TimeOfDayCondition | §7.6.5.2 | In | p5-s1 | `condition_byvalue_test.cpp`, `test_byvalue_conditions.py` | Kernel landed: host-anchored simulated clock advancing with sim time, epoch-seconds compare (UTC offset honored), leap-year-correct; unset anchor ⇒ false + warn-once. Same anchor fed later by the EnvironmentAction clock (p5-s6). XML lowering deferred (P4) |
| TrafficSignalCondition | §7.6.5.2, §6.11 | In | p5-s6 | `traffic_signal_test.cpp`, `test_traffic_signals.py` | Kernel landed: a level predicate on the byte-for-byte signal state ("reaches" comes from conditionEdge rising, the p5-s1 catalog precedent); an unwritten signal id is a deterministic false + warn-once. Road-network id validation (rule C.7.10) → p3-s4. XML lowering deferred (P4) |
| TrafficSignalControllerCondition | §7.6.5.2, §6.11 | In | p5-s6 | `traffic_signal_test.cpp`, `test_traffic_signals.py` | Kernel landed: a level predicate on the controller's phase name; a controller before its §6.11.3 delayed start has no phase, so the condition is a deterministic false. Both references validated at init (rule ...traffic_signal_controller_condition_references, C.7.12). 1.4 semantics excluded. XML lowering deferred (P4) |
| UserDefinedValueCondition | §7.6.5.2 | In | p5-s1 | `condition_byvalue_test.cpp`, `test_byvalue_conditions.py` | Kernel landed: host-provided named values (`set_user_defined_value`, stageable pre-init); unset name ⇒ false + warn-once. XML lowering deferred (P4) |

## Storyboard & runtime semantics

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| Storyboard / Story / Act / ManeuverGroup / Maneuver / Event / Action nesting | §7.2.1, §7.3 | In | p1-s1 | `storyboard_test.cpp` |  |
| Element state machine (standby/running/complete; start/end/stop/skip transitions) | §8.1–8.3 | In | p1-s1, p1-s3 | `storyboard_test.cpp`, `event_priority_test.cpp` | Landed: skipTransition in p1-s3, emitted for both of its meanings (§8.2 priority skip and §8.4.2.1 exhausted execution count), told apart by the resulting state |
| Init phase (global + private init actions before simulation time) | §8.5 | In | p1-s1, p4-s2 | `storyboard_test.cpp`; p4-s2 part planned | Non-instantaneous init actions carry into the storyboard |
| Trigger = OR(ConditionGroups); group = AND(Conditions); empty trigger false | §7.6.1 | In | p1-s2 | `trigger_test.cpp` | Landed: absent trigger (`nullopt`) distinct from empty (always false); empty ConditionGroup rejected at init (1..\* cardinality); no short-circuit evaluation |
| ConditionEdge none/rising/falling/risingOrFalling + first-evaluation corner cases | §7.6.2, §7.6.4 | In | p1-s2 | `trigger_test.cpp` | Landed: per-condition history; first check of an edge condition false, starting when the enclosing element enters standby |
| Condition delay (evaluates state of t−Δt; Δt ≥ 0) | §7.6.3 | In | p1-s2 | `trigger_test.cpp` | Rule-cited validation. Landed: delay applies post-edge (per `Condition.delay` class reference over §7.6.3 prose); sample-and-hold lookup of the most recent evaluation at or before t−Δt, exact comparisons |
| Start triggers (Act, Event only), stop triggers (Storyboard, Act) + inheritance | §7.6.1.1–7.6.1.2 | In | p1-s2 | `trigger_test.cpp` | Landed: stop checked before start at every level; stop applies from standby and running; execution-count clearing hooked in `stop_cascade` for p1-s3 |
| Event priority `override`/`skip`/`parallel` | §7.3.2, §8.4.2.2 | In | p1-s3 | `event_priority_test.cpp`, `test_event_lifecycle.py` | Landed: scope is the Maneuver (§7.3.3); `override` stops running siblings only, clearing their remaining executions, through the same path as a stop trigger; `skip` is conditional on a running sibling (`Priority` class reference, which reconciles §7.3.2 with §8.4.2.2) and its skipTransition counts as an execution; priority applies to trigger-less events via the Act-inherited trigger (§8.4.2). Simultaneous triggers resolved in document order in a single pass — the standard gives no rule. Deprecated literal `overwrite` is a lexical synonym with no separate IR value; the frontend maps it at load time (p4-s2), which also applies the XSD default |
| maximumExecutionCount (Event) | §8.3.3.2, §8.4.2.1 | In | p1-s3 | `event_priority_test.cpp` | Landed: executions = startTransitions + skipTransitions, spent sequentially; an endTransition re-arms to standby while executions remain and completes the event once exhausted; a standby event at its maximum completes with a skipTransition; condition history is *not* reset on re-arm, so an edge trigger must rise again; `0` accepted (schema-valid, §8.4.2.1 gives it a coherent reading) and negative rejected at init; stop triggers and `override` clear the remaining budget. A trigger-less event executes once — its Act-inherited trigger has already fired (§8.4.2) |
| maximumExecutionCount (ManeuverGroup) | §8.3.3.2, §8.4.4 | In | p4-s2 | — planned (p4-s2, #52) | Deferred from p1-s3 (ADR-0005, issue #52): §8.4.4 re-arms the group onto the Act-inherited start trigger, which is not re-evaluated while the act runs, so any behaviour requires an invented restart rule with determinism consequences — an ADR-level decision that belongs with the sprint loading the attribute |
| Action lifetime (instantaneous vs never-ending) | §7.4.1.2, §7.5.3 | In | p1-s3, p5-s4 | `event_priority_test.cpp`, `longitudinal_test.cpp` | Landed: two-valued action outcome reported by the applier — complete on application, or ongoing and ended only by a stopTransition. Every shipped action is the §7.4.1.2 step-dynamic case. Finite-duration actions governed by transition dynamics arrive as an additive third outcome with p2-s2/p5-s4 (ADR-0005) |
| Action conflict resolution & completion reasons | §7.5 | In | p5-s4 | `action_distance_test.cpp`, `action_routing_test.cpp` (minimal supersession); full §7.5 planned (#51) | Deferred from p1-s3 (ADR-0005, issue #51): §7.5.1 conflicts require two actions competing for a domain, and every shipped action is the §7.4.1.2 step-dynamic case that assigns no control strategy, so the machinery would be unreachable until p2-s2. Covers continuous actions (§7.5.3), bulk/actor semantics (§7.5.4, §8.3.3.3) and stop-notification of ongoing actions (§7.5.2.1) |
| Storyboard stop trigger as sole completion | §8.4.7 | In | p1-s1 | `storyboard_test.cpp` |  |

## Document & reuse machinery

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| OpenScenario root / FileHeader (revMajor/revMinor 1.0–1.3) | class ref | In | p4-s1 | — planned (p4-s1) | 1.4 rejected with diagnostic |
| ScenarioDefinition (incl. RoadNetwork reference) | class ref | In | p4-s2 | — planned (p4-s2) | RoadNetwork logic file handed to road backend/host |
| ParameterDeclaration + ValueConstraint(Group) | §9.1 | In | p4-s3 | — planned (p4-s3) |  |
| Expressions `$param`, `${...}` | §9.2 | In | p4-s3 | — planned (p4-s3) | ≥1.1; full operator whitelist + typing rules; 1.4-only constant `pi` rejected |
| VariableDeclaration | §6.12 | In | p4-s3 | — planned (p4-s3); runtime store already covered by `condition_byvalue_test.cpp` | ≥1.2; runtime store in kernel |
| MonitorDeclaration | §6.14 | Post | — | — | With SetMonitorAction *(verify version)* |
| Catalog / CatalogReference / CatalogLocations (vehicle, controller, pedestrian, miscObject, environment, maneuver, trajectory, route) | §9.4–9.6 | In | p4-s4 | — planned (p4-s4) | 1.4-only trafficDistributionEntry catalog excluded |
| ParameterValueDistribution files (Deterministic/Stochastic) | §9.3 | Post | — | — | Parameter-variation workflow belongs with the post-release generation family |
| Entities: ScenarioObject + inline Vehicle/Pedestrian/MiscObject | §7.2.2 | In | p2-s1, p4-s2 | `entity_model_test.cpp`, `entity_types_test.cpp`, `test_entity_model.py` | IR + bindings landed (p2-s1): ObjectType/category/Role enums (full 1.4 sets, 1.3→1.4 deprecations noted), `Performance`, axles and properties as data, bounding boxes, EntityObject variant + `object_type_of`/`bounding_box_of`/`performance_of`; full h/p/r `EntityState`; C-ABI typed builders + metadata accessors; Performance range validation (ADR-0010). XML lowering → p4-s2 |
| ExternalObjectReference | §7.2.2 | Post | — | — | Requires road-network object binding beyond v0.0.1 map scope |
| Trailer attributes on Vehicle | §7.2.2.6 | Post | — | — | With the trailer action family |
| EntitySelection / ByType / ByObjectType | §7.2.2.2–7.2.2.5 | In | p4-s4 | — planned (p4-s4) | Homogeneity rules cited |
| ObjectController / Controller (+ properties) | §6.6 | In | p4-s4, p5-s5 | `action_controller_visibility_test.cpp` (assignment); catalogs planned (p4-s4) | Controller IR (name, controllerType, ordered properties) + assignment and gateway hand-off landed p5-s5; ObjectController wrapper, catalogs and multiple controllers (≥1.2) → p4-s4 |
| Environment / Weather / TimeOfDay / RoadCondition (as data) | §7.4.2 | In | p4-s4, p5-s6 | `action_global_test.cpp` (kernel store); catalogs planned (p4-s4) | Kernel value types landed (p5-s6): stored, merged member-wise and queryable via `Engine::environment`; no physics coupling. Catalog instantiation + parameterDeclarations → p4-s4 |
| TrafficDefinition / TrafficDistribution | §6.10 | Post | — | — | With the traffic family |

## Positions, trajectories, routes

| Element | Section | Status | Sprint | Tests | Notes |
|---|---|---|---|---|---|
| WorldPosition | §6.3.8 | In | p2-s4 | `position_test.cpp`, `action_teleport_test.cpp` | Resolved directly (x/y/z + world-frame h/p/r) via the `PositionResolver`. ≥1.3 corrected calculations applied to all versions (per §5) |
| RelativeWorldPosition | §6.3.8 | In | p2-s4 | `position_test.cpp`, `action_teleport_test.cpp`, `test_positions.py` | Kernel landed: world-axis deltas from a reference entity (not rotated), with §Orientation absolute/relative composition (ADR-0017) |
| RelativeObjectPosition | §6.3.8 | In | p2-s4 | `position_test.cpp`, `action_routing_test.cpp`, `test_positions.py` | Kernel landed: deltas in the reference entity's local frame, rotated by its heading via `det_sincos` (yaw-only while the runtime keeps pitch/roll = 0); orientation composition (ADR-0017) |
| RoadPosition / RelativeRoadPosition | §6.3.8 | In | p2-s4, p3-s4 | `position_test.cpp` (reported unsupported); resolution planned (p3-s4) | IR + resolver dispatch landed (p2-s4); road-network resolution and s-axis tangent for orientation → p3-s4 (reported `UnsupportedFeature` until then, never silently wrong) |
| LanePosition / RelativeLanePosition | §6.3.8 | In | p2-s4, p3-s4 | `position_test.cpp` (reported unsupported); resolution planned (p3-s4) | IR + resolver dispatch landed (p2-s4); resolution via `IRoadQuery` → p3-s4 |
| RoutePosition (all InRoutePosition variants) | §6.3.8 | In | p2-s4, p3-s4 | `position_test.cpp` (reported unsupported); resolution planned (p3-s4) | IR + resolver dispatch landed (p2-s4); resolution needs a resolved route → p3-s4 |
| TrajectoryPosition | §6.3.8 | In | p2-s5 | `position_test.cpp` | Resolves (p2-s5, ADR-0018): the referenced trajectory is evaluated at arc length `s` and the lateral offset `t` is stepped along the tangent left-normal. Road-projected trajectory lateral distance (§6.4.6) still needs the road backend (p3-s4) |
| GeoPosition | §6.3.8 | Post | — | `position_test.cpp`, `test_positions.py` (reported) | Reported `UnsupportedFeature` with rule `asam.net:xosc:1.1.0:positioning.geodetic_datum_defined` (p2-s4); needs geodetic datum handling → post-v0.0.1 |
| Orientation + ReferenceContext | §6.3.8 | In | p2-s4 | `position_test.cpp`, `test_positions.py` | Kernel landed: h/p/r + ReferenceContext, absolute/relative composition against the resolved base pose (ADR-0017) |
| Trajectory + Polyline / Clothoid / Nurbs | §6.9 | In | p2-s5, p5-s5 | `trajectory_test.cpp`, `action_routing_test.cpp`, `test_trajectory.py` | All three shapes land (ADR-0018): Polyline (exact segment interpolation, p5-s5); Clothoid (Euler spiral — closed-form line/arc, general spiral by deterministic composite-Simpson quadrature of the Fresnel-type integrand via `det_sincos`, 1e-9 to the analytic circle); NURBS (rational de Boor, IEEE-only, exact quarter circle to 1e-9; arc length by a fixed-resolution table). A `closed` trajectory is accepted with an UnsupportedFeature warning and followed as an open path |
| ClothoidSpline | §6.9 | Post | — | — | Introduction version unconfirmed *(verify)*; single-segment Clothoid covers the v0.0.1 need (ADR-0018) |
| Motion class / Polyline Interpolation | §6.9 | Excl | — | — | 1.4-only (ADR-0018) |
| TimeReference / Timing / TrajectoryFollowingMode | §6.9.1–6.9.5 | In | p2-s5, p5-s5 | `action_routing_test.cpp`, `trajectory_test.cpp` | TimeReference none/timing with ReferenceContext, scale and offset (p5-s5), incl. the §6.9.1–§6.9.3 start-state cases; for clothoid/NURBS the clock maps linearly across the shape's start/end times to arc length (p2-s5). `followingMode=follow` is accepted and executed as `position` with a warning until a steering controller exists (deferred, see ADR-0018) |
| Route / Waypoint / RouteStrategy / RouteRef | §6.8 | In | p3-s3, p5-s5 | `action_routing_test.cpp` (assignment); road semantics planned (p3-s3, p3-s4) | Route/Waypoint/RouteStrategy IR + per-entity assignment landed p5-s5 (world-frame waypoints; the strategy is stored, never interpreted, so `random` reaches no random generator). Road-network path selection and RouteRef → p3-s3/p3-s4 |
| TrafficSignalController / Phase / TrafficSignalState | §6.11 | In | p5-s6 | `traffic_signal_test.cpp`, `test_traffic_signals.py` | Kernel landed: the ordered cycle with an arithmetic (fmod) phase clock that cannot drift with the host's step pattern, half-open phase intervals, transitive §6.11.3 delay chains, and write-on-transition semantics; validated at init for unique names, non-negative durations (rule C.2.3) and acyclic resolvable references (C.7.13). 1.4 TrafficSignalSemantics / TrafficSignalGroupState excluded. XML lowering deferred (P4) |

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
