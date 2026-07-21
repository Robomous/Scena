# ASAM OpenSCENARIO DSL 2.x — v0.0.1 coverage matrix

Normative scope declaration for the DSL frontend (P7) and DSL execution
(P8). Section numbers follow the ASAM OpenSCENARIO DSL 2.2.0 text (the
local reference copy; 2.2.0 is the current published 2.x version). The DSL
spec defines no `asam.net:` rule IDs, so diagnostics cite section numbers.

Two capability columns, because the DSL pillar split makes them distinct:

- **Check** (P7): parsed, resolved, and type-checked — `scena-check`
  accepts/rejects correctly with §-cited diagnostics.
- **Exec** (P8): executable for **attribute-level concrete scenarios**
  (§6.3.1.2.1 — all attribute values fixed; the engine synthesizes
  motion). The spec's semantics are trace-acceptance based (§7.6.1); a
  deterministic executor is one conformant operational choice.

Statuses: **In** (v0.0.1, tested), **Post** (post-v0.0.1; structured
`UnsupportedFeature` diagnostic when encountered at that layer), **Excl**
(excluded with reason), **n/a** (nothing to execute — a static-only
construct).

Headline honesty rule: **"concrete scenarios only" excludes** — range and
distribution *selection*, `keep()` constraints requiring search (anything
but fixed-value resolution), coverage-driven generation, and external
methods. All of these still **Check** cleanly; execution diagnoses them.

## Language core (§7.2, §7.3)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| Lexical structure (indentation blocks, comments, continuation, escaped identifiers) | §7.2.1 | In | n/a | p7-s1 | Offside rule → INDENT/DEDENT |
| Literals (int/uint/hex/float/bool/string/physical) | §7.2.1.5.2 | In | In | p7-s1, p8-s1 | Physical literal = number + mandatory unit |
| Physical types + units, SI dimensions, conversion factors/offsets | §7.2.2.2.1, §7.3.4 | In | In | p7-s3, p8-s1 | Dimension checking; global unit namespace |
| Enums (incl. `enum!member`, extension) | §7.3.3 | In | In | p7-s3, p8-s1 | |
| Primitive types (bool, int, uint, float, string) | §7.3.2 | In | In | p7-s3 | IEEE 754 float; 64-bit int/uint |
| Structs | §7.3.5.1.1 | In | In | p7-s3, p8-s1 | |
| Actors | §7.3.5.1.2 | In | In | p7-s3, p8-s1 | |
| Scenarios (assoc. actor, ≤1 `do`, n `on`) | §7.3.5.1.3 | In | In | p7-s3, p8-s1 | |
| Actions (atomic behaviors) | §7.3.5.1.4, §7.6.3 | In | In (matrix subset) | p7-s3, p8-s1 | Action internals are implementation-defined per §7.6.3; see §8.8 table |
| Modifier declarations + application | §7.3.12 | In | In (matrix subset) | p7-s3, p8-s3 | Argument = equality constraint (§7.3.12.4) |
| `override()` atomic modifier | §7.3.12.1.1 | In | Post | p7-s3 | Maps to XML override/skip machinery; wiring deferred with a documented diagnostic |
| Lists (+ list operators) | §7.3.5.2.1, §7.4.2.7 | In | In | p7-s4 | |
| Ranges (`[a..b]`, range operators) | §7.3.5.2.2, §7.4.2.8 | In | Partial → see keep/ranges row | p7-s4 | |
| Fields: parameters vs `var` variables, `with:` blocks | §7.3.6 | In | In | p7-s3, p8-s1 | Parameters fixed at init; variables runtime-mutable |
| `sample()` variable initializer | §7.3.10.4 | In | Post | p7-s4 | Event-sampled variables deferred (needs event-valuation plumbing beyond v0.0.1 set) |
| `keep(hard/default)` constraints | §7.3.11 | In | **Concrete-value only** | p7-s4, p8-s1 | Fixed-value/equality resolution at init; violated hard keep = error; anything requiring search → diagnostic (ADR-0004) |
| `remove_default()` | §7.3.11 | In | In (within concrete resolution) | p7-s4 | |
| Methods (expression, `undefined`, override rules) | §7.3.7 | In | In (expression methods in constant contexts) | p7-s3 | |
| External methods (`is external ...`) | §7.3.7.4 | In (parse/check) | Post | p7-s4 | Host-binding FFI is post-release; invocation diagnosed |
| Inheritance (single; conditional `inherits X(f == v)`) | §7.3.8 | In | In | p7-s3 | Latent subtypes via `is()`/`as()` |
| Extension (`extend`) | §7.3.9 | In | In | p7-s3 | Compile-time composition |
| Events (`event`, predefined start/end/fail) | §7.3.10 | In | In | p7-s3, p8-s2 | Predefined events map to storyboard element transitions |
| Event specifications (`@`, `rise`, `fall`, `elapsed`, `every`, `if`, `as` binding) | §7.3.10.4 | In | In except `every` | p7-s2, p8-s2 | `every` (periodic) Post: no v0.0.1 scenario needs it; deterministic period machinery deferred |
| Global parameters | §7.3.14 | In | In | p7-s3 | |
| Type resolution order (declare-anywhere) | §7.3.15 | In | n/a | p7-s3 | Multi-pass resolution |
| Namespaces + `::`, export rules | §7.7.4 | In | n/a | p7-s3 | |
| Import (URI + identifier forms; `osc.standard.all/types/domain`, legacy `osc.standard`) | §7.7.5 | In | n/a | p7-s2 | Dedup per spec |
| Scenario entry-point selection | §7.7.2 | n/a | In | p8-s1 | Implementation-defined per spec: qualified name via API/CLI |

## Expressions (§7.4)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| Atomic (identifiers, literals, `it`) | §7.4.1 | In | In | p7-s4 | |
| Logical (short-circuit), arithmetic (+ numeric conversion rules), relational + `in`, ternary, `=>` | §7.4.2.2–.4, .9 | In | In | p7-s4 | Physical types never implicitly converted |
| `is()` / `as()` | §7.4.2.5–.6 | In | In | p7-s4 | Cast failure = error |
| Method application | §7.4.2.1 | In | In (constant contexts + In-scope library methods) | p7-s4 | |
| List/range operators | §7.4.2.7–.8 | In | In | p7-s4 | |

## Coverage constructs (§7.5)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| `cover()` / `record()` / cross / args / override / grading | §7.5 | In | Post | p7-s4 | Checked; collection not performed in v0.0.1 (explicit diagnostic note). `target`/grading exist to steer generation — post-release with the solver family |

## Behavior composition & semantics (§7.6)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| Direct invocation in `do` | §7.6.2.1.1 | In | In | p7-s2, p8-s2 | |
| `serial` | §7.6.2.1.2 | In | In | p8-s2 | Member starts when predecessor ends |
| `parallel` (default `start` overlap; duration) | §7.6.2.1.4 | In | In | p8-s2 | All-members-complete join |
| `parallel` non-default overlap kinds + `start_to_start`/`end_to_end` offsets | §7.6.2.1.4 | In | Post | p8-s2 | Declared honestly: only default overlap executes in v0.0.1 |
| `one_of` | §7.6.2.1.3 | In | In | p8-s2 | Deterministic host-selected alternative (default: first) — any single branch is a valid acceptance; randomness barred by the determinism contract. The exact operator set in 2.2.0 is serial/parallel/one_of — there is no `first_of` or `mix` |
| Labels on do-members | §7.2.2.4.7 | In | In | p7-s2, p8-s2 | |
| `duration` bounds on compositions/invocations | §7.6.2.4 | In | In | p8-s2 | |
| `until` | §7.6.2.5.4 | In | In | p8-s2 | Terminates the annotated invocation exactly at the event; first-of-any for multiples |
| `wait` | §7.6.2.5.3 | In | In | p8-s2 | Pure synchronization |
| `emit` | §7.6.2.5.2 | In | In | p8-s2 | Zero-time; eager parameter evaluation |
| `on` directives (call/emit reactions) | §7.6.2.5.5 | In | In | p8-s2 | |
| `call` | §7.2.2.4.7 | In | In (In-scope methods) | p8-s2 | |
| Actor-binding rules for scenario parameters | §7.6.2.2 | In | In | p8-s1 | Constant binding over invocation lifetime |
| Constraint sets over invocation lifetime | §7.6.2.3 | In | Concrete-value only | p8-s1 | See keep row |

## Standard library (§8) — checking

The entire §8 library (namespaces `stdtypes` + `std`) **type-checks** in
v0.0.1 — that is the P7 pillar gate (p7-s5): all physical types and units
(§8.14.1), compound structs (§8.14.2), string methods (§8.13), all actors
(§8.7: osc_actor, physical_object, stationary_object, movable_object,
traffic_participant, vehicle, trailer, person, animal, vehicle_group,
environment, map, traffic_light_controller), all actions (§8.8, §8.11,
§8.15.9), all modifiers (§8.9), and the map/route/environment/traffic-light
type families (§8.10, §8.12, §8.15). "Check = In" is therefore implicit for
every §8 row below; the Exec column is the declared execution scope.

### Actors — execution

| Actor | Section | Exec | Sprint | Notes |
|---|---|---|---|---|
| vehicle (+ vehicle_category) | §8.7.7 | In | p8-s1 | → IR vehicle taxonomy |
| person | §8.7.9 | In | p8-s1 | → IR pedestrian |
| stationary_object / movable_object base data | §8.7.4–.5 | In | p8-s1 | → IR misc object / base fields |
| traffic_participant metric methods (time_to_collision, time_headway, time_gap, space_gap, space_headway) | §8.7.6.1 | In | p8-s2 | Backed by the P5 measurement machinery |
| physical_object distance methods (object_distance, road_distance, get_s_coord, get_t_coord, …) | §8.7.3.1 | In | p8-s2 | Backed by P3 `IRoadQuery` |
| animal | §8.7.10 | Post | — | No dedicated motion model in v0.0.1 |
| trailer (+ hitch structs) | §8.7.8 | Post | — | Trailer model out of entity scope (matches XML decision) |
| vehicle_group | §8.7.26.1 | Post | — | Group orchestration deferred |
| environment (weather/air/precipitation/wind/fog/clouds structs) | §8.10 | Post | — | State-only in XML path; DSL environment actions deferred |
| map actor: set_map_file, create_route/point/path/trajectory, conversions | §8.12.2 | In (listed methods) | p8-s1 | `set_map_file` binds the P3 backend; creation/conversion methods over `IRoadQuery` |
| map search-space modifiers (number_of_lanes, routes_are_in_sequence, …) | §8.12.2.2 | Post | — | Abstract map-matching (search-space binding) belongs to the solver family; `set_map_file` + concrete routes cover concrete execution |
| traffic_light_controller | §8.15.8.1 | Post | — | DSL signal control deferred; XML signal actions cover v0.0.1 (kernel phase model is shared groundwork) |

### Actions — execution (§8.8, §8.11, §8.15.9)

| Action | Section | Exec | Sprint | Notes |
|---|---|---|---|---|
| move (generic, modifier-shaped) | §8.8.2.3 | In | p8-s1/p8-s3 | The modifier carrier |
| assign_position / assign_orientation / assign_speed | §8.8.2.4–.6 | In | p8-s1 | Teleport-family → IR |
| change_position / change_speed / keep_speed | §8.8.2.11–.13 | In | p8-s1 | → P2/P5 longitudinal machinery |
| assign_acceleration / change_acceleration / keep_acceleration | §8.8.2.7, .14–.15 | Post | — | Direct acceleration-target actions deferred (speed-target machinery covers the golden set) |
| follow_path / follow_trajectory | §8.8.2.16–.17 | In | p8-s1 | → P2-s5 trajectory machinery |
| replay_path / replay_trajectory (+ relative variants) | §8.8.2.8–.9 | Post | — | Timestamped replay deferred; overlaps host-controlled replay capability |
| remain_stationary | §8.8.2.10 | In | p8-s1 | Trivial on the runtime |
| drive (generic vehicle action) | §8.8.3.1 | In | p8-s1/p8-s3 | The GS-12 workhorse |
| follow_lane / change_lane | §8.8.3.2–.3 | In | p8-s1 | → lane machinery |
| change/keep_time_gap, change/keep_space_gap | §8.8.3.4–.7 | In | p8-s1 | → LongitudinalDistanceAction machinery |
| change/keep_time_headway, change/keep_space_headway | §8.8.3.8–.11 | Post | — | Headway-hold variants deferred; gap family covers the golden set |
| connect_trailer / disconnect_trailer | §8.8.3.12–.13 | Post | — | With trailers |
| walk | §8.8.4.1 | In | p8-s1 | → pedestrian motion |
| environment actions (air, rain, snow, wind, fog, clouds, assign_celestial_position) | §8.11 | Post | — | With the environment family |
| traffic-light actions (set_bulb_state … play_cycles) | §8.15.9 | Post | — | With DSL signal control |

### Movement modifiers — execution (§8.9)

Common machinery: `at` phase anchoring (§8.9.19) In; `movement_mode`,
`track`, shape structs (§8.9.1.2) Post (default profiles only —
documented); scalar params In, `*_range` params Post (range-valued
envelopes belong to the logical level).

| Modifier | Section | Exec | Sprint | Notes |
|---|---|---|---|---|
| position() / keep_position() | §8.9.2–.3 | In | p8-s3 | ahead_of/behind + distance forms; `*_range`/at_point-with-projection variants Post |
| speed() / change_speed() / keep_speed() | §8.9.4–.6 | In | p8-s3 | Absolute + faster_than/slower_than/same_as relative forms |
| acceleration() | §8.9.7 | In | p8-s3 | Scalar form |
| lateral() | §8.9.8 | In | p8-s3 | t-axis offset; measure_by default |
| lane() / change_lane() / keep_lane() | §8.9.14–.16 | In | p8-s3 | `change_lane` without an explicit side is diagnosed, not randomized (determinism) |
| along() / along_trajectory() | §8.9.11–.12 | In | p8-s3 | Route/path/trajectory binding |
| distance() | §8.9.13 | In | p8-s3 | Traveled-distance bound |
| yaw() / orientation() | §8.9.9–.10 | Post | — | Orientation-target modifiers deferred (teleport orientation covers placements) |
| physical_movement() | §8.9.17 | Post | — | Single documented default profile in v0.0.1 |
| avoid_collisions() | §8.9.18 | Post | — | No collision-avoidance controller in v0.0.1 (engine executes what the scenario says; Collision condition detects) |

## What "concrete scenarios only" excludes (summary)

Checked but not executed in v0.0.1, all with structured diagnostics:
range/distribution *selection* (logical scenarios, §6.3.1.2.2), `keep()`
requiring search, coverage collection and coverage-driven generation
(§7.5), abstract map-matching modifiers (§8.12.2.2), external methods
(§7.3.7.4 — note Annex C shows all distributions are built on external
methods, so distribution workflows are inherently post-release),
`sample()`/`every` event machinery, non-default parallel overlap kinds.
Constraint solving and abstract-scenario generation are post-v0.0.1 and
feasibility-gated (ADR-0004).

## Declared coverage summary (DSL 2.2.0)

- **Check (P7): 100 %** of the language (§7) and standard library (§8) —
  the full-library type-check is the p7-s5 pillar gate.
- Exec, language/composition constructs: **serial, parallel (default
  overlap), one_of, until, wait, emit, on, call, duration, labels, events**
  In — 11 of 14 behavior constructs (79%); `every`, non-default overlaps,
  `override()` modifier Post.
- Exec, §8.8 movement actions: **13 of 24** In (54%); the deferred 11 are
  acceleration-target, replay, headway-hold, trailer, environment, and
  signal families.
- Exec, §8.9 movement modifiers: **13 of 18** In (72%); scalar forms only.
