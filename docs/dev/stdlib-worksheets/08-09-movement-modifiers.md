# §8.9 — movement modifiers

Source: ASAM OpenSCENARIO DSL 2.2.0, `domain-model` §8.9 (Tables 147–153, plus
Code 78–120). Landed in slice 43g. Target chunk: `kDomainMovementModifiers` in
`frontends/dsl/src/stdlib.cpp`.

§8.9 is presented differently from the rest of §8: each modifier gets a *usage
signature* and a prose parameter list rather than a parameter table, and the
shape hierarchy is printed as DSL. Only the seven enums are tables.

Unlike §8.8, stripping the `Examples` blocks does not help here — the examples
are inline in each section. The whole chapter has to be read.

## The four common parameters (§8.9.1.1)

> The following parameters are common to all domain model movement modifiers.

| Parameter | Type | § |
| --- | --- | --- |
| `at` | `at` | 8.9.1.1.1 |
| `movement_mode` | `movement_mode` | 8.9.1.1.2 |
| `track` | `track` | 8.9.1.1.3 |
| `shape` | `any_shape` | 8.9.1.1.4 |

Every modifier below carries all four. ⚠ Five usage signatures
(`keep_position`, `keep_speed`, `keep_lane`, `physical_movement`,
`avoid_collisions`) omit the `<standard-movement-parameters>` placeholder that
the other twelve include. §8.9.1.1's "common to all" is the general normative
statement and the usage blocks are illustrative of a typical call, so all
seventeen get them — and over-accepting is the safer error here, since the
alternative rejects a scenario the standard permits.

## Shape hierarchy (§8.9.1.2, printed as DSL)

| Struct | Base | Members |
| --- | --- | --- |
| `any_shape` | — | `def duration() -> time` |
| `any_acceleration_shape` | `any_shape` | `def compute(time: time) -> acceleration` |
| `any_speed_shape` | `any_shape` | `def compute(time: time) -> speed` |
| `any_position_shape` | `any_shape` | `def compute(time: time) -> length` |
| `any_lateral_shape` | `any_shape` | `def compute(time: time) -> length` |
| `common_acceleration_shape` | `any_acceleration_shape` | `rate_profile: dynamic_profile`, `rate_peak: jerk`, `target: acceleration` |
| `common_speed_shape` | `any_speed_shape` | `rate_profile`, `rate_peak: acceleration`, `target: speed` |
| `common_position_shape` | `any_position_shape` | `rate_profile`, `rate_peak: speed`, `target: length` |
| `common_lateral_shape` | `any_lateral_shape` | `rate_profile`, `rate_peak: speed`, `target: length` |

## Enums (§8.9.19–§8.9.25)

| § | Enum | Members | ⚠ |
| --- | --- | --- | --- |
| .19 | `at` | `start, end, all` | a field named `at` of type `at` on every modifier |
| .20 | `movement_mode` | `monotonous, other` | `other` is in ~six other enums |
| .21 | `track` | `actual, projected` | |
| .22 | `lat_measure_by` | `left_to_left … right_to_right, closest` (10) | |
| .23 | `yaw_measure_by` | `length_to_length, length_to_width, width_to_length, width_to_width, relative_to_north, relative_to_road` | |
| .24 | `orientation_measured_by` | `absolute, relative_to_reference, relative_to_road` | `relative_to_road` shared with `yaw_measure_by` |
| .25 | `movement_options` | `prefer_physical, prefer_non_physical, must_be_physical` | |

## Modifiers (§8.9.2–§8.9.18)

Association per §7.3.12.3 — see note 1. `+4` marks the four common parameters.

| § | Modifier | Declared on | Own parameters |
| --- | --- | --- | --- |
| .2 | `position` | `movable_object` | `distance: length`, `time: time`, `distance_range: range of length`, `time_range: range of time`, `ahead_of: physical_object`, `behind: physical_object`, `ahead_of_point: position_3d`, `behind_point: position_3d`, `at_point: position_3d`, `project_on_route: bool` |
| .3 | `keep_position` | `movable_object` | — (+4) |
| .4 | `speed` | `movable_object` | `speed: speed`, `speed_range: range of speed`, `faster_than`/`slower_than`/`same_as: physical_object`, `factor: float`, `direction: lon_lat` |
| .5 | `change_speed` | **unassociated** | `speed: speed`, `speed_range: range of speed` |
| .6 | `keep_speed` | **unassociated** | — (+4) |
| .7 | `acceleration` | `movable_object` | `acceleration: acceleration`, `acceleration_range: range of acceleration` |
| .8 | `lateral` | `movable_object` | `distance: length`, `distance_range: range of length`, `side_of: vehicle`, `side: side_left_right`, `measure_by: lat_measure_by` |
| .9 | `yaw` | `movable_object` | `angle: angle`, `angle_range: range of angle`, `relative_to: physical_object`, `measure_by: yaw_measure_by` |
| .10 | `orientation` | `movable_object` | `yaw: angle`, `pitch: angle`, `roll: angle`, `relative_to: physical_object`, `measure_by: orientation_measured_by` |
| .11 | `along` | `movable_object` | `route: route`, `start_offset: length`, `end_offset: length` |
| .12 | `along_trajectory` | `movable_object` | `trajectory: trajectory`, `start_offset: length`, `end_offset: length` |
| .13 | `distance` | `movable_object` | `distance: length` |
| .14 | `lane` | `vehicle` | `lane: uint`, `side_of: physical_object`, `side: side_left_right`, `same_as: physical_object`, `from: side_left_right` |
| .15 | `change_lane` | **unassociated** | `lane: int`, `side: side_left_right` |
| .16 | `keep_lane` | `vehicle` | — (+4) |
| .17 | `physical_movement` | `movable_object` | `option: movement_options` |
| .18 | `avoid_collisions` | `movable_object` | `avoid: bool` |

## ⚠ Notes carried into the library

1. **These modifiers are actor-associated, and the language reference says so
   outright.** §7.3.12.3's example of an actor-associated modifier is
   `modifier vehicle.keep_lane()`, annotated *"keep_lane() is defined in the
   domain model (see §8.9.16)"* — §7.3 is telling us how §8.9 is declared.
   Each modifier goes on the most general actor that can execute the movement
   actions it tunes: `movable_object`, which owns `move()` and parents both
   `vehicle` and `person`, or `vehicle` for the lane-related ones (`lane`,
   `keep_lane`), which §7.3.12.3 places there explicitly.
2. **Three modifiers must be unassociated, because the standard collides with
   itself.** §8.8 declares actions `movable_object.change_speed`,
   `movable_object.keep_speed` and `vehicle.change_lane`; §8.9 declares
   modifiers of the same names for the same actors. A qualified behavior name
   (§7.2.2.2.5) identifies exactly one declaration, so the language cannot hold
   both. Declaring these three in §7.3.12.3's *unassociated* form is the only
   spelling that exists in the language and does not collide. The defect is in
   the standard, not in the translation, and the choice is recorded in the
   coverage matrix as well as here.
3. **Declaring them associated is also what avoids two further collisions.**
   An unassociated `modifier lane` collides head-on with §8.12.10's
   `struct lane`, and an unassociated `modifier speed` shadows
   `stdtypes::speed` badly enough that `speed_range: range of speed` stops
   resolving to a physical type. Under `modifier vehicle.lane` and
   `modifier movable_object.speed` the declared names are qualified and neither
   problem arises. This is corroborating evidence for note 1: association is
   not decoration, it is what makes the chapter declarable.
4. **§8.9.14's `lane` counts lanes as `uint`, §8.9.15's `change_lane` as
   `int`.** Both carried as printed.
5. **Applying any of these is issue #100**, in both of §7.3.12.4.1's positions:
   as a scenario member, an actor-associated modifier is not found by simple
   name; inside a `with:` block, modifier applications are not validated at all
   (a nonsense name is accepted silently). The declarations here are
   well-formed and are pinned; nothing pins application, because neither
   current behaviour is one to keep. p8-s3 (#46) needs #100 fixed first, and
   §8.9 is the surface that fix should be tested against.
6. **§8.9.1.4's scalar/range pairs are two separate fields.** `speed` and
   `speed_range`, `distance` and `distance_range`, `angle` and `angle_range`:
   "at most one of them is used within an invocation", which is a constraint on
   applications rather than on the declaration. Same rule as §8.8's mutually
   exclusive parameters.
