# §8.8.2–§8.8.4 — movement actions

Source: ASAM OpenSCENARIO DSL 2.2.0, `domain-model` §8.8.2–§8.8.4
(Tables 88–146). Landed in slice 43f. Target chunk: `kDomainMovementActions` in
`frontends/dsl/src/stdlib.cpp`.

Twenty-nine actions in three families, four enums, and the two intermediate
bases the chapter's "Parents" rows name but §8.8.1 does not declare. Every
action is printed as a "Basic information" table (parents, controlled states,
action ending) plus, where it takes arguments, a parameter table. Controlled
states and action ending are runtime semantics with no place in a declaration;
only the parents and the parameters translate.

## The intermediate bases

§8.8.1 declares `osc_action` with exactly two children, `action_for_environment`
and `action_for_movable_object` (both landed in 43d). §8.8.3's and §8.8.4's
"Parents" rows then name `action_for_vehicle` and `action_for_person`, which no
table declares. §8.8.1's prose supplies them: "Actions for actors that are
children of `movable_object`, like `vehicle` or `person`, inherit from
`action_for_movable_object`."

```
action vehicle.action_for_vehicle inherits movable_object.action_for_movable_object
action person.action_for_person   inherits movable_object.action_for_movable_object
```

⚠ These two are the only declarations in the chapter not backed by a table of
their own. They are named as parents thirteen and one times respectively, so
leaving them out would make those thirteen actions unparentable.

## §8.8.2 actions for movable object (parent `action_for_movable_object`)

| § | Action | Fields | ⚠ |
| --- | --- | --- | --- |
| .3 | `move` | — | generic; §8.8.3's `drive` and §8.8.4's `walk` are its per-actor counterparts |
| .4 | `assign_position` | `position: position_3d`, `route_point: route_point`, `odr_point: odr_point` | "Use only one of the three"; all three are `Mandatory: no` |
| .5 | `assign_orientation` | `orientation: orientation_3d` | |
| .6 | `assign_speed` | `speed: speed` | |
| .7 | `assign_acceleration` | `acceleration: acceleration` | |
| .8 | `replay_path` | `absolute: path`, `relative: relative_path`, `reference: physical_object`, `transform: relative_transform`, `start_offset: length`, `end_offset: length` | `absolute` and `relative` are both `Mandatory: yes` yet mutually exclusive |
| .9 | `replay_trajectory` | same six, with `trajectory`/`relative_trajectory` | |
| .10 | `remain_stationary` | — | |
| .11 | `change_position` | `target_position: position`, `target_st: route_point`, `target_odr: odr_point`, `target_xyz: position_3d`, `interpolation: path_interpolation`, `on_road_network: bool` | `target_xyz` is marked deprecated in favour of `target_position`; carried, see note 3 |
| .12 | `change_speed` | `target: speed`, `rate_profile: dynamic_profile`, `rate_peak: acceleration` | |
| .13 | `keep_speed` | — | |
| .14 | `change_acceleration` | `target: acceleration`, `rate_profile: dynamic_profile`, `rate_peak: jerk` | |
| .15 | `keep_acceleration` | — | |
| .16 | `follow_path` | as `replay_path` | the target-behaviour counterpart of `replay_path` |
| .17 | `follow_trajectory` | as `replay_trajectory` | |
| .18 | enum `dynamic_profile` | `none, constant, smooth, asap` | `none` also in `hitch_type`, `directionality`, `stop_line_marking` |

## §8.8.3 actions for vehicle (parent `action_for_vehicle`)

| § | Action | Fields |
| --- | --- | --- |
| .1 | `drive` | — |
| .2 | `follow_lane` | `offset: length`, `rate_profile: dynamic_profile`, `rate_peak: speed`, `target: lane` |
| .3 | `change_lane` | `num_of_lanes: uint`, `side: lane_change_side`, `reference: physical_object`, `offset: length`, `rate_profile: dynamic_profile`, `rate_peak: speed`, `target: lane` |
| .4 | `change_time_gap` | `target: time`, `direction: gap_direction`, `reference: physical_object` |
| .5 | `keep_time_gap` | `reference: physical_object`, `direction: road_distance_direction` |
| .6 | `change_space_gap` | `target: length`, `direction: gap_direction`, `reference: physical_object` |
| .7 | `keep_space_gap` | `reference: physical_object`, `direction: road_distance_direction` |
| .8 | `change_time_headway` | `target: time`, `direction: headway_direction`, `reference: physical_object` |
| .9 | `keep_time_headway` | `reference: physical_object` |
| .10 | `change_space_headway` | `target: length`, `direction: headway_direction`, `reference: physical_object` |
| .11 | `keep_space_headway` | `reference: physical_object` |
| .12 | `connect_trailer` | `trailer: trailer` |
| .13 | `disconnect_trailer` | — |
| .14 | enum `lane_change_side` | `left, right, inside, outside, same` |
| .15 | enum `gap_direction` | `ahead, behind, left, right, inside, outside` |
| .16 | enum `headway_direction` | `ahead, behind` |

Note the asymmetry the standard chose deliberately: the `change_*` actions take
a `gap_direction` (six values, longitudinal *and* lateral), while the `keep_*`
actions take a `road_distance_direction` (§8.7.22, just `longitudinal` and
`lateral`). Carried as printed.

## §8.8.4 actions for person (parent `action_for_person`)

| § | Action | Fields |
| --- | --- | --- |
| .1 | `walk` | — |

§8.8.4's prose says a `person` **or an `animal`** walks, but the parent it names
is `action_for_person` and no `action_for_animal` exists. Declared on `person`
as printed; `animal` reaches `move` through `action_for_movable_object`.

## ⚠ Notes carried into the library

1. **Mutually exclusive parameters are still just fields.** `assign_position`
   says "use only one of the three"; `replay_path` marks both `absolute` and
   `relative` mandatory although only one can be given. The language has no
   choice-group construct, and §7.3.11 lets a scenario leave a field
   unconstrained, so all of them are plain fields. This is the same rule that
   made every table row a field in 43b.
2. **`left`, `right`, `inside` and `outside` now live in four enums**
   (`side_left_right`, `lane_change_side`, `gap_direction`, and
   `junction_direction` for `left`/`right`). §7.3.3's `lane_change_side!left`
   becomes unavoidable in ordinary scenario text. Worth a pinning test, since
   §8.9's modifiers add more of the same.
3. **`change_position.target_xyz` is marked deprecated** in favour of
   `target_position`, in the same table that declares it. It is declared: the
   language has no deprecation marker, and dropping a field the standard prints
   would reject a conforming scenario. A diagnostic for it belongs to a
   deprecation pass, not to the library.
4. **`change_lane.reference` documents `Default=it.actor`.** A default that
   names the invoking actor is not a constant expression, so no default is
   declared — the field is left unconstrained, which is what §7.3.11 already
   means.
5. **Three names appear as both an action here and a modifier in §8.9**
   (`change_speed`, `keep_speed`, `change_lane`). They do not collide: an
   action's name is a qualified behavior name (§7.2.2.2.5), so this chapter
   declares `movable_object.change_speed` while §8.9 declares a plain
   `change_speed`. The 43g slice must keep that distinction.
