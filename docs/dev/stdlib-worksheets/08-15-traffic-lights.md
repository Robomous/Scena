# §8.15 — traffic lights

Source: ASAM OpenSCENARIO DSL 2.2.0, `domain-model` §8.15 (Tables 301–343).
Landed in slice 43h. Target chunk: `kDomainTrafficLights` in
`frontends/dsl/src/stdlib.cpp`.

§8.15 is three enums and a bulb struct (§8.15.2), a semantic-state enum
(§8.15.3), the traffic-light and group structs with their methods (§8.15.4), the
stop line (§8.15.5), phase and cycle (§8.15.6), the two `map` fields (§8.15.7),
the controller actor (§8.15.8) and seven actions (§8.15.9). Everything is
printed as tables except the four method prototypes.

## Enums

| § | Enum | Members | ⚠ |
| --- | --- | --- | --- |
| .2.1 | `bulb_icon` | 45 values, `unknown` first — see the chunk for the full list in table order | the comment column maps each to an OpenDRIVE signal type; not carried, it is documentation |
| .2.2 | `bulb_color` | `unknown, red, yellow, green, blue, white` | every member also exists in `color` (§8.7.15) → callers need `bulb_color!red` (§7.3.3) |
| .2.3 | `bulb_state` | `unknown, is_off, is_on, is_flashing` | |
| .3.1 | `semantic_traffic_light_state` | `off, stop, attention, caution, stop_attention, go, go_exclusive, non_functional` | `off` is not a reserved word (§7.2.1.5.1 makes keywords positional), so it needs no escaping |
| .5.1 | `stop_line_marking` | `none, solid, broken` | `none` also exists in `hitch_type` (§8.7.18) and `directionality` (§8.12.6) |

## Structs

| § | Struct | Members | ⚠ |
| --- | --- | --- | --- |
| .2.4 | `traffic_light_bulb` | `map_id: string`, `icon: bulb_icon`, `color: bulb_color`, `icon_positive: bool`, `state: bulb_state` | Table 308 lists the first four as parameters, Table 309 lists `state` as a state variable; both become fields (43b rule) |
| .4.1 | `traffic_light` | `map_id: string`, `bulbs: list of traffic_light_bulb`, `pose: pose_3d`, `height: length`, `width: length`, `group: traffic_light_group` | forward reference to `traffic_light_group`, declared below it |
| .4.2 | `traffic_light_group` | `map_id: string`, `bulbs: list of traffic_light_bulb`, `traffic_lights: list of traffic_light`, `cycle: traffic_light_cycle` | mutually recursive with `traffic_light`; `cycle` forward-references §8.15.6.2 |
| .5.2 | `traffic_light_stop_line` **inherits `route_element`** | `traffic_light_group: traffic_light_group`, `route: route`, `rightmost_lane: uint`, `leftmost_lane: uint`, `offset: length`, `secondary_stop_offset: length`, `primary_stop_line_marking: stop_line_marking`, `secondary_stop_line_marking: stop_line_marking` | two field names equal their type names |
| .6.1 | `traffic_light_phase` | `group: traffic_light_group`, `bulbs_state: list of traffic_light_bulb`, `duration: time` | |
| .6.2 | `traffic_light_cycle` | `phases: list of traffic_light_phase`, `synchronization_group_id: uint`, `start_offset: time` | |

## Methods (§8.15.4.1.1, §8.15.4.2.1 — prototypes printed as code)

| Owner | Method | Signature |
| --- | --- | --- |
| `traffic_light` | `state_equal` | `(bulbs: list of traffic_light_bulb) -> bool` |
| `traffic_light` | `semantic_state_to_state` | `(state: semantic_traffic_light_state) -> list of traffic_light_bulb` |
| `traffic_light` | `state_to_semantic_state` | `(bulbs: list of traffic_light_bulb) -> semantic_traffic_light_state` |
| `traffic_light_group` | `state_equal` | `(bulbs: list of traffic_light_bulb) -> bool` |

⚠ §8.15.4.2.1's prototype for the **group's** `state_equal` prints
`extend traffic_light:`, not `extend traffic_light_group:`. The surrounding prose,
the section heading and Table 319 all say the method belongs to
`traffic_light_group`, so the prototype's receiver is a copy-paste slip and the
method is declared on the group. This is the one place in the chapter where the
printed code is not followed; see note 1 below.

## Actor and actions

`actor traffic_light_controller inherits osc_actor` (§8.15.8.1, Table 329).

The seven §8.15.9 actions are declared on it. §8.8.1 defines only
`action_for_environment` and `action_for_movable_object` as intermediate bases,
so these inherit `osc_actor.osc_action` directly rather than inventing a third
intermediate. All are instantaneous except `play_cycles`; action ending is
runtime semantics and is not expressible in a declaration.

| § | Action | Fields |
| --- | --- | --- |
| .9.1.1 | `set_bulb_state` | `traffic_light: traffic_light`, `bulb_color: bulb_color`, `bulb_kind: bulb_icon`, `bulb_state: bulb_state`, `sync: bool` |
| .9.2.1 | `set_state` | `traffic_light: traffic_light`, `state: list of bulb_state`, `sync: bool` |
| .9.3.1 | `set_semantic_state` | `traffic_light: traffic_light`, `state: semantic_traffic_light_state`, `sync: bool` |
| .9.4.1 | `set_group_bulb_state` | `traffic_light: traffic_light`, `bulb_color: bulb_color`, `bulb_kind: bulb_icon`, `bulb_state: bulb_state`, `sync: bool` ⚠ |
| .9.5.1 | `set_group_state` | `group: traffic_light_group`, `state: list of bulb_state`, `sync: bool` |
| .9.6.1 | `set_group_semantic_state` | `traffic_light_group: traffic_light_group`, `state: semantic_traffic_light_state`, `sync: bool` |
| .9.7.1 | `play_cycles` | `cycles: list of traffic_light_cycle` |

## ⚠ Notes carried into the library

1. **Table 337 (`set_group_bulb_state`) names its first parameter
   `traffic_light` of type `traffic_light`, while its description says "The
   traffic light group affected by the action".** The parameter table is the
   normative surface and it is what a conforming scenario would be written
   against, so the printed name and type are carried verbatim and the
   inconsistency is recorded here. This follows the same rule as the §8.14.1.3
   conversion factors (ADR-0029): what the standard prints wins over what it
   evidently meant. Contrast §8.15.4.2.1, where the *prose, the heading and the
   table* agree against a single line of printed code — there the code loses.
   The distinction is which surface the rest of the chapter corroborates.
2. **`set_group_state` names the parameter `group`, `set_group_semantic_state`
   names it `traffic_light_group`.** Both carried as printed.
3. **`traffic_light` and `traffic_light_group` are mutually recursive** through
   `traffic_light.group` and `traffic_light_group.traffic_lights`. The resolver's
   declare pass registers every type before the link pass reads any field type,
   so declaration order is free; they are declared in chapter order.
4. **Table 313's `state` for `traffic_light` does not exist.** A traffic light has
   no state of its own — its state is the state of its bulbs, which is why
   §8.15.4.1.1 provides `state_equal` and the two conversion methods instead. The
   §8.15.10 examples read state only through those methods.
5. **§8.15.10's examples write `stop_line.road`,** but Table 323 declares the
   field as `route`. The table is normative and the example is illustrative; the
   field is `route`. An author following the example gets an "unknown member"
   diagnostic, which is the correct outcome.
