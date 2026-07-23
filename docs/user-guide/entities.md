# The entity model

An entity is a participant in a scenario — a vehicle, a pedestrian, or a
static object. Scena models entities exactly as ASAM OpenSCENARIO XML 1.4.0
§7.2.2 defines them: a `ScenarioObject` is a name-identified participant plus,
optionally, one **entity object** that classifies it.

> Scena targets OpenSCENARIO XML 1.0–1.3; the category enumerations carry the
> full 1.4.0 sets so any 1.0–1.3 document maps, with post-1.3 additions and
> deprecations noted on each value.

## The shape of an entity

Every entity has an `id`, a `name`, and a control mode
([engine- or host-controlled](conditions.md)). Beyond that it *may* carry one
concrete **entity object**:

- **Vehicle** — a category (car, bus, bicycle, …), an optional role, a bounding
  box, `Performance` limits, an axle set, and a properties list.
- **Pedestrian** — a category (pedestrian, animal), an optional role, a bounding
  box, and properties.
- **MiscObject** — a category (barrier, pole, tree, …), a bounding box, and
  properties.

An entity with **no object** is a valid *unclassified participant*: it has an
identity and a control mode but no geometry or performance. This is the minimal
form the earliest scenarios used, and it still works.

Three properties read the classification without unpacking the object yourself:

- `object_type` → `Vehicle` / `Pedestrian` / `MiscObject`, or absent.
- `bounding_box` → the object's box, or absent. This is the single geometry
  source the interaction conditions measure against.
- `performance` → the vehicle's limits, or absent (only vehicles have them).

## Bounding box and performance

The **bounding box** (`§BoundingBox`) is the geometric center in the entity
body frame (`+x` forward, `+y` left, `+z` up) plus `length`/`width`/`height`.
The 2D freespace kernel the interaction conditions use consumes `length`,
`width`, and the `x`/`y` center offset; `center_z`/`height` are stored but do
not enter the planar math.

**Performance** (`§Performance`) is `max_speed`, `max_acceleration`,
`max_deceleration` (all required, ranges `[0..inf[`) and two optional rate
limits (`max_acceleration_rate`, `max_deceleration_rate`) that mean "unbounded"
when absent. `Engine::init` rejects a negative, NaN, or non-finite value as a
validation error. The default longitudinal controller (a later sprint) clamps
target-speed setpoints against these limits.

**Axles** and **properties** are carried faithfully as data — the straight-line
runtime does not read them yet, but they round-trip unchanged for frontends and
hosts. Properties preserve authored order and duplicate names (a list, not a
map).

## Pose

An entity's kinematic state is a full Z-up, right-handed pose:
`x, y, z` (meters), `heading`, `pitch`, `roll` (radians, h/p/r per
`§Orientation`), and a scalar `speed` along the heading. The straight-line
runtime integrates position from speed and heading only, leaving `pitch`/`roll`
at 0; a host may report any pose for a host-controlled entity.

## Building entities

### C++

```cpp
scena::ir::Vehicle car;
car.category = scena::ir::VehicleCategory::Car;
car.bounding_box = {1.4, 0.0, 0.8, 4.6, 2.0, 1.5}; // center + L/W/H
car.performance = {55.0, 4.0, 9.0, std::nullopt, std::nullopt};

scena::ir::Entity ego;
ego.id = "ego";
ego.name = "ego";
ego.control_mode = scena::ir::ControlMode::HostControlled;
ego.object = car;

// Derived views:
auto type = scena::ir::object_type_of(ego);      // ObjectType::Vehicle
auto box  = scena::ir::bounding_box_of(ego);      // the car's box
auto* pf  = scena::ir::performance_of(ego);       // &car.performance
```

### Python

```python
import scena as scn

ego = scn.Entity(
    "ego", "ego", scn.ControlMode.HostControlled,
    object=scn.Vehicle(
        category=scn.VehicleCategory.Car,
        bounding_box=scn.BoundingBox(center_x=1.4, center_z=0.8,
                                     length=4.6, width=2.0, height=1.5),
        performance=scn.Performance(max_speed=55.0, max_acceleration=4.0,
                                    max_deceleration=9.0),
    ),
)
assert ego.object_type == scn.ObjectType.Vehicle
assert ego.performance.max_speed == 55.0
```

The spec's `none` role and MiscObject category are exposed in Python as `NONE`,
because Python does not allow the keyword `None` as an attribute name.

See [`python/examples/entity_model.py`](../../python/examples/entity_model.py)
for a runnable example.

### C ABI

Typed builders `scn_engine_add_vehicle`, `scn_engine_add_pedestrian`,
`scn_engine_add_misc_object` construct classified entities; the bounding box is
required and, for a vehicle, so is `scn_performance`. The read-back accessors
`scn_engine_entity_object_type`, `scn_engine_entity_bounding_box`, and
`scn_engine_entity_performance` query the authored scenario and are valid before
and after `scn_engine_init`.

## Entering and leaving the scenario

Every declared entity starts **active** — in the scenario. `DeleteEntityAction`
takes one out and `AddEntityAction` puts it back at a position (§EntityAction);
the declaration itself is untouched either way, which is why an entity can
return after being deleted.

An inactive entity does not move, is not polled from or published to the
gateway, and reports nothing: `state`, `visibility_of`,
`controller_activation_of` and `route_of` all come back empty, `report_state`
fails with `UnknownEntity`, and every by-entity condition over it is a
deterministic `false`. `entity_active` tells the two failure modes apart —
`None` for an id the scenario never declared, `False` for one a delete removed:

```python
engine.entity_active("hazard")   # True / False / None
```

The lifecycle is a flag on the entity record, never an erase and insert: the
entity table's structure stays fixed for the whole run, so iteration order and
per-entity bookkeeping — what the determinism contract rests on — are
unaffected. The full rules, including what a delete clears and what it keeps,
are in
[Global actions](global-actions.md#the-entity-lifecycle).

## Not yet modeled

Entity **selections** (`EntitySelection`, `ByType`, `ByObjectType`) arrive with
the XML frontend catalogs (p4-s4). Trailers and hitches, the `model3d` render
hint, and catalog `parameterDeclarations` are out of the v0.0.1 entity scope.
Lowering the entity model from OpenSCENARIO XML lands with the frontend (P4).
