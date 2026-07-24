# Routing, trajectories, controllers and visibility

This chapter covers the private actions that steer an entity along a path or
install state on it: `AssignRouteAction` and `AcquirePositionAction` (routes),
`FollowTrajectoryAction` (trajectories), `AssignControllerAction` /
`ActivateControllerAction` (controllers), and `VisibilityAction`. It follows
ASAM OpenSCENARIO XML 1.4.0 §6.8 (routes), §6.9 (trajectories), §6.6 and
§7.4.1.4 (controllers), and §VisibilityAction.

Distance keeping — the other action of this group — lives in
[Motion](motion.md#distance-keeping).

## What ends immediately, and what takes time

Annex A Table 10 divides these actions cleanly. An action that assigns a
**control strategy** takes simulation time; the rest complete in the evaluation
they fire:

| Action | Assigns | Ends |
|---|---|---|
| `AssignRouteAction` | nothing | immediately |
| `AcquirePositionAction` | nothing | immediately |
| `AssignControllerAction` | nothing | immediately |
| `ActivateControllerAction` | nothing | immediately |
| `VisibilityAction` | nothing | immediately |
| `FollowTrajectoryAction` (`timeReference=none`) | lateral | at the end of the trajectory |
| `FollowTrajectoryAction` (`timeReference=timing`) | lateral + longitudinal | at the end of the trajectory |

"Completes immediately" is not "does nothing": these actions install per-entity
state that stays until something replaces it, and the engine exposes it.

## Routes

A `Route` is a name, a `closed` flag and an ordered list of `Waypoint`s — a
position plus the `RouteStrategy` for reaching it (`fastest`, `shortest`,
`leastIntersections`, `random`). At least two waypoints are required.

`AssignRouteAction` installs a route on an entity; a later routing action
overwrites it (§6.8.2). `AcquirePositionAction` installs the implicit
two-waypoint route of §7.4.1.4 — the entity's position **at apply time** as the
first waypoint and the target as the second, both with the `shortest` strategy
— so *when* it fires changes the route it produces.

Scena **stores** a route; it does not follow one. Selecting a path between
waypoints needs a road network, which arrives with the road sprints. That is
also why `random` is harmless here: the strategy is data, and no random number
generator exists anywhere in the runtime (see [Determinism](determinism.md)).

Neither action touches a control domain, so assigning a route to an entity in
the middle of a speed ramp leaves the ramp running (§7.4.1.4).

Read the current route back with `Engine::route_of` (C++), `route_of` (Python),
or `scn_engine_entity_route_info` + `scn_engine_entity_route_waypoint_at` (C).

## Trajectories

A `Trajectory` carries one of three **shapes** (§6.9), and
`FollowTrajectoryAction` moves an entity along it:

- **Polyline** — an ordered chain of at least two vertices, each a position and
  an optional time; the path is the straight segments between them.
- **Clothoid** — an Euler spiral whose curvature changes linearly with arc
  length: `curvature` is the initial curvature and `curvaturePrime` its rate of
  change. `curvaturePrime = 0` is a circular arc; both zero is a straight line.
- **NURBS** — a non-uniform rational B-spline of a given `order` (= degree + 1)
  over a control-point and knot vector; it expresses circles and other conics
  exactly.

`ClothoidSpline`, the 1.4 `Motion` element and the polyline `Interpolation`
element are out of scope for v0.0.1 (see the coverage matrix). A `closed`
trajectory is accepted with a warning and followed as an open path.

**Numerical fidelity (risk R3).** The clothoid's straight-line and circular-arc
cases are closed form; a general spiral integrates the Fresnel-type integrand
with a deterministic composite-Simpson quadrature built on Scena's `det_sincos`,
so it is bit-identical everywhere. The NURBS is evaluated by the rational de
Boor recursion (IEEE operations only) and reparameterised to arc length through
a fixed-resolution table. Sampled points match the analytic curve within
`1e-9` m, and the whole evaluation is covered by the determinism contract.
A trajectory-relative `Position` (`TrajectoryPosition`) resolves the same way:
the geometry is evaluated at arc length `s` and offset laterally by `t`.

### Without a time reference

The entity **teleports to the start** of the trajectory when the action begins
(§6.9.1) and then advances along it by `speed × dt` — the trajectory is pure
geometry, and whatever owns the longitudinal domain still sets the pace. A
`SpeedAction` fired mid-trajectory therefore changes how fast the entity travels
the path without disturbing the path itself.

### With a `Timing`

The vertex times drive the motion, so the action owns the longitudinal domain
too. Every vertex must carry a time. The effective time of a vertex is
`time × scale + offset`, measured from simulation time zero
(`domain=absolute`) or from the instant the action started (`domain=relative`).
Two start cases follow from that:

- the first time reference is still in the **future**: the entity keeps moving
  as before and teleports to the start when that time arrives (§6.9.2);
- it is already in the **past**: the entity joins at the point interpolated
  between the surrounding time references (§6.9.3).

Position, heading and speed all come from the timed segments, so the state a
condition observes stays consistent with the path.

### Both modes

The entity's **heading** follows the current segment's direction, computed with
Scena's deterministic `atan2` so it is bit-identical everywhere. The action ends
by reaching the end of the trajectory, snapped exactly onto the final vertex,
and the entity returns to the straight-line model from there.

`initialDistanceOffset` truncates the trajectory so following starts at that arc
length. `followingMode=follow` is accepted but executed as `position`: a
best-effort steering controller needs lateral dynamics, so until then Scena does
what it can guarantee and says so with a warning.

## Controllers

Scena implements **no controller models**. An `AssignControllerAction` stores
the controller's name, its `controllerType` (which domains it may act on) and
its properties — in authored order — and hands them to the host through
`ISimulatorGateway::on_controller_assigned`. What a controller named "aggressive
driver" means is a contract between the scenario author and the host simulator.

Activation is different, because Scena's engine **is** the default controller
(see [Motion](motion.md)). The activation flags on `AssignControllerAction` and
on `ActivateControllerAction` toggle the engine's own control of a domain:

- **deactivating longitudinal** retires whatever action was driving the speed —
  it completes cleanly — and the entity holds the speed it had;
- **deactivating lateral** stops an active trajectory follower the same way;
- while a domain is inactive, an action needing it is reported and skipped (its
  event still completes, so nothing hangs);
- an **unset** flag means "no change", and re-activating resumes normal
  dispatch.

Activating a domain the controller's `controllerType` does not cover is a
validation error (rule
`asam.net:xosc:1.2.0:scenario_logic.controller_activation`): a lighting
controller cannot take over the steering. Deactivation is never constrained that
way. The lighting and animation domains have no runtime in Scena and are not
modeled.

## Visibility

A `VisibilityAction` sets three flags — visible to the image generator, to
sensors, and to other traffic participants. Entities are visible everywhere
until one fires. The engine has no image generator, sensor model or traffic
participants of its own, so it stores the flags, reports them
(`Engine::visibility_of`) and notifies the gateway
(`on_visibility_changed`); acting on them is the host's job. Visibility never
affects motion. Per-sensor visibility (`sensorReferenceSet`) is not modeled.

## Driving it from code

**C++**:

```cpp
using namespace scena::ir;

Route route;
route.waypoints.push_back(Waypoint{WorldPosition{0.0, 0.0, 0.0}, RouteStrategy::Shortest});
route.waypoints.push_back(Waypoint{WorldPosition{200.0, 50.0, 0.0}, RouteStrategy::Fastest});
event.actions.push_back(std::make_shared<AssignRouteAction>("ego", route));

Trajectory trajectory; // defaults to a polyline shape
trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, 0.0});
trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{100.0, 0.0, 0.0}, 10.0});
event.actions.push_back(std::make_shared<FollowTrajectoryAction>(
    "ego", trajectory, FollowingMode::Position, Timing{ReferenceContext::Absolute, 1.0, 0.0}));

// A clothoid arc, or a NURBS, is just a different shape on the Trajectory:
Trajectory arc{"arc", false, Clothoid{WorldPosition{0.0, 0.0, 0.0}, /*curvature=*/0.05,
                                      /*curvaturePrime=*/0.0, /*length=*/31.4}};
event.actions.push_back(std::make_shared<FollowTrajectoryAction>("ego", arc));

Controller controller;
controller.name = "driver";
controller.properties.push_back(Property{"model", "idm"});
event.actions.push_back(std::make_shared<AssignControllerAction>("ego", controller));
event.actions.push_back(std::make_shared<VisibilityAction>("ego", true, false, false));
```

**C ABI** — `scn_engine_add_assign_route_action`,
`scn_engine_add_acquire_position_action`,
`scn_engine_add_follow_trajectory_action` (polyline) with
`scn_engine_add_follow_clothoid_trajectory_action` and
`scn_engine_add_follow_nurbs_trajectory_action` for the other shapes,
`scn_engine_add_assign_controller_action`,
`scn_engine_add_activate_controller_action` and
`scn_engine_add_visibility_action`:

```c
scn_waypoint waypoints[] = {
    {0.0, 0.0, 0.0, SCN_ROUTE_STRATEGY_SHORTEST},
    {200.0, 50.0, 0.0, SCN_ROUTE_STRATEGY_FASTEST},
};
scn_engine_add_assign_route_action(engine, "ego", "r1", waypoints, 2, /*closed=*/0, 1.0,
                                   SCN_PRIORITY_PARALLEL, 1);

scn_trajectory_vertex vertices[] = {
    {0.0, 0.0, 0.0, 0.0, /*has_time=*/1},
    {100.0, 0.0, 0.0, 10.0, 1},
};
scn_timing timing = {SCN_REFERENCE_CONTEXT_ABSOLUTE, 1.0, 0.0};
scn_engine_add_follow_trajectory_action(engine, "ego", "t1", vertices, 2, 0,
                                        SCN_FOLLOWING_MODE_POSITION, &timing, 0.0, 2.0,
                                        SCN_PRIORITY_PARALLEL, 1);

/* Tri-state activation: negative leaves the domain alone. */
scn_engine_add_activate_controller_action(engine, "ego", /*lateral=*/-1, /*longitudinal=*/0,
                                          5.0, SCN_PRIORITY_PARALLEL, 1);
scn_engine_add_visibility_action(engine, "ego", 1, 0, 0, 6.0, SCN_PRIORITY_PARALLEL, 1);
```

**Python** — see
[`python/examples/routing.py`](../../python/examples/routing.py):

```python
route = scn.Route(waypoints=[
    scn.Waypoint(scn.WorldPosition(0.0, 0.0, 0.0)),
    scn.Waypoint(scn.WorldPosition(200.0, 50.0, 0.0), scn.RouteStrategy.Fastest),
])
event.add_action(scn.AssignRouteAction("ego", route))

trajectory = scn.Trajectory(vertices=[
    scn.TrajectoryVertex(scn.WorldPosition(0.0, 0.0, 0.0)),
    scn.TrajectoryVertex(scn.WorldPosition(100.0, 100.0, 0.0)),
])
event.add_action(scn.FollowTrajectoryAction("ego", trajectory))
event.add_action(scn.VisibilityAction("ego", graphics=True, sensors=False, traffic=False))

engine.route_of("ego")                 # the assigned route, or None
engine.assigned_controller_of("ego")   # the assigned controller, or None
engine.controller_activation_of("ego") # which domains the engine drives
engine.visibility_of("ego")            # the current detectability flags
```
