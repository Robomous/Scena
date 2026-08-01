# Roads: the road-network interface

Scena executes road-relative scenario behavior — lane changes, road- and
lane-relative positions, road-based distances — without owning a road
network. All road knowledge reaches the engine through one abstraction,
`scena::gateway::IRoadQuery`, provided by the host via the simulator gateway
(`ISimulatorGateway::road_query()`). The engine never depends on a concrete
map format; an OpenDRIVE-backed implementation ships as a separate library
the host injects like any other backend.

This interface is **frozen as v1**: everything downstream (position
resolution, road-based conditions, routing) codes against exactly this
surface, and extending it is an architecture-level decision (ADR-0003,
amendment p3-s1).

## Coordinate and lane conventions

The interface speaks ASAM OpenDRIVE road coordinates (OpenDRIVE 1.9.0 §8.3):

- `s` — distance along the road reference line, in meters, from the road's
  start.
- `t` — signed lateral offset from the reference line, positive to the
  **left** of the s-direction.
- Lane ids number lanes outward from the reference line (OpenDRIVE 1.9.0
  §11.1): the centre lane is `0` and has no width; lanes to the right are
  `-1, -2, …` (descending), lanes to the left are `1, 2, …` (ascending).
- Relative-lane arithmetic counts positive towards +t (left) and skips the
  centre lane, matching ASAM OpenSCENARIO XML 1.4.0 §7.4.1.4.

`LanePosition` bundles `(road_id, lane_id, s, t)`; it is the currency of all
lane-relative queries.

## The v1 query surface

| Group | Queries |
|---|---|
| Conversions | `to_lane_position`, `to_world_position`, `road_heading` |
| Lane queries | `lane_exists`, `lane_width`, `lane_center_offset`, `lane_type`, `relative_lane` |
| s-ranges | `road_length`, `lane_s_range` |
| Routes | `build_route` (waypoints → ordered `RouteSpan`s), `position_along_route` |

A route is an ordered sequence of `RouteSpan`s — contiguous stretches of a
single lane, each traversed from `s_begin` towards `s_end` (`s_end <
s_begin` means driving against the road's s-axis). Spans never lie on the
centre lane.

## Unsupported-reporting semantics

Every query returns `bool`: `true` fills the out-parameters with a
definitive answer; `false` means **no answer** — whether the input was
off-network, the id unknown, the input non-finite, or the query unsupported
by this backend. The engine treats all of those the same way: it falls back
to its flat-world behavior where one is defined (for example the configured
flat-world lane width for lane changes) or reports the operation as
unsupported through a diagnostic. Backends must never throw across the
boundary, and identical queries against identical road data must answer
bit-identically on every platform — the determinism contract extends through
this interface.

## Road-free scenarios: `FlatWorldRoadQuery`

`scena::gateway::FlatWorldRoadQuery` is the null-object backend: every query
answers `false`. Hosts that sometimes run without a map can hand the engine
this object instead of branching on `nullptr`; the behavior is identical to
providing no road query at all.

## The backend contract suite

`core/tests/support/road_query_contract.h` is an executable statement of
this contract: round-trip consistency of the conversions, rejection of
non-finite input, uniform no-answer behavior for unknown ids, well-formed
answered values, centre-lane exclusion, and bit-identical repeated queries.
It runs against `FlatWorldRoadQuery` today; every real backend instantiates
the same suite and must pass it unchanged.
