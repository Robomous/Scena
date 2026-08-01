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

## The OpenDRIVE backend (p3-s2 onward)

`roads/opendrive/` is a separate library (`scena::roads-opendrive`) the core
never links: hosts parse a map with it and inject the resulting backend
through the gateway. p3-s2 delivers the document reader and exact
reference-line geometry; lanes, routing and the `IRoadQuery` implementation
follow in p3-s3.

Loading follows the kernel's diagnostics contract: content defects are
errors citing the `asam.net:xodr` rule id where one exists, and map features
outside the consumed subset are **never dropped silently** — each is
reported as an `UnsupportedFeature` warning (`DeprecatedFeature` for
`<poly3>`, which is still evaluated). Numeric attributes are parsed
locale-independently (`std::from_chars`), and evaluation runs through the
deterministic math layer, so identical maps evaluate bit-identically on
every platform.

### Supported OpenDRIVE subset

| Feature (OpenDRIVE 1.9.0) | Status |
|---|---|
| `<header>` revMajor/revMinor | read; non-1.x majors warned |
| `<road>` id/name/length | read |
| `<planView>` `<line>` (§9.3) | evaluated (closed form) |
| `<planView>` `<arc>` (§9.5) | evaluated (closed form) |
| `<planView>` `<spiral>` (§9.4) | evaluated (deterministic quadrature) |
| `<planView>` `<paramPoly3>` (§9.6), both pRange modes | evaluated (exact cubics, fixed-resolution s table) |
| `<planView>` `<poly3>` (§9.7) | evaluated; deprecation warned |
| `<lanes>` / `<laneSection>` left/center/right, `<lane>` id/type, `<width>` records (§11) | evaluated (polynomial widths, per-section lanes) |
| `<lane><link>` predecessor/successor (§11.6) | consumed (routing; temporary-layer multi-links warned) |
| road `<link>` predecessor/successor (§10.3) | consumed (routing) |
| `<junction type="default">` connections + laneLinks (§12.2) | consumed (routing); other junction types warned |
| elevation/superelevation/crossfall, lane offsets | scoped out for v0.0.1 (flat world); warned |
| `<objects>`, `<signals>`, surface/OpenCRG, georeferencing | scoped out for v0.0.1; warned |

The reference line is evaluated by `scena::opendrive::ReferenceLine`:
`pose_at(s)` returns the inertial pose of the s-axis, and `project(x, y)`
inverts it into track coordinates `(s, t)` with t positive to the left
(§8.3) and a documented smallest-s tie-break.

### Lanes and routing (p3-s3)

`OpenDriveRoadQuery` implements the complete frozen `IRoadQuery` v1 over the
lane model:

- **Lane ids** follow OpenDRIVE §11.1: 0 is the centre lane on the reference
  line (no width), negative ids go right (−t), positive ids go left (+t).
  Relative-lane arithmetic skips the centre lane, matching ASAM OpenSCENARIO
  XML 1.4.0 §7.4.1.4.
- **Widths** evaluate the §11.7.1 cubic records (`w(ds) = a + b·ds + c·ds² +
  d·ds³`), with the active record chosen by `sOffset` and results clamped at
  zero per the lane-width validity rule.
- **On-road** means the lateral offset lies inside the cumulative lane span
  of the cross-section at s. Elevation is scoped out for v0.0.1: roads live
  in the z = 0 plane and points off it are off-road, which keeps the two
  conversions exact mutual inverses.
- **Routing** is a deterministic Dijkstra over (road, lane) nodes connected
  by lane links across road links (§11.6/§10.3) and junction connections
  (§12.2), weighted by driven length. Driving direction follows the lane-id
  sign (negative lanes run along +s — §11.3.1, right-hand traffic). Equal
  paths tie-break on the lexicographically smallest (road id, lane id); the
  result is the frozen `RouteSpan` sequence, and `position_along_route`
  measures into it. The subset assumes stable lane ids within a road.
