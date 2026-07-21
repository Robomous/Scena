<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p3-s2] OpenDRIVE backend: reference-line geometry

**Pillar:** P3 — Road Interface · **Roadmap:** `docs/roadmap/roadmap.md` § p3-s2

## Goal
Parse OpenDRIVE and evaluate reference-line geometry exactly.

## Deliverables
- New component `roads/opendrive/` (separate library target; the core never links it — hosts inject it via the gateway).
- XML reading via the approved parser dependency (shared with P4).
- planView geometry evaluation (line/arc/spiral/poly3/paramPoly3) through `detmath`; s-parameterization; inertial↔track (s,t) conversion for the reference line.
- Structured diagnostics for unsupported map features (never silent).

## Tests
- `opendrive_geometry_test.cpp` (hex-pinned analytic fixtures per geometry primitive, round-trip world↔track).

## Docs
- Roads page: supported OpenDRIVE subset table.

## Exit criteria
- [ ] Geometry suite green cross-platform
- [ ] Unsupported-feature diagnostics verified
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p3-s1, p1-s4 (diagnostics for reader warnings). Prerequisite maintainer action: OpenDRIVE spec text fetched (`--std opendrive`) before this sprint starts (open question OQ-3).
