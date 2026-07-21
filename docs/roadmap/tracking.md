# Road to v0.0.1 — GitHub tracking map

The roadmap of `docs/roadmap/roadmap.md` is reflected on GitHub
(seeded 2026-07-21 from the issue seeds in `docs/roadmap/issues/` via
`scripts/roadmap-gh-seed.sh`). This file is the durable local map between
the markdown plan and the GitHub tracking artifacts.

- **Project board:** https://github.com/orgs/Robomous/projects/2
  ("Scena — Road to v0.0.1", linked to the repository; group by
  **Milestone** to see the pillar lanes).
- **Milestones:** one per pillar (P1–P8) — pillar completion =
  milestone 100 %.
- **Epics:** one tracking issue per pillar; the 38 sprint issues are
  attached as native **sub-issues** of their epic, so each epic shows live
  progress.
- **Tasks:** one issue per sprint. A sprint closes when its single focused
  PR merges with CI green (reference the issue from the PR body with
  `Closes #<n>`).

How the two sides stay in sync: the markdown roadmap is the plan of
record (scope, exit criteria, coverage); GitHub tracks execution state.
Scope changes go through a roadmap PR first, then the affected issues are
edited to match. Releases and tags remain maintainer-only (ADR-0004);
closing every issue here does **not** publish anything by itself — the
release gate in `roadmap.md` still applies.

## Epics (pillar tracking issues)

| Pillar | Epic | Milestone |
|---|---|---|
| P1 — Runtime Core & Determinism | [#2](https://github.com/Robomous/Scena/issues/2) | P1 - Runtime Core & Determinism |
| P2 — Entities, Motion & Control | [#3](https://github.com/Robomous/Scena/issues/3) | P2 - Entities, Motion & Control |
| P3 — Road Interface | [#4](https://github.com/Robomous/Scena/issues/4) | P3 - Road Interface |
| P4 — OpenSCENARIO XML Frontend | [#5](https://github.com/Robomous/Scena/issues/5) | P4 - OpenSCENARIO XML Frontend |
| P5 — Actions & Conditions Semantics | [#6](https://github.com/Robomous/Scena/issues/6) | P5 - Actions & Conditions Semantics |
| P6 — Embedding: Gateway, C API & Python | [#7](https://github.com/Robomous/Scena/issues/7) | P6 - Embedding: Gateway, C API & Python |
| P7 — OpenSCENARIO DSL Frontend | [#8](https://github.com/Robomous/Scena/issues/8) | P7 - OpenSCENARIO DSL Frontend |
| P8 — OpenSCENARIO DSL Execution | [#9](https://github.com/Robomous/Scena/issues/9) | P8 - OpenSCENARIO DSL Execution |

## Sprint issues

| Sprint | Issue | Sprint | Issue |
|---|---|---|---|
| p1-s1 | [#10](https://github.com/Robomous/Scena/issues/10) | p5-s1 | [#29](https://github.com/Robomous/Scena/issues/29) |
| p1-s2 | [#11](https://github.com/Robomous/Scena/issues/11) | p5-s2 | [#30](https://github.com/Robomous/Scena/issues/30) |
| p1-s3 | [#12](https://github.com/Robomous/Scena/issues/12) | p5-s3 | [#31](https://github.com/Robomous/Scena/issues/31) |
| p1-s4 | [#13](https://github.com/Robomous/Scena/issues/13) | p5-s4 | [#32](https://github.com/Robomous/Scena/issues/32) |
| p1-s5 | [#14](https://github.com/Robomous/Scena/issues/14) | p5-s5 | [#33](https://github.com/Robomous/Scena/issues/33) |
| p2-s1 | [#15](https://github.com/Robomous/Scena/issues/15) | p5-s6 | [#34](https://github.com/Robomous/Scena/issues/34) |
| p2-s2 | [#16](https://github.com/Robomous/Scena/issues/16) | p6-s1 | [#35](https://github.com/Robomous/Scena/issues/35) |
| p2-s3 | [#17](https://github.com/Robomous/Scena/issues/17) | p6-s2 | [#36](https://github.com/Robomous/Scena/issues/36) |
| p2-s4 | [#18](https://github.com/Robomous/Scena/issues/18) | p6-s3 | [#37](https://github.com/Robomous/Scena/issues/37) |
| p2-s5 | [#19](https://github.com/Robomous/Scena/issues/19) | p6-s4 | [#38](https://github.com/Robomous/Scena/issues/38) |
| p3-s1 | [#20](https://github.com/Robomous/Scena/issues/20) | p7-s1 | [#39](https://github.com/Robomous/Scena/issues/39) |
| p3-s2 | [#21](https://github.com/Robomous/Scena/issues/21) | p7-s2 | [#40](https://github.com/Robomous/Scena/issues/40) |
| p3-s3 | [#22](https://github.com/Robomous/Scena/issues/22) | p7-s3 | [#41](https://github.com/Robomous/Scena/issues/41) |
| p3-s4 | [#23](https://github.com/Robomous/Scena/issues/23) | p7-s4 | [#42](https://github.com/Robomous/Scena/issues/42) |
| p4-s1 | [#24](https://github.com/Robomous/Scena/issues/24) | p7-s5 | [#43](https://github.com/Robomous/Scena/issues/43) |
| p4-s2 | [#25](https://github.com/Robomous/Scena/issues/25) | p8-s1 | [#44](https://github.com/Robomous/Scena/issues/44) |
| p4-s3 | [#26](https://github.com/Robomous/Scena/issues/26) | p8-s2 | [#45](https://github.com/Robomous/Scena/issues/45) |
| p4-s4 | [#27](https://github.com/Robomous/Scena/issues/27) | p8-s3 | [#46](https://github.com/Robomous/Scena/issues/46) |
| p4-s5 | [#28](https://github.com/Robomous/Scena/issues/28) | p8-s4 | [#47](https://github.com/Robomous/Scena/issues/47) |

## Labels

`roadmap` (all), `tracking` (epics), `sprint` (tasks), `pillar:p1` …
`pillar:p8`.
