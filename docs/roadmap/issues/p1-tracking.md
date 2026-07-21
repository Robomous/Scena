<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P1] Runtime Core & Determinism — tracking

Tracking issue for pillar P1. See `docs/roadmap/roadmap.md`.

## Objective
Replace the F0 flat condition/action list with the full ASAM OpenSCENARIO storyboard execution model — Story/Act/ManeuverGroup/Maneuver/Event/Action lifecycle and state machines, trigger and condition-group evaluation, event priority rules — on a deterministic fixed-step scheduler with a structured diagnostics and error model (XML §7 components, §8 runtime behavior).

## Sprints
- [ ] p1-s1 — Full storyboard IR & element state machines (#)
- [ ] p1-s2 — Triggers, condition groups & edge semantics (#)
- [ ] p1-s3 — Event lifecycle, priority & overwrite rules (#)
- [ ] p1-s4 — Structured diagnostics & error model (#)
- [ ] p1-s5 — Determinism hardening & replay harness (#)

## Pillar exit criteria
- [ ] All P1 sprints merged
- [ ] Storyboard semantics tests (`storyboard_test`, `trigger_test`, `event_priority_test`) green on all platforms
- [ ] Determinism replay CI job comparing traces across the three OS runners green
- [ ] Diagnostics carry severity + location + rule ID
