#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Road-to-v0.0.1 GitHub seeding script.
#
# *** MAINTAINER-EXECUTED ONLY. ***
# This script is a reviewable artifact: agents and CI must never run it.
# Review every command below, then run it yourself from the repo root.
# It creates labels, one milestone per pillar, one tracking issue per
# pillar, and one issue per sprint (bodies from docs/roadmap/issues/).
# It creates NO tags and NO releases — releases are a separate, manual,
# maintainer-only act per docs/architecture/ADR-0004-road-to-v001.md.
#
# Requirements: gh (authenticated), repo = current directory's origin.
# Note: gh has no built-in milestone command; milestones use `gh api`.
# Re-running: label/milestone creation fails harmlessly if they exist;
# issue creation is NOT idempotent — do not run twice without review.

set -euo pipefail

ISSUES_DIR="docs/roadmap/issues"
[ -d "$ISSUES_DIR" ] || { echo "run from the repo root"; exit 1; }

echo "== Labels =="
gh label create roadmap   --color 0e8a16 --description "Road to v0.0.1" || true
gh label create sprint    --color 1d76db --description "One-sprint issue (single focused PR)" || true
gh label create tracking  --color 5319e7 --description "Pillar tracking issue" || true
for n in 1 2 3 4 5 6 7 8; do
  gh label create "pillar:p$n" --color c2e0c6 --description "Pillar P$n" || true
done

echo "== Milestones (one per pillar) =="
mkmilestone() { # title description
  gh api "repos/{owner}/{repo}/milestones" -f title="$1" -f description="$2" >/dev/null \
    && echo "milestone: $1" || echo "milestone exists (or failed): $1"
}
mkmilestone "P1 - Runtime Core & Determinism"        "Storyboard execution model, triggers, priorities, diagnostics, determinism hardening"
mkmilestone "P2 - Entities, Motion & Control"        "Entity taxonomy, longitudinal/lateral dynamics, positions, trajectories"
mkmilestone "P3 - Road Interface"                    "IRoadQuery v1 + ASAM OpenDRIVE reference backend + routes"
mkmilestone "P4 - OpenSCENARIO XML Frontend"         "XML 1.0-1.3 loader: storyboard, parameters, catalogs, validation"
mkmilestone "P5 - Actions & Conditions Semantics"    "Declared action/condition catalog per coverage matrix"
mkmilestone "P6 - Embedding: Gateway, C API & Python" "Full C ABI, gateway callbacks, Python parity, scena-run + golden harness"
mkmilestone "P7 - OpenSCENARIO DSL Frontend"         "DSL 2.x grammar, AST, types, standard-library checking, scena-check"
mkmilestone "P8 - OpenSCENARIO DSL Execution"        "Concrete-scenario lowering, composition operators, modifiers, XML/DSL parity"

mkissue() { # file title milestone extra_label
  gh issue create \
    --title "$2" \
    --body-file "$ISSUES_DIR/$1" \
    --milestone "$3" \
    --label roadmap --label "$4" --label "${5:-sprint}"
}

echo "== Pillar tracking issues =="
mkissue p1-tracking.md "[P1] Runtime Core & Determinism - tracking"         "P1 - Runtime Core & Determinism"         pillar:p1 tracking
mkissue p2-tracking.md "[P2] Entities, Motion & Control - tracking"         "P2 - Entities, Motion & Control"         pillar:p2 tracking
mkissue p3-tracking.md "[P3] Road Interface - tracking"                     "P3 - Road Interface"                     pillar:p3 tracking
mkissue p4-tracking.md "[P4] OpenSCENARIO XML Frontend - tracking"          "P4 - OpenSCENARIO XML Frontend"          pillar:p4 tracking
mkissue p5-tracking.md "[P5] Actions & Conditions Semantics - tracking"     "P5 - Actions & Conditions Semantics"     pillar:p5 tracking
mkissue p6-tracking.md "[P6] Embedding: Gateway, C API & Python - tracking" "P6 - Embedding: Gateway, C API & Python" pillar:p6 tracking
mkissue p7-tracking.md "[P7] OpenSCENARIO DSL Frontend - tracking"          "P7 - OpenSCENARIO DSL Frontend"          pillar:p7 tracking
mkissue p8-tracking.md "[P8] OpenSCENARIO DSL Execution - tracking"         "P8 - OpenSCENARIO DSL Execution"         pillar:p8 tracking

echo "== Sprint issues =="
# P1
mkissue p1-s1-storyboard-ir.md          "[p1-s1] Full storyboard IR & element state machines"      "P1 - Runtime Core & Determinism" pillar:p1
mkissue p1-s2-triggers.md               "[p1-s2] Triggers, condition groups & edge semantics"      "P1 - Runtime Core & Determinism" pillar:p1
mkissue p1-s3-event-lifecycle.md        "[p1-s3] Event lifecycle, priority & overwrite rules"      "P1 - Runtime Core & Determinism" pillar:p1
mkissue p1-s4-diagnostics.md            "[p1-s4] Structured diagnostics & error model"             "P1 - Runtime Core & Determinism" pillar:p1
mkissue p1-s5-determinism-hardening.md  "[p1-s5] Determinism hardening & replay harness"           "P1 - Runtime Core & Determinism" pillar:p1
# P2
mkissue p2-s1-entity-taxonomy.md        "[p2-s1] Entity taxonomy, bounding boxes & performance"    "P2 - Entities, Motion & Control" pillar:p2
mkissue p2-s2-longitudinal-dynamics.md  "[p2-s2] Longitudinal dynamics & default controller"       "P2 - Entities, Motion & Control" pillar:p2
mkissue p2-s3-lateral-dynamics.md       "[p2-s3] Lateral dynamics & lane-change shapes"            "P2 - Entities, Motion & Control" pillar:p2
mkissue p2-s4-positions-host-roundtrip.md "[p2-s4] Position resolution, teleport & host round-trip" "P2 - Entities, Motion & Control" pillar:p2
mkissue p2-s5-trajectories.md           "[p2-s5] Trajectory following: polyline, clothoid, NURBS"  "P2 - Entities, Motion & Control" pillar:p2
# P3
mkissue p3-s1-roadquery-v1.md           "[p3-s1] IRoadQuery v1: the frozen road interface"         "P3 - Road Interface" pillar:p3
mkissue p3-s2-opendrive-geometry.md     "[p3-s2] OpenDRIVE backend: reference-line geometry"       "P3 - Road Interface" pillar:p3
mkissue p3-s3-lanes-routes.md           "[p3-s3] Lanes, lane sections & routes"                    "P3 - Road Interface" pillar:p3
mkissue p3-s4-road-integration.md       "[p3-s4] Road-based positions in actions & conditions"     "P3 - Road Interface" pillar:p3
# P4
mkissue p4-s1-xml-infrastructure.md     "[p4-s1] XML infrastructure: parsing, versions, diagnostics" "P4 - OpenSCENARIO XML Frontend" pillar:p4
mkissue p4-s2-storyboard-loading.md     "[p4-s2] Entities, storyboard & init loading"              "P4 - OpenSCENARIO XML Frontend" pillar:p4
mkissue p4-s3-parameters-expressions.md "[p4-s3] Parameters, expressions & variables"              "P4 - OpenSCENARIO XML Frontend" pillar:p4
mkissue p4-s4-catalogs-controllers.md   "[p4-s4] Catalogs, entity selections & controllers"        "P4 - OpenSCENARIO XML Frontend" pillar:p4
mkissue p4-s5-versions-validation.md    "[p4-s5] 1.0-1.3 migration & file-level validation"        "P4 - OpenSCENARIO XML Frontend" pillar:p4
# P5
mkissue p5-s1-byvalue-conditions.md     "[p5-s1] ByValue conditions"                               "P5 - Actions & Conditions Semantics" pillar:p5
mkissue p5-s2-byentity-conditions-1.md  "[p5-s2] ByEntity conditions I: kinematics & position"     "P5 - Actions & Conditions Semantics" pillar:p5
mkissue p5-s3-byentity-conditions-2.md  "[p5-s3] ByEntity conditions II: interaction metrics"      "P5 - Actions & Conditions Semantics" pillar:p5
mkissue p5-s4-private-actions-1.md      "[p5-s4] Private actions I: longitudinal, lateral, teleport" "P5 - Actions & Conditions Semantics" pillar:p5
mkissue p5-s5-private-actions-2.md      "[p5-s5] Private actions II: routing, distance keeping & controllers" "P5 - Actions & Conditions Semantics" pillar:p5
mkissue p5-s6-global-actions.md         "[p5-s6] Global & infrastructure actions"                  "P5 - Actions & Conditions Semantics" pillar:p5
# P6
mkissue p6-s1-capi-expansion.md         "[p6-s1] C ABI expansion to the full engine surface"       "P6 - Embedding: Gateway, C API & Python" pillar:p6
mkissue p6-s2-gateway-maturity.md       "[p6-s2] Gateway maturity: callbacks & state injection"    "P6 - Embedding: Gateway, C API & Python" pillar:p6
mkissue p6-s3-python-parity.md          "[p6-s3] Python parity & examples"                         "P6 - Embedding: Gateway, C API & Python" pillar:p6
mkissue p6-s4-scena-run-golden.md       "[p6-s4] scena-run CLI & golden harness"                   "P6 - Embedding: Gateway, C API & Python" pillar:p6
# P7
mkissue p7-s1-dsl-grammar.md            "[p7-s1] DSL grammar & lexer (ANTLR4)"                     "P7 - OpenSCENARIO DSL Frontend" pillar:p7
mkissue p7-s2-parser-ast.md             "[p7-s2] Parser & AST"                                     "P7 - OpenSCENARIO DSL Frontend" pillar:p7
mkissue p7-s3-types-symbols.md          "[p7-s3] Symbols & type system"                            "P7 - OpenSCENARIO DSL Frontend" pillar:p7
mkissue p7-s4-expressions-constraints.md "[p7-s4] Expressions, constraints & coverage (check-only)" "P7 - OpenSCENARIO DSL Frontend" pillar:p7
mkissue p7-s5-stdlib-scena-check.md     "[p7-s5] Standard library checking & scena-check"          "P7 - OpenSCENARIO DSL Frontend" pillar:p7
# P8
mkissue p8-s1-lowering.md               "[p8-s1] Lowering concrete scenarios to the IR"            "P8 - OpenSCENARIO DSL Execution" pillar:p8
mkissue p8-s2-composition-operators.md  "[p8-s2] Composition operators: serial, parallel, one_of"  "P8 - OpenSCENARIO DSL Execution" pillar:p8
mkissue p8-s3-movement-modifiers.md     "[p8-s3] Movement modifiers"                               "P8 - OpenSCENARIO DSL Execution" pillar:p8
mkissue p8-s4-parity-golden.md          "[p8-s4] XML-DSL parity & DSL golden scenarios"            "P8 - OpenSCENARIO DSL Execution" pillar:p8

echo "Done. Review the created issues, then link sprint issues into each"
echo "pillar tracking issue's checklist by hand."
