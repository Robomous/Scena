#!/usr/bin/env python3

# Copyright 2026 Robomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Runs the golden scenarios through `scena-run` and verifies their traces.

Two validation modes, per `docs/roadmap/golden-scenarios.md`:

1. **Determinism (bit-exact).** Re-running a scenario with the same step
   sequence must reproduce the committed reference trace byte-for-byte. This is
   the primary criterion, and any diff is a release blocker. The trace is
   compared as *text* deliberately: `scena-run` writes shortest-round-trip
   doubles, so identical text means identical bits, and a diff is readable.

2. **Semantic checkpoints (tolerance).** Each scenario declares assertions —
   "ego's speed is 20 ± 1e-9 at t = 9" — verified here. These catch behaviour
   that is deterministically *wrong*, which bit-identity alone never would.

There is **one** committed reference set, not one per platform. Determinism is
bit-identical across platforms — that is the project's core promise — so a
platform-specific reference would be a place for a divergence to hide. Comparing
every platform against the same bytes turns the golden job into a cross-platform
determinism check as well as a regression check. `--platform` exists for the
case where a platform legitimately needs its own reference; nothing uses it
today, and needing it would itself be the finding.

Subcommands:

    golden.py list                       names and what each exercises
    golden.py run <gs> [--out DIR]       run one scenario, write its trace
    golden.py verify <gs> <trace>        bit-compare + checkpoints
    golden.py record <gs> [--platform P] (re)record the reference trace
    golden.py check-all [--platform P]   run and verify every scenario

`record` overwrites a committed reference and is therefore never run by CI: a
reference trace changes only when a human decided the behaviour should.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
GOLDEN = REPO / "tests" / "golden"
SCENARIOS = GOLDEN / "scenarios"
TRACES = GOLDEN / "traces"


@dataclass(frozen=True)
class Checkpoint:
    """One semantic assertion: a field of an entity at a time, ± tolerance."""

    t: float
    entity: str
    field: str
    value: float
    tolerance: float = 1e-9


@dataclass(frozen=True)
class Scenario:
    name: str
    file: str
    duration: float
    exercises: str
    dt: float = 0.01
    checkpoints: tuple[Checkpoint, ...] = field(default_factory=tuple)


#: The suite. `exercises` is prose for `list`; the checkpoints are the contract.
SUITE: tuple[Scenario, ...] = (
    Scenario(
        name="gs1",
        file="gs1-cruise-baseline.xosc",
        duration=12.0,
        exercises="storyboard lifecycle, by-value condition, longitudinal dynamics, loader",
        checkpoints=(
            # Cruising at the init speed before the event fires at t > 5.
            Checkpoint(t=4.0, entity="ego", field="speed", value=10.0),
            # A 4 s linear ramp 10 -> 20 starting just after t = 5.
            Checkpoint(t=11.0, entity="ego", field="speed", value=20.0),
            # Straight line: heading never changes and y stays put.
            Checkpoint(t=11.0, entity="ego", field="heading", value=0.0),
            Checkpoint(t=11.0, entity="ego", field="y", value=0.0),
        ),
    ),
    Scenario(
        name="gs2",
        file="gs2-cut-in.xosc",
        duration=14.0,
        exercises="trigger edges, lateral dynamics, lane change, by-entity conditions",
        checkpoints=(
            # The cut-in vehicle starts one lane width to the left ...
            Checkpoint(t=1.0, entity="cutin", field="y", value=3.5),
            # ... and has settled into ego's lane after its 3 s lane change.
            Checkpoint(t=13.0, entity="cutin", field="y", value=0.0, tolerance=0.05),
            Checkpoint(t=13.0, entity="cutin", field="speed", value=16.0),
            Checkpoint(t=13.0, entity="ego", field="y", value=0.0),
        ),
    ),
    Scenario(
        name="gs3",
        file="gs3-overtake.xosc",
        duration=14.0,
        exercises="event sequencing via storyboard-state conditions, headway and "
        "freespace triggers, absolute lane targets on a real road network",
        checkpoints=(
            # Outer lane (-2 centre) at the declared speed before anything fires.
            Checkpoint(t=1.0, entity="ego", field="y", value=-5.25),
            Checkpoint(t=1.0, entity="ego", field="speed", value=22.0),
            # Phase 1 done: ego sits on the inner lane's centre (-1.75), which
            # only a road backend can supply — the flat-world model has no lane
            # identity at all.
            Checkpoint(t=6.0, entity="ego", field="y", value=-1.75, tolerance=0.01),
            # Phase 2 done: accelerated past the lead.
            Checkpoint(t=9.0, entity="ego", field="speed", value=30.0),
            # Phase 3 done: back on the outer lane, still at the overtaking speed.
            Checkpoint(t=13.0, entity="ego", field="y", value=-5.25, tolerance=0.01),
            Checkpoint(t=13.0, entity="ego", field="speed", value=30.0),
            # The lead never changed anything.
            Checkpoint(t=13.0, entity="lead", field="speed", value=12.0),
            Checkpoint(t=13.0, entity="lead", field="y", value=-5.25),
        ),
    ),
    Scenario(
        name="gs4",
        file="gs4-traffic-jam-approach.xosc",
        duration=20.0,
        exercises="continuous distance keeping, dynamic constraints, freespace gaps",
        checkpoints=(
            # Ego settles to the lead's speed while holding the gap.
            Checkpoint(t=19.0, entity="ego", field="speed", value=8.0, tolerance=0.5),
            Checkpoint(t=19.0, entity="lead", field="speed", value=8.0),
        ),
    ),
    Scenario(
        name="gs5",
        file="gs5-pedestrian-crossing.xosc",
        duration=12.0,
        exercises="pedestrian entity, braking to standstill, mixed entity types",
        checkpoints=(
            Checkpoint(t=11.0, entity="ego", field="speed", value=0.0),
            # The pedestrian keeps walking across; it is not engine-braked.
            Checkpoint(t=11.0, entity="walker", field="speed", value=1.4),
        ),
    ),
    Scenario(
        name="gs6",
        file="gs6-emergency-brake.xosc",
        duration=20.0,
        exercises="brake to standstill, standstill hold, a second event resuming",
        checkpoints=(
            Checkpoint(t=10.0, entity="ego", field="speed", value=0.0),
            Checkpoint(t=19.0, entity="ego", field="speed", value=12.0),
        ),
    ),
    Scenario(
        name="gs7",
        file="gs7-trajectory-slalom.xosc",
        duration=14.0,
        exercises="polyline trajectory following, arc-length evaluation",
        checkpoints=(
            # The slalom's waypoints, sampled where the path passes them: +2 m
            # at x = 30, -2 m at x = 60, and back on the axis at the end of the
            # 120 m path (t = 1 + 120/12 = 11 s). After that the follower is
            # done and the entity carries on with the heading it finished on —
            # so the checkpoints stop at the end of the trajectory, not after.
            Checkpoint(t=3.5, entity="ego", field="y", value=2.0, tolerance=0.1),
            Checkpoint(t=6.0, entity="ego", field="y", value=-2.0, tolerance=0.1),
            Checkpoint(t=11.0, entity="ego", field="y", value=0.0, tolerance=0.1),
            Checkpoint(t=11.0, entity="ego", field="speed", value=12.0),
        ),
    ),
    Scenario(
        name="gs8",
        file="gs8-junction-route.xosc",
        duration=20.0,
        exercises="four-way junction map, route assignment and acquire-position, "
        "a crossing vehicle yielding on a distance trigger",
        checkpoints=(
            # Ego holds lane -1 of the west-east corridor all the way through
            # the junction box (x in [100,150]) and out along road 4.
            Checkpoint(t=5.0, entity="ego", field="x", value=75.0),
            Checkpoint(t=5.0, entity="ego", field="y", value=-1.75),
            Checkpoint(t=9.0, entity="ego", field="y", value=-1.75),  # inside the box
            Checkpoint(t=19.0, entity="ego", field="y", value=-1.75),
            # The crosser yields: stopped while ego is approaching and inside
            # the junction, moving only after ego has cleared it.
            Checkpoint(t=5.0, entity="crosser", field="speed", value=0.0),
            Checkpoint(t=9.0, entity="crosser", field="speed", value=0.0),
            Checkpoint(t=19.0, entity="crosser", field="speed", value=12.0),
            # ... and it crosses along its own lane centre, not ego's.
            Checkpoint(t=19.0, entity="crosser", field="x", value=126.75),
        ),
    ),
    Scenario(
        name="gs9",
        file="gs9-parameters.xosc",
        duration=12.0,
        exercises="parameter declarations, $references and ${expressions}",
        checkpoints=(
            Checkpoint(t=2.0, entity="ego", field="speed", value=18.0),
            # 18 * 1.5, computed at load time by the expression evaluator.
            Checkpoint(t=11.0, entity="ego", field="speed", value=27.0),
        ),
    ),
    Scenario(
        name="gs10",
        file="gs10-host-controlled.xosc",
        duration=10.0,
        exercises="control ownership: an engine-controlled and a host-controlled entity",
        checkpoints=(Checkpoint(t=9.0, entity="ego", field="speed", value=18.0),),
    ),
    Scenario(
        name="gs11",
        file="gs11-signalized-intersection.xosc",
        duration=20.0,
        exercises="traffic signal controller phase clock, TrafficSignalCondition",
        checkpoints=(
            # Green for 6 s, amber for 2, then red at t = 8 triggers the stop.
            Checkpoint(t=5.0, entity="ego", field="speed", value=12.0),
            Checkpoint(t=19.0, entity="ego", field="speed", value=0.0),
        ),
    ),
    # --- OpenSCENARIO DSL (p8-s4) -------------------------------------------
    # Written in the other language, executed by the same runtime. GS-12 has an
    # XML twin (gs12-xml-cruise.xosc) whose trace must be byte-identical; that
    # pair is what turns "two frontends, one runtime" from a claim into a test.
    Scenario(
        name="gs12",
        file="gs12-dsl-cruise.osc",
        duration=12.0,
        exercises="DSL frontend end to end: entry point, actor lowering, serial "
        "composition with concrete durations, speed modifiers anchored at start and end",
        checkpoints=(
            # Phase 1 holds the speed the `at: start` modifier fixed.
            Checkpoint(t=4.0, entity="ego", field="speed", value=10.0),
            # Phase 2 ramps to 20 over its 4 s duration, and the scenario ends
            # when its `do` directive does (t = 9), freezing the last value.
            Checkpoint(t=11.0, entity="ego", field="speed", value=20.0, tolerance=0.03),
            # Straight line throughout: nothing lateral was ever asked for.
            Checkpoint(t=11.0, entity="ego", field="y", value=0.0),
        ),
    ),
    Scenario(
        name="gs13",
        file="gs13-dsl-alternatives.osc",
        duration=8.0,
        exercises="DSL composition with no XML counterpart: a parallel nested in a serial, "
        "a placement modifier, a relative-speed modifier, and a one_of alternative "
        "chosen by input",
        checkpoints=(
            # The placement modifier put lead 30 m ahead of ego at t = 0.
            Checkpoint(t=0.01, entity="lead", field="x", value=30.12),
            Checkpoint(t=1.0, entity="ego", field="speed", value=10.0),
            Checkpoint(t=1.0, entity="lead", field="speed", value=12.0),
            # The default alternative is `follow`: 4 m/s slower than lead, from
            # the moment the parallel phase ends at t = 2.
            Checkpoint(t=5.0, entity="ego", field="speed", value=8.0),
            Checkpoint(t=5.0, entity="lead", field="speed", value=12.0),
        ),
    ),
)

BY_NAME = {scenario.name: scenario for scenario in SUITE}


@dataclass(frozen=True)
class Pair:
    """One scenario written in both languages, with the same trace expected.

    The architecture's central claim is that the two frontends compile into one
    IR and that the runtime is shared. A pair is that claim as a test: the same
    scenario, authored once in OpenSCENARIO XML and once in OpenSCENARIO DSL,
    must produce traces that are equal byte for byte — not close, equal.
    """

    name: str
    dsl: str
    xml: str
    duration: float
    dt: float = 0.01


#: The declared pairs. `dsl` is a suite member, so its trace is already
#: bit-identical across platforms; the twin is run alongside and compared.
PAIRS: tuple[Pair, ...] = (
    Pair(name="gs12", dsl="gs12-dsl-cruise.osc", xml="gs12-xml-cruise.xosc", duration=12.0),
)


def platform_key(explicit: str | None = None) -> str:
    """The reference-trace directory. "reference" — shared — unless overridden."""
    return explicit or "reference"


def scena_run_binary() -> Path:
    """Finds the built scena-run. Honours SCENA_RUN if the caller knows better."""
    import os

    override = os.environ.get("SCENA_RUN")
    if override:
        return Path(override)
    # A multi-config generator (Visual Studio) puts the binary under a
    # per-configuration directory, so search rather than guess the layout.
    for name in ("scena-run", "scena-run.exe"):
        for candidate in sorted((REPO / "build").rglob(name)):
            if candidate.is_file():
                return candidate
    found = shutil.which("scena-run")
    if found:
        return Path(found)
    raise SystemExit(
        "scena-run not found; build it (cmake --build build) or set SCENA_RUN to its path"
    )


def run_scenario(scenario: Scenario, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    trace = out_dir / f"{scenario.name}.csv"
    command = [
        str(scena_run_binary()),
        str(SCENARIOS / scenario.file),
        "--dt",
        repr(scenario.dt),
        "--duration",
        repr(scenario.duration),
        "--trace",
        str(trace),
        "--quiet",
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise SystemExit(
            f"{scenario.name}: scena-run exited {result.returncode}\n{result.stderr}"
        )
    return trace


def read_trace(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    with path.open(encoding="utf-8") as file:
        header = file.readline().strip().split(",")
        for line in file:
            line = line.strip()
            if not line:
                continue
            rows.append(dict(zip(header, line.split(","))))
    return rows


def check_checkpoints(scenario: Scenario, trace: Path) -> list[str]:
    """Returns a list of failures; empty means every checkpoint held."""
    rows = read_trace(trace)
    failures: list[str] = []
    for checkpoint in scenario.checkpoints:
        # The sample at or just before the checkpoint time, for that entity.
        best: dict[str, str] | None = None
        for row in rows:
            if row["entity"] != checkpoint.entity:
                continue
            if float(row["t"]) <= checkpoint.t + 1e-9:
                best = row
            else:
                break
        if best is None:
            failures.append(
                f"{scenario.name}: no sample for '{checkpoint.entity}' at t <= {checkpoint.t}"
            )
            continue
        actual = float(best[checkpoint.field])
        if abs(actual - checkpoint.value) > checkpoint.tolerance:
            failures.append(
                f"{scenario.name}: {checkpoint.entity}.{checkpoint.field} at t={checkpoint.t} "
                f"is {actual!r}, expected {checkpoint.value!r} ± {checkpoint.tolerance}"
            )
    return failures


def reference_path(scenario: Scenario, platform: str) -> Path:
    return TRACES / platform / f"{scenario.name}.csv"


def compare_bytes(actual: Path, reference: Path) -> list[str]:
    if not reference.is_file():
        return [
            f"no reference trace at {reference.relative_to(REPO)}; "
            f"record one with: python scripts/golden.py record {actual.stem}"
        ]
    actual_bytes = actual.read_bytes()
    reference_bytes = reference.read_bytes()
    if actual_bytes == reference_bytes:
        return []
    # Report the first differing line rather than dumping the whole file: a
    # golden diff is usually one number drifting, and that is what to show.
    actual_lines = actual_bytes.decode("utf-8").splitlines()
    reference_lines = reference_bytes.decode("utf-8").splitlines()
    if len(actual_lines) != len(reference_lines):
        return [
            f"trace length differs: {len(actual_lines)} rows, reference has "
            f"{len(reference_lines)}"
        ]
    for index, (a, b) in enumerate(zip(actual_lines, reference_lines), start=1):
        if a != b:
            return [f"trace differs from the reference at line {index}:\n  got {a}\n  ref {b}"]
    return ["traces differ in trailing bytes"]


def cmd_list(_args: argparse.Namespace) -> int:
    width = max(len(scenario.name) for scenario in SUITE)
    for scenario in SUITE:
        print(f"{scenario.name.ljust(width)}  {scenario.duration:>5.1f}s  {scenario.exercises}")
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    scenario = BY_NAME[args.scenario]
    trace = run_scenario(scenario, Path(args.out))
    print(trace)
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    scenario = BY_NAME[args.scenario]
    trace = Path(args.trace)
    failures = compare_bytes(trace, reference_path(scenario, platform_key(args.platform)))
    failures += check_checkpoints(scenario, trace)
    for failure in failures:
        print(f"FAIL {failure}", file=sys.stderr)
    if not failures:
        print(f"ok {scenario.name}: bit-identical, {len(scenario.checkpoints)} checkpoint(s)")
    return 1 if failures else 0


def cmd_record(args: argparse.Namespace) -> int:
    platform = platform_key(args.platform)
    names = [args.scenario] if args.scenario else [s.name for s in SUITE]
    for name in names:
        scenario = BY_NAME[name]
        trace = run_scenario(scenario, REPO / "build" / "golden-out")
        destination = reference_path(scenario, platform)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(trace.read_bytes())
        print(f"recorded {destination.relative_to(REPO)}")
    return 0


def run_file(path: Path, duration: float, dt: float, out: Path) -> Path:
    """Runs one scenario file — either language — and returns its trace."""
    out.mkdir(parents=True, exist_ok=True)
    trace = out / f"{path.stem}.csv"
    result = subprocess.run(
        [
            str(scena_run_binary()),
            str(path),
            "--dt",
            repr(dt),
            "--duration",
            repr(duration),
            "--trace",
            str(trace),
            "--quiet",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise SystemExit(f"{path.name}: scena-run exited {result.returncode}\n{result.stderr}")
    return trace


def check_pair(pair: Pair, out_dir: Path) -> list[str]:
    """Both languages, one runtime: the traces must be equal byte for byte."""
    left = run_file(SCENARIOS / pair.dsl, pair.duration, pair.dt, out_dir)
    right = run_file(SCENARIOS / pair.xml, pair.duration, pair.dt, out_dir)
    left_bytes = left.read_bytes()
    right_bytes = right.read_bytes()
    if left_bytes == right_bytes:
        return []
    if len(left_bytes) != len(right_bytes):
        return [
            f"{pair.name}: {pair.dsl} is {len(left_bytes)} bytes, "
            f"{pair.xml} is {len(right_bytes)}"
        ]
    for index, (a, b) in enumerate(zip(left_bytes, right_bytes)):
        if a != b:
            line = left_bytes[:index].count(b"\n") + 1
            return [
                f"{pair.name}: the two languages diverge at line {line}\n"
                f"     {pair.dsl}: {left_bytes.splitlines()[line - 1].decode()}\n"
                f"     {pair.xml}: {right_bytes.splitlines()[line - 1].decode()}"
            ]
    return []


def cmd_compare_pair(args: argparse.Namespace) -> int:
    pairs = [p for p in PAIRS if not args.pair or p.name == args.pair]
    out_dir = REPO / "build" / "golden-out"
    failures: list[str] = []
    for pair in pairs:
        pair_failures = check_pair(pair, out_dir)
        failures.extend(pair_failures)
        for failure in pair_failures:
            print(f"FAIL {failure}", file=sys.stderr)
        if not pair_failures:
            print(f"ok   {pair.name} pair  ({pair.dsl} == {pair.xml}, byte for byte)")
    return 1 if failures else 0


def cmd_check_all(args: argparse.Namespace) -> int:
    platform = platform_key(args.platform)
    out_dir = REPO / "build" / "golden-out"
    failures: list[str] = []
    for scenario in SUITE:
        trace = run_scenario(scenario, out_dir)
        scenario_failures = compare_bytes(trace, reference_path(scenario, platform))
        scenario_failures += check_checkpoints(scenario, trace)
        if scenario_failures:
            failures.extend(scenario_failures)
            print(f"FAIL {scenario.name}", file=sys.stderr)
            for failure in scenario_failures:
                print(f"     {failure}", file=sys.stderr)
        else:
            print(f"ok   {scenario.name}  ({len(scenario.checkpoints)} checkpoint(s))")

    for pair in PAIRS:
        pair_failures = check_pair(pair, out_dir)
        if pair_failures:
            failures.extend(pair_failures)
            print(f"FAIL {pair.name} pair", file=sys.stderr)
            for failure in pair_failures:
                print(f"     {failure}", file=sys.stderr)
        else:
            print(f"ok   {pair.name} pair  ({pair.dsl} == {pair.xml}, byte for byte)")
    print(f"\n{len(SUITE)} scenarios, {len(failures)} failure(s)", file=sys.stderr)
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="list the suite").set_defaults(func=cmd_list)

    run_parser = sub.add_parser("run", help="run one scenario")
    run_parser.add_argument("scenario", choices=sorted(BY_NAME))
    run_parser.add_argument("--out", default=str(REPO / "build" / "golden-out"))
    run_parser.set_defaults(func=cmd_run)

    verify_parser = sub.add_parser("verify", help="bit-compare and check one trace")
    verify_parser.add_argument("scenario", choices=sorted(BY_NAME))
    verify_parser.add_argument("trace")
    verify_parser.add_argument("--platform", default=None)
    verify_parser.set_defaults(func=cmd_verify)

    record_parser = sub.add_parser("record", help="(re)record reference traces")
    record_parser.add_argument("scenario", nargs="?", choices=sorted(BY_NAME), default=None)
    record_parser.add_argument("--platform", default=None)
    record_parser.set_defaults(func=cmd_record)

    pair_parser = sub.add_parser(
        "compare-pair", help="assert a declared XML/DSL pair traces identically"
    )
    pair_parser.add_argument("pair", nargs="?", default="")
    pair_parser.set_defaults(func=cmd_compare_pair)

    check_parser = sub.add_parser("check-all", help="run and verify the whole suite")
    check_parser.add_argument("--platform", default=None)
    check_parser.set_defaults(func=cmd_check_all)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
