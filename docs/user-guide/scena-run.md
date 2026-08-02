# `scena-run`

The headless execution CLI: load a scenario, step it at a fixed rate, write a
state trace. It is the validation vehicle for the release gate and a worked
example of an embedder — a *thin* consumer of the public API with no scenario
semantics of its own.

No visualization, by design (ADR-0001, library-first).

```sh
scena-run tests/golden/scenarios/gs1-cruise-baseline.xosc \
  --dt 0.01 --duration 12 --trace out/gs1.csv
```

## Options

| Option | Meaning |
|---|---|
| `--dt <seconds>` | fixed step size (default `0.01`) |
| `--duration <seconds>` | how long to run (default `10`) |
| `--trace <file>` | write the trace to `<file>` |
| `--trace-format <csv\|json>` | override the format inferred from the extension |
| `--map <file.xodr>` | road network, overriding the scenario's `RoadNetwork/LogicFile` |
| `--replay <entity>=<file>` | drive an entity from a recorded trace |
| `--select <alternative>` | choose a `one_of` alternative (DSL; p8-s2) |
| `--entry <scenario>` | the DSL scenario to run (`.osc` only) |
| `-I`, `--search-path <dir>` | where DSL imports are looked up (repeatable) |
| `--quiet` | do not print diagnostics |
| `-h`, `--help` | usage |

## Two frontends, one runtime

The file's extension picks the frontend: `.osc` is compiled by the
OpenSCENARIO DSL frontend, anything else by the OpenSCENARIO XML frontend.
Everything past the load — the engine, the gateway, the trace — sees only the
Scenario IR, so the options, the exit codes and the trace format are the same
either way.

```sh
scena-run cruise.osc --dt 0.01 --duration 12 --trace out/cruise.csv
```

A DSL file may declare several scenarios, and §7.7.2 leaves it to the
implementation which one runs. Scena runs the only one when there is only one
and reports the choice when there is not, because guessing would make the run
depend on declaration order:

```
error: the file declares more than one scenario; name the entry point (§7.7.2): demo::first demo::second
```

`--entry second` (or `--entry demo::second`) names it. `-I` adds a directory to
the import search path, exactly as it does for `scena-check`.

The road network works the same way in both languages: XML names it in
`RoadNetwork/LogicFile`, DSL in §8.5.4's `map_file` — written either as
`map.set_map_file("m.xodr")` or as `keep(my_map.map_file == "m.xodr")`. Both
are resolved relative to the scenario file, and `--map` overrides either.

Only whole steps run: `--duration 1 --dt 0.3` runs three steps, not three and a
third. A variable last step would make the tail of every trace depend on how the
duration happens to divide.

## Exit codes

| Code | Meaning |
|---|---|
| `0` | the run completed |
| `2` | bad command line |
| `3` | the scenario did not parse or did not validate |
| `4` | a step returned a non-Ok status |
| `5` | the trace could not be written |
| `6` | a `--replay` file was unreadable, malformed, or named an entity the scenario does not declare |
| `7` | the road network did not load |

They are distinct so a script can branch on *why* a run failed. Diagnostics go
to stderr with their path and rule id; the trace goes to the file.

## The trace format

CSV, one row per step per **engine-controlled** entity:

```
t,entity,x,y,z,heading,speed
0.01,ego,0.1,0,0,0,10
```

Host-controlled entities are polled, not published, so they do not appear — the
trace records what the engine decided, and what the host decided the host
already knows.

`--trace-format json` writes the same samples as a JSON object.

Every number is written with `std::to_chars`' shortest round-trip form: reading
it back yields the identical double, and the formatting is locale-independent,
so the same run writes the same bytes on every machine. That is what makes
"bit-identical trace" checkable from the text file, and it is what the golden
suite compares.

The file is written with LF line endings on every platform, for the same reason.

## Road networks

A scenario names its own map in `RoadNetwork/LogicFile`, resolved relative to
the scenario file; `scena-run` loads it with the OpenDRIVE backend and hands it
to the engine through `gateway::IRoadQuery` — the engine never sees a concrete
road representation (ADR-0003).

```sh
scena-run scenario.xosc --map other-network.xodr
```

`--map` overrides whatever the scenario declares, which is how a host points a
scenario at a different network without editing it. With neither, the engine
runs road-free and the flat-world lane model applies: lane-relative targets fall
back to a configurable default lane width, and absolute lane ids — which name
elements of a real network — are reported rather than guessed.

## Replaying a host-controlled entity

```sh
scena-run scenario.xosc --replay npc=recorded.csv --trace out.csv
```

`--replay` declares that **the host will drive that entity** — and switches it
to host-controlled before init. Control ownership belongs to the embedder, not
to the scenario file (ADR-0003, ADR-0017), and OpenSCENARIO XML has no way to
say "the host drives this one", so `scena-run` says it on the command line.

The replay file is a trace in the format above; rows for other entities are
ignored, so a multi-entity trace can drive one entity. When the recorded states
run out the entity holds its last one.

## The golden suite

`scripts/golden.py` drives `scena-run` over the committed acceptance scenarios
and validates two things:

- **bit-identity** against the committed reference trace — the primary
  criterion, and any diff is a release blocker;
- **semantic checkpoints** with per-checkpoint tolerances, which catch behaviour
  that is deterministically *wrong*.

```sh
python scripts/golden.py list                 # the suite and what each exercises
python scripts/golden.py check-all            # run and verify everything
python scripts/golden.py run gs1              # one scenario, write its trace
python scripts/golden.py verify gs1 out.csv   # compare + checkpoints
python scripts/golden.py record gs1           # (re)record the reference
```

There is **one** reference set, shared by every platform, not one per platform:
determinism is bit-identical across platforms, so a per-platform reference would
be a place for a divergence to hide. Comparing every platform against the same
bytes makes the golden CI job a cross-platform determinism check as well as a
regression check.

`record` overwrites a committed reference and is never run by CI — a reference
changes only when a human decided the behaviour should.

See [golden-scenarios.md](../roadmap/golden-scenarios.md) for the suite's
contents and the release-gate procedure.

## Further reading

- [Embedding Scena](embedding.md) — the step contract `scena-run` implements.
- [Loading scenarios](loading-scenarios.md) — what the loader accepts and reports.
- [Determinism](determinism.md) — what bit-identity means and what it rests on.
