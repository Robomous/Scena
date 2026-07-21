# Contributing to Kinema

Thanks for your interest in Kinema. This document covers how to build, test,
and submit changes. The `docs/` directory is the public source of truth for
architecture and process; when in doubt, the ADRs in `docs/architecture/` win.

## Building

Requirements: CMake ≥ 3.24, a C++20 compiler (tested with recent Clang, GCC,
and MSVC), Python ≥ 3.9 for the bindings. All third-party dependencies are
fetched by CMake and pinned in `cmake/Dependencies.cmake`.

```sh
cmake -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

`./scripts/build.sh` runs all three steps. Useful options:

| Option | Default | Meaning |
|--------|---------|---------|
| `KNM_BUILD_TESTS` | `ON` | Build the gtest suites |
| `KNM_BUILD_PYTHON` | `ON` | Build the nanobind Python module |
| `KNM_BUILD_CAPI` | `ON` | Build the C API shared library |
| `KNM_WARNINGS_AS_ERRORS` | `OFF` (ON in CI) | Treat warnings as errors |
| `KNM_SANITIZE` | empty | `address`, `undefined`, `thread` (comma-separated) |

Python bindings against the build tree (without installing):

```sh
PYTHONPATH=build/python python python/examples/hello_engine.py
```

## Testing

- Every change to `core/`, `frontends/`, or `capi/` needs unit tests; suites
  live in `core/tests/`.
- All tests must pass on all three platforms (CI enforces this), with zero
  warnings.
- Determinism is a contract: identical scenario + identical step sequence must
  produce bit-identical entity states. Never introduce wall-clock reads,
  unordered iteration that affects results, or randomness into the runtime.

## Style

- C++20, no compiler extensions. Formatting is enforced by clang-format
  (`.clang-format`); run `./scripts/format.sh` before committing, or
  `./scripts/format.sh --check` to verify.
- Namespace `kinema`; CMake options and macros prefixed `KNM_`.
- Every source file starts with `// SPDX-License-Identifier: MIT`.
- Implement standard behavior exclusively from the ASAM specifications; cite
  them by section number in comments (e.g. "per ASAM OpenSCENARIO XML 1.3
  §x.y"). Never copy, translate, or paraphrase code from other scenario
  engines, regardless of their license (see ADR-0002).
- Refer to external simulators and tools generically ("host simulator",
  "external reference player") — no third-party product names anywhere in the
  tree.

## Dependencies

MIT/BSD/Apache-2.0 compatible only; no GPL/LGPL anywhere, no MPL in the
runtime tree. Every new dependency must be pinned in
`cmake/Dependencies.cmake` and recorded in `THIRD_PARTY_LICENSES.md`, and
needs maintainer approval beforehand.

## Pull requests

- Keep PRs focused and within the current roadmap phase
  (`docs/roadmap/roadmap.md`); work beyond the active gate needs a maintainer
  discussion first.
- PRs must be green in CI (build, tests, sanitizers, formatting) before
  review.
- Architectural changes require an ADR in `docs/architecture/`.
- Releases, tags, and packaging are handled exclusively by the maintainer.
