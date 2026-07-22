#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Convenience wrapper: configure + build + test.
# Usage: scripts/build.sh [extra cmake configure args...]
# Environment: BUILD_DIR (default: build)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BUILD_DIR="${BUILD_DIR:-build}"

cmake -B "$BUILD_DIR" "$@"
cmake --build "$BUILD_DIR" --parallel
ctest --test-dir "$BUILD_DIR" --output-on-failure
