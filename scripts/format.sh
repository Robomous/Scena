#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Formats all first-party C/C++ sources with clang-format.
# Usage: scripts/format.sh [--check]
#   --check  verify formatting without modifying files (non-zero exit on drift)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"

FILES=$(git ls-files '*.h' '*.hpp' '*.c' '*.cpp')
if [ -z "$FILES" ]; then
    echo "no C/C++ sources found"
    exit 0
fi

if [ "${1:-}" = "--check" ]; then
    # shellcheck disable=SC2086
    $CLANG_FORMAT --dry-run --Werror $FILES
    echo "formatting OK"
else
    # shellcheck disable=SC2086
    $CLANG_FORMAT -i $FILES
    echo "formatted $(echo "$FILES" | wc -l | tr -d ' ') files"
fi
