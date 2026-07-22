#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Forbids raw libm transcendentals in runtime code (core/). Transcendental
# math must go through scena::runtime detmath (det_sin/det_cos/det_sincos),
# which is bit-identical across platforms. IEEE-exact functions (sqrt, fabs,
# floor, ceil, ...) are allowed and are not matched.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Portable ERE (works on both BSD and GNU grep): manual word boundary via a
# non-identifier lead character, no \b.
PATTERN='(^|[^[:alnum:]_])(std::)?(sin|cos|tan|asin|acos|atan|atan2|sincos|sinh|cosh|tanh|asinh|acosh|atanh|exp|exp2|expm1|log|log2|log10|log1p|pow|cbrt|hypot|erf|erfc|tgamma|lgamma)(f|l)?[[:space:]]*\('

FILES=$(git ls-files 'core/src/**' 'core/include/**' \
    | grep -E '\.(h|hpp|c|cpp)$' \
    | grep -v '^core/src/runtime/detmath\.cpp$' || true)
if [ -z "$FILES" ]; then
    echo "no runtime sources found"
    exit 0
fi

# `if VIOLATIONS=$(grep ...)` keeps grep's exit-1-on-no-match from tripping
# `set -e`: no match is the success path.
# shellcheck disable=SC2086
if VIOLATIONS=$(grep -nE "$PATTERN" $FILES); then
    echo "error: raw libm transcendentals are forbidden in runtime code."
    echo "Use det_sin/det_cos/det_sincos (scena/runtime/detmath.h):"
    echo "$VIOLATIONS"
    exit 1
fi
echo "detmath guard OK"
