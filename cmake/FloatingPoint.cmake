# SPDX-License-Identifier: Apache-2.0
# Pins IEEE-strict floating-point evaluation for first-party targets:
# no FMA contraction, no fast-math. Bit-identity across ISAs (x64 vs arm64)
# depends on this — a * b + c must not fuse on one platform and round twice
# on another. See docs/architecture/ADR-0006 and docs/user-guide/determinism.md.

function(scn_set_fp_strictness target)
    if(MSVC)
        # VS2022 /fp:precise does not contract to FMA unless /fp:contract is
        # explicitly requested, so /fp:precise alone gives strict evaluation.
        target_compile_options(${target} PRIVATE /fp:precise)
    else()
        # GCC defaults to -ffp-contract=fast; Apple clang fuses on arm64.
        # Force contraction off so every platform rounds each operation.
        target_compile_options(${target} PRIVATE -ffp-contract=off)
    endif()
endfunction()
