# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# Sanitizer support, controlled by the SCN_SANITIZE cache variable.
# Accepts a comma-separated list of: address, undefined, thread.
# Note: thread cannot be combined with address.

function(scn_enable_sanitizers target)
    if(SCN_SANITIZE STREQUAL "")
        return()
    endif()

    if(MSVC)
        # MSVC only supports AddressSanitizer.
        if(SCN_SANITIZE MATCHES "address")
            target_compile_options(${target} PRIVATE /fsanitize=address)
        endif()
        return()
    endif()

    string(REPLACE "," ";" _sanitizers "${SCN_SANITIZE}")
    set(_flags "")
    foreach(_sanitizer IN LISTS _sanitizers)
        if(_sanitizer STREQUAL "address" OR _sanitizer STREQUAL "undefined" OR _sanitizer STREQUAL "thread")
            list(APPEND _flags "-fsanitize=${_sanitizer}")
        else()
            message(FATAL_ERROR "Unknown SCN_SANITIZE value: ${_sanitizer}")
        endif()
    endforeach()

    target_compile_options(${target} PRIVATE ${_flags} -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE ${_flags})
endfunction()
