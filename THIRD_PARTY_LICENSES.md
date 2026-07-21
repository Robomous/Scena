# Third-party licenses

Every dependency of Kinema, its exact pinned version, license, and where it
is used. All dependencies must be MIT/BSD/Apache-2.0 compatible; GPL and LGPL
are excluded permanently, and MPL is excluded from the runtime dependency
tree. Update this file whenever `cmake/Dependencies.cmake` changes.

| Dependency | Version | License      | Scope                                    | Source |
|------------|---------|--------------|------------------------------------------|--------|
| googletest | v1.15.2 | BSD-3-Clause | Tests only; never linked into shipped libraries | <https://github.com/google/googletest> |
| pugixml    | v1.14   | MIT          | XML frontend only (wired, unused until the XML frontend phase) | <https://github.com/zeux/pugixml> |
| nanobind   | v2.13.0 | BSD-3-Clause | Python bindings only                     | <https://github.com/wjakob/nanobind> |

The `kinema-core` runtime library has **zero** third-party runtime
dependencies.
