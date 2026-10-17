# kinematic/markers — interaction markers, body measurements (and, from A2, alignment)

Depends on `core/math`, `core/rig` and, privately, nlohmann/json. No JSON type may appear in a public header.

## Rules
- File format: `docs/formats/markers.md`. Change it and the loader, writer and round-trip tests together; bump `version` for incompatible changes.
- Loaders never throw and report file, 1-based line and JSON pointer. The writer is canonical: load(write(x)) == x and write(load(canonical)) is byte-identical.
- nlohmann/json is built with `JSON_Diagnostic_Positions` (set in `cmake/OpasDependencies.cmake`); line numbers depend on it.

## Pitfalls
- nlohmann/json rejects a number that overflows a double (e.g. `1e999`) with an `out_of_range` exception that has no position. The loader replaces such literals with a sentinel string and parses again, so the error still has its line and field.
- A helper named `quoted` collides with `std::quoted` through argument-dependent lookup on `std::string_view`; the writer uses `jsonString`.
- `-0` is written as `-0` but reads back as integer 0, so the next write gives `0`. Harmless, but not byte-identical.
