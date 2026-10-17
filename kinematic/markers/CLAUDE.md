# kinematic/markers — interaction markers, body measurements (and, from A2, alignment)

Depends on `core/math`, `core/rig`, `kinematic/ik` (alignment) and, privately, nlohmann/json. No JSON type may appear in a public header.

## Rules
- File format: `docs/formats/markers.md`. Change it and the loader, writer and round-trip tests together; bump `version` for incompatible changes.
- Loaders never throw and report file, 1-based line and JSON pointer. The writer is canonical: load(write(x)) == x and write(load(canonical)) is byte-identical.
- nlohmann/json is built with `JSON_Diagnostic_Positions` (set in `cmake/OpasDependencies.cmake`); line numbers depend on it.

## Pitfalls
- nlohmann/json rejects a number that overflows a double (e.g. `1e999`) with an `out_of_range` exception that has no position. The loader replaces such literals with a sentinel string and parses again, so the error still has its line and field.
- A helper named `quoted` collides with `std::quoted` through argument-dependent lookup on `std::string_view`; the writer uses `jsonString`.
- Negative zero is written `0` (`number()` in JsonSupport.cpp): nlohmann/json reads `-0` as the integer 0, so writing `-0` broke byte-for-byte round trips.
- Alignment (`alignPair`) copies each actor's pose, bends limbs by turning IK world-space deltas back into local rotations, and recomputes world transforms. This is why IK chains must be direct parent-child bones.
- Penetration is a sum over capsule overlaps. The pole search keeps a candidate only if it is strictly better (by 1e-9 m), so on ties the smallest rotation and then the chain's own pole win.
- Shared JSON machinery (`src/JsonSupport.hpp/.cpp`: the reader base class, the parse-and-read driver with line numbers and the non-finite fallback, file reading, canonical number and string writing) serves both marker files and body maps. A new format derives a reader from `detail::JsonReader` and calls `detail::parseAndRead`.
- Body maps (`docs/formats/bodymap.md`) check structure only. Bone names are checked when a map meets a skeleton (`measureBody`, `alignPair`).
- When a host project already provides `nlohmann_json`, CreativeMotion uses it instead of fetching its own. That host build must enable `JSON_Diagnostic_Positions`, or loader errors lose their line numbers.
- Pose clips (`docs/formats/poseclip.md`) and detection profiles (`docs/formats/detection.md`) use the same JSON support. Contact detection (`ContactDetection.cpp`) follows the numbered rules in `docs/tasks/0006`; keep the two in step.
