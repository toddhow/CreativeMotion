# Task: A3a Body map file format
Status: implemented and reviewed (part of the approved A3 design, decision 5). Linux CI not yet run: not pushed yet.
Design doc section: CLAUDE.md → "Alignment plugin track" → A3; CreativeAlignment `docs/tasks/0001-a3-first-release.md`, decision 5.
Goal: Load and write `BodyMapping` (limbs, capsules, IK chains) as JSON, so the plugin can ship the Skyrim humanoid's mapping as data.
Inputs / outputs: `docs/formats/bodymap.md`; `kinematic/markers/include/opas/markers/BodyMapFile.hpp` (`loadBodyMap`, `loadBodyMapFile`, `writeBodyMap`); implementation in `kinematic/markers/src/`. The JSON reading helpers shared with marker files may move to an internal header.
Constraints: as for marker files: never throws; errors carry file, 1-based line and JSON pointer; canonical writer; nlohmann/json stays private.

## Acceptance criteria
- [x] Format documented in `docs/formats/bodymap.md`.
- [x] Round-trip: load → write of a canonical file is byte-identical; write → load gives an equal `BodyMapping` (including a seeded random property test).
- [x] The synthetic humanoid's mapping (from the A2 tests) written and loaded back works with `measureBody` and `alignPair` exactly as the original.
- [x] Each rule in the format doc's table is rejected with the documented line and pointer: wrong `format` or `version`, missing field, wrong type, unknown key, empty string, a limb with fewer than 2 bones, duplicate limb name, capsule region, chain name or chain `endBone`, `radiusRatio <= 0`, bend limits out of order or range, a zero or non-finite pole, `limbRadiusM <= 0`, non-finite numbers, malformed JSON.
- [x] `loadBodyMapFile` never throws (missing file, non-ASCII path).
- [x] Existing marker tests still pass (shared reader refactor).
- [x] All presets pass on Windows; format and lint clean.

Out of scope: checking bone names against a skeleton (done when used); the Skyrim humanoid body map itself (it ships in CreativeAlignment).
