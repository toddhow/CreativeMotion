# Task: A1 Kinematic foundations
Design doc section: CLAUDE.md → "Alignment plugin track" → A1.
Goal: The engine-independent building blocks the alignment solver (A2) needs: math types, a minimal skeleton, two-bone IK with joint limits, interaction markers with a JSON file format, and body measurements.
Inputs / outputs (new modules, each with a `CLAUDE.md`):
- `core/math` (headers only): `Vec3`, `Quat`, `Transform` in `double`.
- `core/rig`: minimal `Skeleton` (bone names, parents, rest transforms) — M2 extends it with files and profiles.
- `kinematic/ik`: `solveTwoBone`.
- `kinematic/markers`: `InteractionMarker`, `MarkerSet`, JSON load/write (`docs/formats/markers.md`), `measureBody`.
- Dependency: nlohmann/json v3.12.0 (MIT), used privately by `kinematic/markers` only, built with `JSON_Diagnostic_Positions` so errors carry line numbers.
Constraints:
- `kinematic/*` depends only on `core/math`, `core/rig` and the standard library (plus nlohmann/json privately in markers). No physics, game or framework types.
- Deterministic and allocation-light; no exceptions leave a public function (the JSON loader catches its own).
- `double` precision in `core/math` and `kinematic` (IK and alignment accuracy); conversion to the physics `float` types happens at the boundary later.
- Units SI, Z up, right-handed. Angles in radians.

## Contract (headers in this change)
- `opas/math/Vec3.hpp`, `Quat.hpp`, `Transform.hpp` — value types and free functions; see comments.
- `opas/rig/Skeleton.hpp` — `Bone`, `Skeleton`, `validateSkeleton`, `findBone`, `computeWorldRest`.
- `opas/ik/TwoBoneIk.hpp` — `TwoBoneChain`, `HingeLimits`, `TwoBoneResult`, `solveTwoBone`.
- `opas/markers/InteractionMarker.hpp`, `MarkerFile.hpp`, `BodyMeasurements.hpp`.

## Acceptance criteria
**Math**
- [ ] Hand-computed cases for dot, cross, length, normalize (zero vector stays zero), quaternion from axis-angle, multiply, rotate, conjugate, `angleBetween`, `fromTo` (including opposite vectors), transform compose/inverse/transformPoint (≤ 1e-12).
- [ ] Property: composing 1000 seeded random rotations with `normalized` after each keeps |q| within 1e-12 of 1; `angleBetween(q, -q) == 0` (shorter path).
**Skeleton**
- [ ] `validateSkeleton` rejects: empty name, duplicate name, parent index ≥ own index (parents must come first) or < −1, non-unit rotation — message names the bone and field.
- [ ] `computeWorldRest` matches hand-computed world transforms for a 3-bone chain with rotations.
**Two-bone IK**
- [ ] Property (seeded, ≥ 1000 cases each; bone lengths 0.1–1 m, limits random within [0, π]):
  - reachable targets (distance within the limit-dependent reach range) are reached: |end − target| ≤ 1e-9 m;
  - unreachable targets clamp cleanly: end lies on the ray root→target at exactly the nearest reachable distance (≤ 1e-9), no NaN;
  - bone lengths preserved (≤ 1e-9) and bend angle within limits (≤ 1e-9) in every case;
  - the mid joint lies on the pole side of the root→end line whenever the pole is not parallel to it.
- [ ] Hand-computed cases: straight-arm reach (bend 0), 90° bend, target at the root (no NaN; uses the current end direction), pole parallel to the target direction (falls back deterministically).
- [ ] `rootRotation`/`midRotation` applied to the original bone directions give the solved directions (≤ 1e-9).
- [ ] Invalid input (zero-length bone, limits outside [0, π] or min > max) returns `valid == false` and leaves positions unchanged.
**Interaction markers**
- [ ] Format documented in `docs/formats/markers.md`.
- [ ] Load → write of a canonical file is byte-for-byte identical; write → load gives an equal `MarkerSet` (round-trip, `tests/roundtrip/`).
- [ ] Invalid files are rejected with file name, 1-based line and JSON-pointer field, for: malformed JSON (line of the syntax error), wrong `format`, unsupported `version`, missing field, wrong type, unknown key, empty `id`, duplicate `id`, non-unit `contactDirection`, `startS` < 0 or > `endS`, `toleranceM` ≤ 0, non-finite numbers.
**Body measurements**
- [ ] For a synthetic humanoid skeleton (built in the test, no game data), limb lengths equal the hand-computed sums (≤ 1e-12) and capsules run between the named bones' world rest positions with radius = ratio × length.
- [ ] A mapping naming a missing bone, an empty limb, or a non-positive radius ratio is rejected with a message naming it.
**General**
- [ ] All presets build and pass on Windows locally; clang-format and clang-tidy clean; Jolt include check still passes.

Out of scope: chain (> 2 bone) IK, the alignment solver (A2), contact detection from clips (A4), skeleton/body-map file formats (M2 / later), any SKSE or game code.
