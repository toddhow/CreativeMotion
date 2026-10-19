# Task: M4 Headless runner, record and replay
Status: approved by toddhow on 2026-10-18 with all four proposals. Tolerances are 1 mm, 0.01 rad, 0.01 m/s and 0.05 rad/s, against goldens recorded on MSVC, with per-platform goldens as the fallback if Linux fails. All 13 numbers are recorded per body per step. Pose sources are named and built in. Each ragdoll has a `motors` option. Lighter loop: the implementing agent writes the tests and one fresh review runs at the end of the milestone.
Design doc section: CLAUDE.md → "First milestones" → M4, and "Testing and quality gates" (the Replay and Determinism gates). Golden recordings change only in a pull request that says why, reviewed by toddhow.
Goal: scenarios are data files. A headless runner plays them deterministically and writes a JSON recording of every body's state each step. CI replays the scenarios and compares them with golden recordings, so any change in physical behaviour is caught.
Inputs / outputs:
- New `sandbox/runner`:
  - library `opas::sandbox` with the scenario model and loader, the recording model, reader and canonical writer, and the runner;
  - executable `opas_run`.
- New `sandbox/scenarios/` with four scenario files, and `data/recordings/` with their golden recordings (synthetic only).
- New `docs/formats/scenario.md` and `docs/formats/recording.md`.
- Tests in `tests/unit/sandbox_runner/`, `tests/roundtrip/sandbox_runner/` and `tests/replay/`.
- The runner uses `core/motor` and the Jolt backend. Nothing depends on `sandbox` (guide).
Constraints:
- **Deterministic:** no wall-clock time, no randomness, fixed step. Time is the step count × dt.
- **Loaders:** never throw, and report file, line and JSON pointer, as in the other formats (shared `core/format` support).
- **Writers:** canonical, so files round-trip byte-for-byte.
- **Recording size:** under 2 MB per golden file, to keep the repository small.

## Scenario file (`*.scenario.json`, format `"opas.scenario"`, version 1)
```json
{
  "format": "opas.scenario",
  "version": 1,
  "name": "push",
  "stepsPerSecond": 60,
  "durationS": 3,
  "gravityMps2": [0, 0, -9.81],
  "statics": [
    {"name": "floor", "halfExtentsM": [20, 20, 0.5], "translationM": [0, 0, -0.5], "rotation": [0, 0, 0, 1], "friction": 0.8}
  ],
  "ragdolls": [
    {
      "name": "hero",
      "rig": "../../data/rigs/humanoid.rig.json",
      "profile": "../../data/profiles/humanoid-default.profile.json",
      "translationM": [0, 0, 0.02],
      "rotation": [0, 0, 0, 1],
      "rootMotion": "dynamic",
      "pose": "rest"
    }
  ],
  "impulses": [
    {"atS": 0.5, "ragdoll": "hero", "bone": "Chest", "impulseNs": [0, 60, 0], "offsetM": [0, 0, 0]}
  ]
}
```
- **Statics** are boxes only, with a centre and a rotation. A floor, stairs (several boxes) and a slope (one rotated box) all fit, and more shapes can come later.
- **Ragdolls:** paths are relative to the scenario file. `pose` names a pose source built into the runner:
  - `"rest"` holds the rest pose;
  - `"armWave"` is M3's wave.
  Pose sources are behaviour, not character data, so they live in code with a name. Unknown names are load errors.
- **Impulses** happen at the first step whose start time is at least `atS`. Each is applied to the named body, at its centre plus `offsetM` in world axes, before the step.
- **Step count:** `durationS` × `stepsPerSecond`, rounded to the nearest step.

## Recording file (`*.recording.json`, format `"opas.recording"`, version 1)
```json
{
  "format": "opas.recording",
  "version": 1,
  "scenario": "push",
  "stepsPerSecond": 60,
  "bodies": ["hero/Pelvis", "hero/Spine"],
  "steps": [
    [[0, 0, 1.01, 0, 0.7071068, 0, 0.7071068, 0, 0, 0, 0, 0, 0], [0, 0, 1.11, 0, 0.7071068, 0, 0.7071068, 0, 0, 0, 0, 0, 0]]
  ]
}
```
- **Steps:** one entry per step after the step, plus the initial state as entry 0.
- **Body order:** bodies appear in the order of `bodies`: ragdolls in scenario order, each in rig body order. Statics are not recorded.
- **State per body:** 13 numbers, all `float` values as the physics interface reports them: position (3), rotation `x, y, z, w` (4), linear velocity (3) and angular velocity (3).
- **Numbers:** written in the shortest form that reads back as the same `float`, so a recording round-trips exactly.
- **Canonical form:** one step per line.

## Runner
- **Library:** `RunResult run(const Scenario&, const RunOptions&)` builds the world, statics and ragdolls, registers one `RagdollDriver` per ragdoll, applies impulses and steps. It returns the recording, or a located error such as a missing rig file or an unknown bone.
- **Command line:**
  - `opas_run --scenario <file> --record <out>` writes a recording;
  - `opas_run --scenario <file> --compare <golden>` runs and compares.
  Exit codes: 0 = within tolerance, 1 = differs (the first differing step, body and field are printed), 2 = bad arguments or files.
- **Comparison:** for every step and body,
  - position within `positionToleranceM`;
  - rotation within `rotationToleranceRad`, as the angle between them, treating q and −q as the same;
  - velocities within `linearVelocityToleranceMps` and `angularVelocityToleranceRadps`.
  The defaults are listed under "Questions".

## Scenarios (`sandbox/scenarios/`)
1. **`floor`:** the sample ragdoll, limp (motors damping only, through a scenario-level option), released standing 2 cm above the floor. It falls and settles. 4 s.
2. **`stairs`:** five 0.17 m steps that are 0.30 m deep. The ragdoll holds its rest pose and falls down them from the top step. 4 s.
3. **`slope`:** a 20° slope. The ragdoll is released lying on it and slides or rolls. 4 s.
4. **`push`:** the ragdoll stands holding its rest pose on the floor (it will still topple without M5's balance) and gets a 60 N·s chest push at 0.5 s. 3 s.
At 60 Hz each recording is about 16 bodies × 13 numbers × 241 steps, roughly 50k numbers or 0.6 MB.

## Gates (ctest, so every CI job runs them)
- **Replay:** each scenario is run and compared with `data/recordings/<name>.recording.json`.
- **Determinism:** each scenario is run twice in one process and the two recordings must be identical bit for bit.
- **Round-trip:** the scenario and recording files load and write back byte-for-byte.

## Acceptance criteria
- [ ] `docs/formats/scenario.md` and `docs/formats/recording.md` document every field, error and canonical form, in the style of `rig.md`.
- [ ] The four scenario files load. The loader rejects, with file, line and pointer:
  - syntax errors, unknown keys and missing fields;
  - a wrong `format` or `version`;
  - `stepsPerSecond` ≤ 0, or `durationS` ≤ 0 or over 60;
  - non-unit rotations and non-positive box extents;
  - duplicate ragdoll or static names;
  - an unknown pose source or `rootMotion`;
  - an impulse naming an unknown ragdoll, or with `atS` outside the duration.
  An unknown bone and a missing rig file are reported by `run` with the scenario's line for that entry.
- [ ] Recordings round-trip byte-for-byte, and float values survive exactly, including −0 written as `0`, denormals and the largest float.
- [ ] `opas_run` writes recordings, compares with exit codes 0, 1 and 2, and prints the first difference. This is tested through ctest, like `opas_validate_markers`.
- [ ] The Replay gate passes on Windows/MSVC, against goldens recorded there.
- [ ] The Determinism gate passes for all four scenarios.
- [ ] Changing behaviour makes the Replay gate fail: a test runs `push` with a 61 N·s push and expects a difference.
- [ ] Each golden file is under 2 MB.
- [ ] The module has its own `CLAUDE.md`, a `data/recordings/README.md` says how to regenerate goldens and that changes need toddhow's review, and all existing tests still pass.

Out of scope:
- The debug viewer (M6).
- Sensors and balance (M5).
- Recording joint targets or contacts (later versions of the format).
- Playback speed control.
- Any OpenMW code.

## Questions for toddhow
1. **Tolerances and platforms:** Jolt is built cross-platform deterministic so goldens can be shared (`docs/dependencies.md`). Our own `double` math, such as `std::sin` in pose sources, may still differ in the last bit between MSVC and glibc. A falling ragdoll is chaotic, so a tiny difference can grow.
   Proposal: default tolerances of 1 mm, 0.01 rad, 0.01 m/s and 0.05 rad/s. Shared goldens are recorded on Windows/MSVC. If the first Linux CI run fails the Replay gate, we switch to per-platform goldens (`<name>.<platform>.recording.json`); the decision is yours then. Is that right?
2. **Recording content:** every body, every step, all 13 numbers (position, rotation, both velocities)? Or positions and rotations only, which roughly halves the file size?
3. **Pose sources:** are named built-in sources (`rest`, `armWave`) fine for M4? Animation clips as pose sources come with the animation runtime later.
4. **Limp option:** for the `floor` scenario, add a scenario-level `"motors": "off" | "profile"` per ragdoll, or ship a separate limp profile file in `data/profiles/`?

## Implementation notes (2026-10-19)
- **Scenario details beyond the spec, for toddhow to confirm:**
  - The stairs ragdoll gets a 30 N·s forward chest push at 0.2 s. Standing still on the top step, it would topple in whichever direction small asymmetries pick, not necessarily down the stairs.
  - The slope has friction 0.1. Jolt combines friction as the geometric mean (`ContactConstraintManager.h`, `mCombineFriction`), so 0.5 under the ragdoll's 0.8 would grip at 20°.
  - A floor sits under the slope's lower edge.
  - The ragdoll starts lying face down, held 0.22 m clear of the slope surface.
- **Goldens:** recorded on Windows/MSVC Debug. Release reproduces them byte for byte (`OpasRun.RecordedFileEqualsTheGolden`, Windows only).
- **Milestone review (fresh subagent), findings fixed:**
  - Recording floats are now read from their own text with `std::from_chars`. Reading through double rounded twice, and one float, 7.038531e-26, came back one unit in the last place off; it is now in the edge-value test.
  - The loader rejects an impulse that would fall after the last step, and a duration shorter than half a step.
  - New tool tests: the first per-step difference is printed, the recorded file is byte-identical to the golden on Windows, and an option given twice is an error. `armWave` is now exercised in process.
  - Smaller fixes: static-box errors carry their line, `opas_run` checks the write after closing the file, and a stale comment on world capacity is corrected.
- **Open:**
  - toddhow to confirm the scenario details above.
  - The first Linux CI run will show whether the shared MSVC goldens hold within tolerance, or whether the approved fallback (per-platform goldens) is needed.
