# sandbox/runner — headless runner, scenario and recording formats

Depends on everything it needs (`core/*`, `physics/jolt`); nothing depends on it. Spec: `docs/tasks/0015`. Formats: `docs/formats/scenario.md`, `docs/formats/recording.md`.

## Rules
- Deterministic: no wall-clock time, no randomness; time is step index × dt. The same scenario on the same build gives a bit-identical recording (`sameBits`).
- Golden recordings in `data/recordings/` change only in a pull request that says why, reviewed by toddhow (CLAUDE.md). Regenerate with `opas_run --scenario sandbox/scenarios/<name>.scenario.json --record data/recordings/<name>.recording.json`, from a Windows/MSVC build (the goldens' platform, decided in `docs/tasks/0015`).
- Pose sources are named and built in (`makePoseSource`); add a new one there and to `isPoseSourceName`, and document it in `scenario.md`.
- Run errors (missing rig, unknown impulse bone) are reported at the scenario entry's line, kept in `ScenarioRagdoll::line` / `Impulse::line` by the loader.

## Pitfalls
- Jolt combines friction as the geometric mean of the two bodies (`ContactConstraintManager.h`, `mCombineFriction`): a 0.5 slope under a 0.8-friction ragdoll grips (√0.4 ≈ 0.63 > tan 20°). The slope scenario uses 0.1.
- The runner keeps ragdolls in a vector sized once; `RagdollDriver` holds a pointer into it, so it must not be resized during a run. The world is destroyed before the ragdolls and drivers (declaration order in `run`).
- Recording floats are read as double by nlohmann/json and narrowed to float; the shortest float text reads back exactly (round-trip tests cover denormals, the largest float and random bit patterns).
