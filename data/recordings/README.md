# Golden recordings

One recording per scenario in `sandbox/scenarios/`, used by the replay gate (`opas_run --compare`, run by ctest in every CI job). They are synthetic: the sample humanoid in generated worlds, no game data.

- **Platform:** recorded on Windows/MSVC (Debug and Release give the same result). Linux runs compare against the same files within the tolerances in `docs/formats/recording.md`. If Linux CI ever fails the gate, the fallback approved in `docs/tasks/0015` is per-platform goldens.
- **Changing them:** a golden changes only in a pull request that says why, reviewed by toddhow (CLAUDE.md, "Testing and quality gates"). Regenerate one with:

  ```
  opas_run --scenario sandbox/scenarios/<name>.scenario.json --record data/recordings/<name>.recording.json
  ```

| Scenario | What happens |
| --- | --- |
| `floor` | A limp ragdoll (motors off) released standing 2 cm above the floor collapses and settles. 4 s. |
| `stairs` | Standing on the top of five 0.17 m steps, holding its rest pose, pushed forward 30 N·s at 0.2 s, it falls down the stairs. 4 s. |
| `slope` | Released lying face down on a 20° slope with friction 0.1, it slides to the bottom and stops on the floor. 4 s. |
| `push` | Standing on the floor holding its rest pose, it is pushed 60 N·s at the chest at 0.5 s and falls (no balance until M5). 3 s. |
