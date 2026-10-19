# Recording files (`*.recording.json`)

A recording holds the state of every ragdoll body at every step of a scenario run. The runner writes it (`opas_run --record`), and the replay gate compares runs with golden recordings in `data/recordings/` (`opas_run --compare`). It is read by `opas::sandbox::loadRecording` and written by `writeRecording`. JSON, UTF-8.

## Example (canonical form, shortened)

```json
{
  "format": "opas.recording",
  "version": 1,
  "scenario": "push",
  "stepsPerSecond": 60,
  "bodies": ["hero/Pelvis", "hero/Spine"],
  "steps": [
    [[0, 0, 1.01, 0, 0.7071068, 0, 0.7071068, 0, 0, 0, 0, 0, 0], [0, 0, 1.17, 0, 0.7071068, 0, 0.7071068, 0, 0, 0, 0, 0, 0]],
    [[0, 0, 1.0097275, 0, 0.7071068, 0, 0.7071068, 0, 0, -0.1635, 0, 0, 0], [0, 0, 1.1697275, 0, 0.7071068, 0, 0.7071068, 0, 0, -0.1635, 0, 0, 0]]
  ]
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.recording"` |
| `version` | integer | `1` |
| `scenario` | string | the scenario's `name` |
| `stepsPerSecond` | integer | `> 0` |
| `bodies` | array of strings | at least 1, unique; `"<ragdoll>/<bone>"`, ragdolls in scenario order, each in rig body order |
| `steps` | array | at least 1. Entry 0 is the state before the first step; entry k is the state after step k |
| step | array | one sample per body, in `bodies` order |
| sample | array of 13 numbers | position x, y, z (m); rotation x, y, z, w; linear velocity x, y, z (m/s); angular velocity x, y, z (rad/s). These are the `float` values `opas::physics::BodyState` reports, and each must be finite and within float range |

Statics are not recorded. Unknown keys are errors.

## Numbers

Every value is written in the shortest text that reads back as exactly the same `float` (`std::to_chars` for `float`). Negative zero is written `0`, as in every format here. A recording therefore round-trips value for value, and its canonical text byte for byte.

## Comparing

`compareRecordings(actual, expected, tolerances)` first requires the headers to match exactly: scenario, `stepsPerSecond`, `bodies` and the step count. It then reports the first step and body, in order, where any of these exceeds its tolerance:

| Check | Default tolerance |
| --- | --- |
| distance between the positions | 1 mm |
| angle between the rotations (q and −q are the same) | 0.01 rad |
| difference of linear velocities | 0.01 m/s |
| difference of angular velocities | 0.05 rad/s |

These defaults were approved by toddhow for M4 (`docs/tasks/0015`). `sameBits` checks bit-for-bit equality, which is what the determinism gate uses.

## Errors

The usual rules apply: file, line and JSON pointer, for example `/steps/12/3/7`. Pointers are built only when something fails, because a recording holds about 50,000 numbers.

## Canonical form

`writeRecording` output:

- header keys in the order above, `bodies` on one line;
- each step on its own line, indented four spaces;
- LF line endings and a trailing newline.
