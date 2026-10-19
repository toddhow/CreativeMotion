# Scenario files (`*.scenario.json`)

A scenario is a world for the headless runner to play: its step rate, length and gravity, static boxes, ragdolls with a pose to hold, and impulses at set times. It is read by `opas::sandbox::loadScenario` and written by `writeScenario`. JSON (RFC 8259), UTF-8, SI units, Z up. The four scenarios that feed the replay gate are in `sandbox/scenarios/`.

## Example (canonical form)

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
      "motors": "profile",
      "pose": "rest"
    }
  ],
  "impulses": [
    {"atS": 0.5, "ragdoll": "hero", "bone": "Chest", "impulseNs": [0, 60, 0], "offsetM": [0, 0, 0]}
  ]
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.scenario"` |
| `version` | integer | `1` |
| `name` | string | non-empty; copied into the recording |
| `stepsPerSecond` | integer | in `[1, 10000]`; the fixed physics step is its inverse |
| `durationS` | number | in `(0, 60]`; the run has `durationS × stepsPerSecond` steps, rounded to the nearest |
| `gravityMps2` | array of 3 numbers | world gravity |
| `statics` | array | required; may be empty |
| static `name` | string | non-empty, unique among statics |
| static `halfExtentsM` | array of 3 numbers | each `> 0`; statics are boxes (floors, stair steps, slopes) |
| static `translationM`, `rotation` | 3 numbers; 4 numbers `[x, y, z, w]` | the box's centre and rotation in the world; the rotation is unit within 1e-6 |
| static `friction` | number | `>= 0`. Jolt combines two bodies' friction as the geometric mean (`ContactConstraintManager.h`, `mCombineFriction`) |
| `ragdolls` | array | at least 1 |
| ragdoll `name` | string | non-empty, unique among ragdolls; prefixes the body names in the recording |
| ragdoll `rig`, `profile` | string | paths to a rig and a profile file, relative to the scenario file's folder |
| ragdoll `translationM`, `rotation` | as for statics | where the model root sits |
| ragdoll `rootMotion` | string | `"dynamic"` or `"kinematic"` (`opas::motor::RootMotion`) |
| ragdoll `motors` | string | `"profile"` (the profile's gains) or `"off"` (stiffness 0 on every joint, damping kept: limp) |
| ragdoll `pose` | string | a built-in pose source: `"rest"` (the rig's rest pose) or `"armWave"` (M3's arm wave, needs bones `RightUpperArm` and `RightForearm`) |
| `impulses` | array | required; may be empty |
| impulse `atS` | number | in `[0, durationS)` |
| impulse `ragdoll` | string | the `name` of a ragdoll in this file |
| impulse `bone` | string | a bone with a body in that ragdoll's rig, checked when the scenario runs |
| impulse `impulseNs` | array of 3 numbers | the impulse, world axes |
| impulse `offsetM` | array of 3 numbers | from the body's centre to where the impulse is applied, world axes |

All fields are required, unknown keys are errors, and numbers must be finite.

## Timing

- **Impulses:** an impulse is applied before the first step whose start time (step index ÷ `stepsPerSecond`) is at least `atS`. At 60 Hz, `atS` 0.5 is step 30. Impulses due at the same step are applied in file order.
- **Pose sources:** each ragdoll is driven by its pose source from a pre-step listener (`opas::motor::RagdollDriver`). The source is asked for the pose at the end of each step.

## Errors

The usual rules apply (`docs/formats/markers.md`): file name, 1-based line and JSON pointer.

- **Duplicates:** a duplicate name is reported at the second occurrence.
- **Unknown names:** an unknown pose source is reported at `pose`, and an impulse naming an unknown ragdoll at its `ragdoll`.
- **Errors found while running:** a missing or invalid rig or profile file, or an impulse on a bone with no body, is reported by `opas::sandbox::run`. The error points at the scenario entry's line and pointer, and its message includes the inner file's own located error.

## Canonical form

`writeScenario` output round-trips byte-for-byte:

- keys in the order above, with 2-space indent;
- each static and each impulse on one line;
- each ragdoll on multiple lines with one key per line;
- numbers in shortest round-trip form, with negative zero written `0`;
- LF line endings and a trailing newline;
- an empty array written `[]`.
