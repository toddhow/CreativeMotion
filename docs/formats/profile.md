# Profile files (`*.profile.json`)

A profile is the tuning for one rig: how strongly its joint motors pull towards the animated pose, and the surface material of its bodies. One rig can have several profiles (default, drunk, wounded). It is the data form of `opas::rig::Profile`. It is read by `opas::rig::loadProfile` and written by `writeProfile`. JSON (RFC 8259), UTF-8.

## Example (canonical form)

```json
{
  "format": "opas.profile",
  "version": 1,
  "rig": "humanoid",
  "defaults": {
    "motor": {"stiffnessNmPerRad": 300, "dampingNmsPerRad": 30, "maxTorqueNm": 500},
    "material": {"friction": 0.8, "restitution": 0, "linearDamping": 0.05, "angularDamping": 0.05}
  },
  "joints": [
    {"bone": "Spine", "motor": {"stiffnessNmPerRad": 600, "dampingNmsPerRad": 60, "maxTorqueNm": 900}}
  ],
  "bodies": [
    {"bone": "LeftFoot", "material": {"friction": 1, "restitution": 0, "linearDamping": 0.05, "angularDamping": 0.05}}
  ]
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.profile"` |
| `version` | integer | `1` |
| `rig` | string | non-empty; the `name` of the rig this profile is for |
| `defaults` | object | exactly `motor` and `material` |
| `joints` | array | required; may be empty |
| joint override `bone` | string | non-empty, unique among joint overrides; a bone with a joint in the rig |
| joint override `motor` | motor object | replaces the default motor for that joint |
| `bodies` | array | required; may be empty |
| body override `bone` | string | non-empty, unique among body overrides; a bone with a body in the rig |
| body override `material` | material object | replaces the default material for that body |

The motor object follows `opas::physics::JointMotor`: torque = stiffness × angle error − damping × relative angular velocity, clamped to `maxTorqueNm`.

| Motor key | Rule |
| --- | --- |
| `stiffnessNmPerRad` | `>= 0` |
| `dampingNmsPerRad` | `>= 0` (both `0` = motor off; the joint moves freely within its limits) |
| `maxTorqueNm` | `> 0` |

The material object follows `opas::physics::BodyDesc`.

| Material key | Rule |
| --- | --- |
| `friction` | `>= 0` |
| `restitution` | `>= 0` (bounciness; `0` = none) |
| `linearDamping`, `angularDamping` | `>= 0` (Jolt's own default is 0.05) |

All fields are required, unknown keys are errors, and numbers must be finite.

## Defaults and overrides

- **Whole objects only:** an override replaces the whole `motor` or `material` object of one bone. Fields are never merged one by one, so the values that apply to a bone are always written in one place. `motorFor` and `materialFor` return the override if there is one, else the default.
- **Checking against the rig:** whether `rig` matches and every overridden bone has a joint or body needs the rig, so `loadProfile` does not check it. Call `validateProfile(profile, rig)` once both are loaded. It reports a mismatched name at `/rig` and a bad override at its `bone`.
- **Sample values:** the gains in `data/profiles/humanoid-default.profile.json` are placeholders, to be tuned in M3 when tracking is measured.
- **Later versions:** M5 adds balance and hit-reaction sections as version 2. Version 1 files keep loading.

## Errors

Same rules as the other formats (`docs/formats/markers.md`): file name, 1-based line and JSON pointer, e.g. `/joints/0/motor/maxTorqueNm`.

- **Where errors point:** a bad value is reported where it starts. A missing field is reported where its enclosing object starts, and an unknown key where its value starts.
- **Duplicates:** a duplicate override is reported at the second occurrence's `bone`.
- **Checking order:** types and finite numbers are checked while reading, in file order. Ranges and duplicates are checked afterwards, in the order defaults, joints, bodies.

## Canonical form

`writeProfile` produces the canonical form, which round-trips byte-for-byte:

- keys in the order above, with 2-space indent;
- `defaults` on multiple lines, with its `motor` and `material` objects each on one line;
- each override object on one line;
- numbers in shortest round-trip form, with negative zero written `0`;
- LF line endings and a trailing newline;
- an empty array written `[]`.
