# Rig files (`*.rig.json`)

A rig describes a character's physical body: the skeleton that animates, the rigid bodies attached to some of its bones, and the swing-twist joints between those bodies. It is the data form of `opas::rig::Rig`. It is read by `opas::rig::loadRig` and written by `writeRig`. JSON (RFC 8259), UTF-8. How the body *behaves* (motor gains, materials) is in a separate profile file (`profile.md`).

Units are SI (meters, kilograms, radians), in a right-handed frame with Z up. Hosts that use other units (OpenMW: 1 m ≈ 69.99 units) convert when they build the ragdoll; the file never holds host units.

## Example (canonical form)

```json
{
  "format": "opas.rig",
  "version": 1,
  "name": "two-link",
  "bones": [
    {"name": "Root", "parent": "", "translationM": [0, 0, 1], "rotation": [0, 0, 0, 1]},
    {"name": "Arm", "parent": "Root", "translationM": [0, 0, -0.3], "rotation": [0, 0, 0, 1]}
  ],
  "bodies": [
    {
      "bone": "Root",
      "shape": {"type": "box", "halfExtentsM": [0.1, 0.1, 0.1]},
      "translationM": [0, 0, 0],
      "rotation": [0, 0, 0, 1],
      "massKg": 5
    },
    {
      "bone": "Arm",
      "shape": {"type": "capsule", "halfHeightM": 0.1, "radiusM": 0.04},
      "translationM": [0, 0, -0.14],
      "rotation": [0, 0, 0, 1],
      "massKg": 1.5
    }
  ],
  "joints": [
    {
      "bone": "Arm",
      "twistAxis": [1, 0, 0],
      "planeAxis": [0, 1, 0],
      "normalHalfConeRad": 0,
      "planeHalfConeRad": 0,
      "twistMinRad": 0,
      "twistMaxRad": 2.6
    }
  ]
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.rig"` |
| `version` | integer | `1` |
| `name` | string | non-empty; what a profile's `rig` names |
| `bones` | array | at least 1 |
| bone `name` | string | non-empty, unique among bones |
| bone `parent` | string | `""` for a root, else the `name` of a bone listed **earlier** (parents first) |
| bone `translationM` | array of 3 numbers | rest position relative to the parent bone (or the model root) |
| bone `rotation` | array of 4 numbers `[x, y, z, w]` | rest rotation relative to the parent; unit length within 1e-6 |
| `bodies` | array | at least 1 |
| body `bone` | string | a bone in this file; at most one body per bone |
| body `shape` | object | one of the shapes below |
| body `translationM`, `rotation` | as for bones | the body's centre and orientation in its bone's frame |
| body `massKg` | number | `> 0` |
| `joints` | array | required; may be empty (a one-body rig) |
| joint `bone` | string | a bone that has a body; at most one joint per bone |
| joint `twistAxis`, `planeAxis` | arrays of 3 numbers | unit length within 1e-6, perpendicular within 1e-6, in the joint bone's frame |
| joint `normalHalfConeRad`, `planeHalfConeRad` | number | in `[0, pi]` |
| joint `twistMinRad`, `twistMaxRad` | number | in `[-pi, pi]`, `twistMinRad <= twistMaxRad` |

Shapes (each object has exactly these keys):

| `type` | Other keys | Rule |
| --- | --- | --- |
| `"capsule"` | `halfHeightM`, `radiusM` | both `> 0`; the axis is the body's local Z; total length `2 * (halfHeightM + radiusM)` |
| `"box"` | `halfExtentsM` (array of 3 numbers) | each `> 0` |
| `"sphere"` | `radiusM` | `> 0` |

All fields are required, unknown keys are errors, and numbers must be finite.

## Joints and the body tree

- **What a joint connects:** a joint is named by its child bone. It connects that bone's body to the body of the nearest ancestor bone that has one. Bones in between without bodies (a neck) are skipped. The rig therefore cannot name a parent body that disagrees with the skeleton.
- **Anchor:** the joint bone's origin in the rest pose.
- **Joint type:** a swing-twist joint, as in `opas::physics::JointDesc` (M1).
  - The twist axis is the joint's main axis. Twist is rotation about it, positive by the right-hand rule, from `twistMinRad` to `twistMaxRad`.
  - Swing is limited by an elliptical cone around the twist axis. `planeHalfConeRad` is measured in the plane that contains `planeAxis`, and `normalHalfConeRad` in the plane normal to it.
  - A hinge (knee, elbow) is a joint with both cones `0`, the hinge axis as its twist axis, and the bend range as its twist limits.
- **Root body:** exactly one body has no ancestor with a body. Normally it is on the pelvis. It has no joint.
- **Every other body has exactly one joint.** A joint on a bone without a body is an error.
- **Bones without bodies** (fingers, twist bones) animate but are not simulated.
- **Limitation of version 1:** the swing cone is centred on the rest pose, so a range that is wider one way than the other (hip flexion 120° but extension 30°) can only be approximated. See `data/rigs/README.md`.

## Errors

Same rules as the other formats (`docs/formats/markers.md`): file name, 1-based line and JSON pointer, e.g. `/joints/2/planeAxis`.

- **Where errors point:** a bad value is reported where it starts. A missing field is reported where its enclosing object starts, and an unknown key where its value starts.
- **Duplicates:** a duplicate bone, a second body on a bone, or a second joint on a bone is reported at the second occurrence.
- **Parents:** a parent that is unknown, the bone itself, or listed after the bone is reported at that bone's `parent`.
- **Twist limits:** `twistMinRad > twistMaxRad` is reported at `twistMinRad`.
- **Root bodies:** a second root body is reported at its `bone`.
- **Missing joints:** a body that needs a joint but has none is reported at the body object.

Checking order:

1. Header, `name`, then each bone: types and finite numbers, unit rotations.
2. Bone names and parents.
3. Each body, then each joint, in file order: types first, then ranges.
4. The body tree last.

## Canonical form

`writeRig` produces the canonical form, which round-trips byte-for-byte:

- keys in the order above, with 2-space indent;
- each bone object on one line;
- each body and joint object on multiple lines with one key per line;
- the shape object on one line;
- arrays of numbers on one line;
- numbers in shortest round-trip form, with negative zero written `0`;
- LF line endings and a trailing newline;
- an empty `joints` array written `[]`.
