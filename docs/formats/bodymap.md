# Body map files (`*.bodymap.json`)

Describe which bones of a skeleton form limbs, body capsules (regions) and IK chains. A body map is the data form of `opas::markers::BodyMapping`; skeletons name their bones differently, so this is data, not code. Read by `opas::markers::loadBodyMap`, written by `writeBodyMap`. JSON (RFC 8259), UTF-8.

## Example (canonical form)

```json
{
  "format": "opas.bodymap",
  "version": 1,
  "limbs": [
    {"name": "RightArm", "bones": ["RightUpperArm", "RightForearm", "RightHand"]}
  ],
  "capsules": [
    {"region": "RightForearm", "bone": "RightForearm", "endBone": "RightHand", "radiusRatio": 0.15}
  ],
  "ikChains": [
    {
      "name": "RightArm",
      "rootBone": "RightUpperArm",
      "midBone": "RightForearm",
      "endBone": "RightHand",
      "minBendRad": 0,
      "maxBendRad": 2.6,
      "poleDirection": [0, -1, 0],
      "limbRadiusM": 0.04
    }
  ]
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.bodymap"` |
| `version` | integer | `1`, or `2` when the file has `fingers` (A5c) |
| `limbs`, `capsules`, `ikChains` | arrays | required; may be empty |
| limb `name` | string | non-empty, unique among limbs |
| limb `bones` | array of strings | at least 2, each non-empty; proximal to distal |
| capsule `region` | string | non-empty, unique among capsules; what a marker's `targetRegion` names |
| capsule `bone`, `endBone` | string | non-empty |
| capsule `radiusRatio` | number | `> 0` |
| chain `name` | string | non-empty, unique among chains |
| chain `rootBone`, `midBone`, `endBone` | string | non-empty; `endBone` unique among chains |
| chain `minBendRad`, `maxBendRad` | number | `0 <= minBendRad <= maxBendRad <= pi` |
| chain `poleDirection` | array of 3 numbers | finite, not all zero; in the actor's root frame |
| chain `limbRadiusM` | number | `> 0`, meters |
| `fingers` | array | version 2 only, and then required |
| finger `name` | string | non-empty, unique among fingers |
| finger `chain` | string | the `name` of an IK chain in this file (the chain whose `endBone` is the finger's hand) |
| finger `bones` | array of strings | at least 2, knuckle first |
| finger `hingeAxis` | array of 3 numbers | not all zero; the axis each joint bends about, in that joint's own frame |
| finger `maxCurlRad` | array of numbers | one per bone, each in `[0, pi]` |

All fields are required; unknown keys are errors; numbers must be finite. Whether the named bones exist, and whether each chain's mid bone is below its root bone and its end bone below its mid bone (at any depth), is checked when the map is used with a skeleton (`measureBody`, `alignPair`), not here.

## Errors

Same rules as marker files (`docs/formats/markers.md`): file name, 1-based line, JSON pointer (e.g. `/ikChains/0/maxBendRad`). A bad value is reported where it starts; a missing field where its enclosing object starts; an unknown key where its value starts. A duplicate name, region or chain `endBone` is reported at the second occurrence. `minBendRad > maxBendRad` is reported at `minBendRad`.

## Canonical form

`writeBodyMap` output, which round-trips byte-for-byte: keys in the order above; 2-space indent; each limb and capsule object on one line; each IK chain object on multiple lines with one key per line; arrays of strings or numbers on one line; numbers in shortest round-trip form; LF line endings; a trailing newline. An empty array is written `[]`. `fingers` comes last, each finger on multiple lines like a chain; a map without fingers is written as version 1 and one with fingers as version 2, so a version 2 file with an empty `fingers` array loads but is not canonical.
