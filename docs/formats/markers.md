# Interaction marker files (`*.markers.json`)

Authored contact points for one paired animation. Read by `opas::markers::loadMarkerSet`, written by `writeMarkerSet`. JSON (RFC 8259), UTF-8.

## Example (canonical form)

```json
{
  "format": "opas.markers",
  "version": 1,
  "markers": [
    {
      "id": "rightHandOnShoulder",
      "sourceBone": "RightHand",
      "targetRegion": "LeftUpperArm",
      "offsetM": [0.02, -0.01, 0.12],
      "contactDirection": [0, 0, -1],
      "window": {"startS": 1.25, "endS": 2},
      "priority": 1,
      "toleranceM": 0.02
    }
  ]
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.markers"` |
| `version` | integer | exactly `1` |
| `markers` | array of marker objects | may be empty |
| `id` | string | non-empty, unique in the file |
| `sourceBone` | string | non-empty; a bone name on the acting actor |
| `targetRegion` | string | non-empty; a body region (a `CapsuleRule::region`) on the other actor |
| `offsetM` | array of 3 numbers | meters, in the target region's frame |
| `contactDirection` | array of 3 numbers | unit length within 1e-6 |
| `window` | object `{startS, endS}` | seconds; `0 <= startS <= endS` |
| `priority` | integer | any; higher wins on conflict |
| `toleranceM` | number | `> 0`, meters |

All fields are required. Unknown keys are errors (catches typos). Numbers must be finite.

## Errors

Every rejection reports file name, 1-based line and the field as a JSON pointer (e.g. `/markers/0/toleranceM`):
- malformed JSON: the line of the syntax error, field `""`;
- a bad value: the line where that value starts;
- a missing field: the line where the enclosing object starts, field = the missing field's pointer;
- an unknown key: the line where its value starts.

## Canonical form

`writeMarkerSet` output, which round-trips byte-for-byte: keys in the order above; 2-space indent; arrays of numbers and the `window` object on one line, as in the example; numbers in shortest round-trip form (`std::to_chars`), so `2.0` is written `2`; LF line endings; a trailing newline. An empty `markers` array is written `"markers": []`.
