# Pose clip files (`*.poseclip.json`)

Two actors' bone world transforms sampled over time: what the plugin records in game (A4) and what contact detection reads. Read by `opas::markers::loadPoseClip`, written by `writePoseClip`. JSON (RFC 8259), UTF-8. Recordings of real game animations stay local and are never committed (CLAUDE.md, game-derived data); tests use synthetic clips.

## Example (canonical form)

```json
{
  "format": "opas.poseclip",
  "version": 1,
  "sampleRateHz": 30,
  "actors": [
    {
      "bodyMap": "humanoid",
      "bones": ["Pelvis", "RightHand"],
      "frames": [
        [[0, 0, 1, 0, 0, 0, 1], [0.3, -0.2, 1.1, 0, 0, 0, 1]],
        [[0, 0, 1, 0, 0, 0, 1], [0.31, -0.2, 1.1, 0, 0, 0, 1]]
      ]
    },
    {
      "bodyMap": "humanoid",
      "bones": ["Pelvis"],
      "frames": [
        [[0.6, 0, 1, 0, 0, 1, 0]],
        [[0.6, 0, 1, 0, 0, 1, 0]]
      ]
    }
  ]
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.poseclip"` |
| `version` | integer | exactly `1` |
| `sampleRateHz` | number | `> 0`; frames are evenly spaced, frame `i` is at time `i / sampleRateHz` seconds |
| `actors` | array | exactly 2 actor objects (index 0 and 1, as in marker files) |
| `bodyMap` | string | non-empty; the body map name the bones belong to (e.g. `humanoid`) |
| `bones` | array of strings | non-empty strings, unique within the actor; may be empty only if `frames` is empty |
| `frames` | array of frames | both actors have the same number of frames (at least 1) |
| frame | array of transforms | one per entry of `bones`, in the same order |
| transform | array of 7 numbers | `[x, y, z, qx, qy, qz, qw]`: world position in meters and a unit quaternion (within 1e-6) |

All fields are required, unknown keys are errors, and numbers must be finite. The error rules are the same as marker files (`docs/formats/markers.md`): file, 1-based line and JSON pointer, e.g. `/actors/1/frames/4/0/6`. A frame whose length differs from `bones` is reported at that frame; an actor whose frame count differs from actor 0's at its `frames`.

## Canonical form

`writePoseClip` output, which round-trips byte-for-byte: keys in the order above; 2-space indent; one frame per line (a frame and its transforms on one line, as in the example); `bones` on one line; numbers in shortest round-trip form; LF line endings; a trailing newline. An empty array is written `[]`.
