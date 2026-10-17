# Detection profiles (`*.detection.json`)

Tunable thresholds for automatic contact detection (`opas::markers::detectContacts`). Read by `loadDetectionProfile`, written by `writeDetectionProfile`. JSON, UTF-8.

## Example (canonical form; these are the defaults)

```json
{
  "format": "opas.detection",
  "version": 1,
  "distanceThresholdM": 0.03,
  "minDurationS": 0.25,
  "maxRelativeSpeedMps": 0.2,
  "markerToleranceM": 0.02,
  "includeLowConfidence": false
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.detection"` |
| `version` | integer | exactly `1` |
| `distanceThresholdM` | number | `> 0`; a source bone this close to a region's surface (or inside it) is "in contact" |
| `minDurationS` | number | `> 0`; a contact held at least this long is "long" |
| `maxRelativeSpeedMps` | number | `> 0`; a contact moving slower than this relative to the region is "still" |
| `markerToleranceM` | number | `> 0`; the `toleranceM` written into detected markers |
| `includeLowConfidence` | boolean | write low-confidence contacts too |

All fields are required; unknown keys are errors; numbers must be finite. The defaults were chosen by toddhow on 2026-10-17 (A4 decision 1) as starting values for tuning. Errors and the canonical form follow the marker-file rules: keys in the order above, 2-space indent, one key per line, shortest round-trip numbers, LF, trailing newline.
