# Preset manifests (`preset.json`)

A community preset is a folder `SKSE/Plugins/CreativeAlignment/presets/<presetName>/` holding this manifest and `markers/*.markers.json` (optionally `bodymap/*.bodymap.json`), installed as its own MO2 mod (A6b, `docs/tasks/0010`). Read by `opas::markers::loadPresetManifest`, written by `writePresetManifest`. JSON (RFC 8259), UTF-8.

## Example (canonical form)

```json
{
  "format": "opas.preset",
  "version": 1,
  "name": "Example Hugs",
  "author": "Someone",
  "presetVersion": "1.0.0",
  "license": "CC-BY-4.0",
  "priority": 10,
  "requires": ["OStim NG 7.3"],
  "description": "Hand markers for the hug scenes."
}
```

## Fields

| Field | Type | Rule |
| --- | --- | --- |
| `format` | string | exactly `"opas.preset"` |
| `version` | integer | exactly `1` (the file format, not the preset) |
| `name`, `author`, `presetVersion` | string | non-empty |
| `license` | string | an SPDX license expression (letters, digits, `.-+:()`, inner spaces); required, so a preset without a license is not loaded (toddhow, 2026-10-17). Not checked against the SPDX list. |
| `priority` | integer | any; higher wins among presets |
| `requires` | array of strings | optional; free text such as the OStim package and version the scene IDs belong to |
| `description` | string | optional, non-empty when present |

Unknown keys are errors. Errors carry file, line and JSON pointer, as for marker files.

## Which marker file a scene uses

Whole files per scene (toddhow, 2026-10-17): the player's own marker file wins; among presets, the highest `priority` wins, then the smallest `name` in byte order (`chooseMarkerSource`). Markers are never merged across files.

## Canonical form

Keys in the order above, one per line, 2-space indent; `requires` on one line and omitted when empty; `description` omitted when empty; LF line endings; a trailing newline.