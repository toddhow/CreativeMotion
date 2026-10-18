# Task: A6 In-game marker editor and community presets
Status: draft, waiting for toddhow's approval (2026-10-17). Lighter workflow (CLAUDE.md, "Lighter loop for A4 to A6").
Design doc section: CLAUDE.md → "Alignment plugin track" (A6 replaces the "Later, not scheduled" items "an in-game marker editor" and "community presets", toddhow 2026-10-17).
Goal: players author and fix markers while watching the scene, and share them as installable presets.
Inputs / outputs: main repository: a preset manifest format and merge rules (`kinematic/markers`), engine-independent with tests. Plugin: the editor and the preset loader.
Constraints: as A4. Presets contain authored markers and body maps only, never game data; every preset declares a license.

## A6a Marker editor (plugin)
1. **Capture:** with a scene node playing, the editor turns "this hand is touching that region right now" into a marker: source bone, target region (the nearest capsule of the other actor), `offsetM` = the hand's position in the region's frame, `contactDirection` toward the capsule axis, window from 0. Same arithmetic as A4a's detection rule 7, applied to one frame.
2. **Adjust:** select a marker; nudge its offset along the region's axes in fixed steps (proposal: 1 cm); change tolerance and priority; delete. IK shows the result live (A4c-2).
3. **Check:** run the A4b validation on the current node's pose and show which markers fail at which body-size pairings.
4. **Save:** writes `<sceneId>.markers.json` with `writeMarkerSet` (canonical form) into the user's marker folder; the previous file is kept as `.bak`.
5. **UI:** see decision 1.

## A6b Community presets (main repository format, plugin loader)
1. **Preset folder:** `SKSE/Plugins/CreativeAlignment/presets/<presetName>/` with `preset.json` and `markers/*.markers.json` (optionally `bodymap/*.bodymap.json`). Each preset is its own MO2 mod.
2. **Manifest (`opas.preset`, version 1):** `name`, `author`, `version`, `license` (SPDX identifier), `priority` (integer), optional `requires` (e.g. the OStim package and version the scene IDs belong to), optional `description`. Unknown keys are errors; loader errors carry file, line and field, as for other formats.
3. **Merging:** for a scene ID, the user's own marker file wins over every preset; among presets, higher `priority` wins, then the name in byte order (deterministic). Whole files are chosen, never merged marker by marker (proposal; simpler to reason about). The log lists, per scene, which preset won and which were shadowed.
4. **Export:** the editor can copy the user's marker files into a new preset folder with a manifest skeleton for sharing.

## Acceptance criteria
- [ ] Manifest loader and merge rules have unit tests (valid and invalid manifests with located errors; priority and tie order; user files win).
- [ ] In game: capture a marker, nudge it, see IK follow, save it, reload, and it applies (toddhow judges).
- [ ] Two installed presets for the same scene: the higher priority wins and the log says so.

## Decisions for toddhow
1. **Editor UI.** (a) Hotkeys and console lines only: no new dependency, quick to build, clumsy to use. (b) A Dear ImGui window (MIT license) drawn through a Direct3D 11 present hook: comfortable, but a new dependency and a second hook to verify in game. (c) Use an existing ImGui menu framework mod as a dependency (its license and API must be checked first). Proposal: (a) first, so editing works soon, then (b).
2. **Merging granularity:** whole files per scene (proposal) or marker by marker.
3. **Preset licence:** require an SPDX identifier in every manifest (proposal); presets without one are not loaded.

Out of scope: an online preset browser, automatic updates, editing body maps in game.
