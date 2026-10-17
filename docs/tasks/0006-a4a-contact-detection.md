# Task: A4a Pose clips and contact detection
Status: in progress (A4 decisions made by toddhow on 2026-10-17).
Design doc section: CLAUDE.md → "Alignment plugin track" → A4; `docs/tasks/0005-a4-contacts-detection.md` (A4a).
Goal: Find interaction markers automatically from a recorded pose clip of two actors, with a confidence tier per contact.
Inputs / outputs: `docs/formats/poseclip.md`, `docs/formats/detection.md`; headers `kinematic/markers/include/opas/markers/PoseClip.hpp` and `ContactDetection.hpp`; implementation in `kinematic/markers/src/`.
Constraints: deterministic, never throws; loaders follow the marker-file error rules; nlohmann/json stays private.

## Detection rules (exact; tests are written from these)
Inputs: a `PoseClip`, one `BodyMapping` per actor, and `DetectionSettings` (the profile).

1. **Candidates.** For each ordered actor pair (source `s`, target `t`), with `s` ∈ {0, 1} and `t = 1 − s`:
   - each **source bone** is the last bone of each limb in `s`'s body map (hands and feet in the humanoid map);
   - each **target region** is each capsule of `t`'s body map.

   Every bone a candidate needs must appear in the clip's `bones` for that actor. A missing bone is an error (`valid == false`) naming the actor and bone.
2. **Per frame f**, with `p` = source bone world position:
   - the capsule axis runs from the region's `bone` to its `endBone` world positions; radius r = `radiusRatio` × axis length in that frame;
   - c is the closest point on the axis segment to `p`;
   - the **surface distance** is `d = |p − c| − r`, which may be negative (inside);
   - the frame is a **contact frame** when `d ≤ distanceThresholdM`.
3. **Region frame.** The target capsule's `bone` world transform. `l_f` is `p` expressed in it, i.e. `inverse(T_bone,f) · p_f`, in meters.
4. **Relative speed** at frame f is `|l_{f+1} − l_{f−1}| / (2/rate)` (a central difference). The first and last frames use one-sided differences (`|l_1 − l_0| · rate`, `|l_n − l_{n−1}| · rate`). A clip of one frame has speed 0.
5. **Runs.** Each maximal stretch of consecutive contact frames is one candidate contact:
   - window `startS = first / rate`, `endS = last / rate`;
   - `durationS = (last − first + 1) / rate`;
   - `meanDistanceM` = mean of d over the run;
   - `meanSpeedMps` = mean relative speed over the run.
6. **Tiers.** Three tests:
   - *close*: `meanDistanceM ≤ distanceThresholdM / 2`;
   - *long*: `durationS ≥ minDurationS`;
   - *still*: `meanSpeedMps ≤ maxRelativeSpeedMps`.

   **High** passes all three, **Medium** exactly two, **Low** one or none. Low contacts are dropped unless `includeLowConfidence`.
7. **Marker** for a kept contact (format v2):
   - `id = "a<s>-<sourceBone>-on-a<t>-<region>-<k>"`, where k counts this (s, bone, t, region) pair's runs from 0 in time order, including runs dropped as Low, so ids do not change with `includeLowConfidence`. Characters other than letters, digits, `-`, `_` and `.` in the bone and region names become `_`.
   - `sourceActor = s`, `targetActor = t`, `sourceBone`, `targetRegion = region`.
   - `offsetM` = mean of `l_f` over the run.
   - `contactDirection` = unit of the mean of `−n_f`, where `n_f` is the unit vector from c to p expressed in the region frame (rotation only). If that mean is zero (e.g. p on the axis every frame), it is `(0, 0, −1)`.
   - `window` from the run.
   - `priority` = 2 for High, 1 for Medium, 0 for Low.
   - `toleranceM = markerToleranceM`.
8. **Output order:** by `window.startS`, then `id`. Each `DetectedContact` also reports its tier and the run statistics.
9. **Edge cases:** a clip with no frames gives no contacts (valid). Sanitizing in rule 7 works per byte, so each byte of a non-ASCII (UTF-8) name becomes `_`.

## Acceptance criteria
- [ ] Pose clip and detection profile formats documented, with loaders, writers, round-trip tests and error tests (file, line, pointer) for every rule in each format table.
- [ ] Synthetic clips (built in tests from a synthetic humanoid; no game data):
  - a hand held still on the other actor's shoulder for 1 s → one High contact on the right bone and region, window within one sample, offset ≤ 1 cm of truth, direction pointing into the surface;
  - a brushing touch shorter than `minDurationS` → Medium or Low per the tier rules;
  - a hand sliding across the chest faster than `maxRelativeSpeedMps` → not High;
  - no contact → nothing detected.
- [ ] Every rule above is checked by at least one hand-computed case: candidates, surface distance (inside and outside), central and one-sided speed, run splitting, each tier combination, the id format and sanitizing, the direction fallback, `includeLowConfidence`, and output order.
- [ ] Detected markers fed back to `alignPair` on the frame at `window.startS` of the clip they came from are met (residual ≤ `markerToleranceM`) without any root shift larger than 1 mm.
- [ ] A missing bone or invalid settings (any threshold ≤ 0 or non-finite, a clip with a non-positive sample rate) gives `valid == false` with a message; never throws.
- [ ] All presets pass; format and lint clean.

Out of scope: the plugin's recorder (A4c side), the validation pass (A4b), IK changes (A4c).
