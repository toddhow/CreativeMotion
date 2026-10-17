# Task: A4 Contacts and detection
Status: draft — awaiting toddhow's decisions below. (toddhow authorized starting A4 once A3 is done, 2026-10-17.)
Design doc section: CLAUDE.md → "Alignment plugin track" → A4.
Goal: Markers are found automatically from recorded paired clips; hands and feet meet their contacts in game through per-frame IK; a second framework is supported; scenes whose markers cannot be met are flagged.

A4 spans both repositories, as the track requires. Engine-independent parts land in CreativeMotion first and are tagged; the plugin bumps its pin.

## Parts
**A4a — Pose clips and contact detection (CreativeMotion, `kinematic/markers`)**
- A *pose clip* format (`docs/formats/poseclip.md`, JSON): two actors' bone world transforms sampled over time, plus each actor's body map name. The plugin's recording command grows from one snapshot to a timed recording in this format. Real recordings stay local (guide rule); tests use synthetic clips.
- `detectContacts(clip, bodyMaps, settings)` → candidate markers. A candidate is a (source bone of one actor, region of the other) pair whose bone stays within `distanceThresholdM` of the region's capsule surface for at least `minDurationS`, while its velocity relative to the region's frame stays below `maxRelativeSpeedMps`. Each candidate gets a time window, an offset (mean contact point in the region's frame), a contact direction (mean surface normal, inverted), a tolerance, and a **confidence tier**:
  - **high:** close, long and still;
  - **medium:** meets two of the three;
  - **low:** brief or moving. Low-tier candidates are written only on request.
- Output is a MarkerSet (format v2), so the A2 solver and A3 plugin use detected markers unchanged.

**A4b — Validation pass (CreativeMotion)**
- `validateMarkers(markers, bodyMaps, poses, sizeSweep)`: runs `alignPair` across a sweep of body-size pairings (e.g. 0.8–1.25 scale) and reports, per marker, the pairings where it is unmet. A command-line tool (`tools/validate_markers`) runs it over a folder of marker files plus clips.

**A4c — Limb IK in game (plugin, plus a solver change in CreativeMotion)**
- **Solver change:** IK chains whose bones are not direct parent and child. XPMSSE puts `CME` nodes between the calf and the foot, and between other limb bones (found in A3, `bodymap/README.md`). Chains declare their three bones, and the solver carries intermediate bones along.
- **Pose hook:** per-frame IK after the animation graph has updated. The candidate hook point is the virtual `UpdateAnimation` (vtable slot `0x7D`; CommonLibSSE-NG `include/RE/T/TESObjectREFR.h:305`, overridden by `Actor` at `include/RE/A/Actor.h:344`). It would be patched in `RE::VTABLE_Actor[0]` (`include/RE/Offsets_VTABLE.h:2142`) and in the `PlayerCharacter` vtable, calling the original first. **Unverified:**
  - that bone world transforms are final when `UpdateAnimation` returns;
  - how to push our changed local rotations to the world transforms (`NiAVObject::UpdateWorldData` or similar). This needs CommonLib-cited facts and an in-game test before the design is final.
- Hand and foot IK use the A2 solver's limb step, with markers from files or detection. It runs only for the actors in an aligned scene, behind the existing toggle.

**A4d — Second framework adapter (plugin)**
- SexLab is the second framework named in the guide. Not yet researched:
  - its API surface (Papyrus mod events vs an SKSE API);
  - its license and terms, and its source availability;
  - the SexLab P+ variant.

  This must be checked and cited before the adapter is designed (guide rule: licenses before dependencies).

## Decisions for toddhow
1. **Detection thresholds:** start from distance 3 cm, duration 0.25 s, relative speed 0.2 m/s, and tune with your recordings? (These are data in a detection profile file, not code.)
2. **Pose clip recording:** the plugin records at a fixed rate (proposal: 30 Hz) for a fixed time, or until you press the record key again?
3. **Order of work:** proposal is A4a → A4b → A4c → A4d. A4c needs in-game testing of the hook, so it can only start when you can test, and it also depends on your A3 in-game results.
4. **XPMSSE chains:** support intermediate bones in the solver (proposal), or use only bone pairs that are direct parent and child in both skeletons (fewer chains, e.g. no knees)?
5. **SexLab:** confirm SexLab (or SexLab P+) is the second framework, so the research in A4d targets the right one.

## Acceptance criteria (refined once decisions are made)
- [ ] Pose clip format documented, with loader, writer, round-trip and error tests.
- [ ] Synthetic clips with known contacts (hand on shoulder held 1 s; a brushing touch; no contact) are detected with the right bone, region, window (± one sample), tier and offset (≤ 1 cm); no false positives in the no-contact clip.
- [ ] Detected markers drive `alignPair` and meet themselves on the clip they came from.
- [ ] The validation pass flags a deliberately unreachable marker at the body-size pairings where it fails, and passes reachable ones.
- [ ] The solver handles chains with intermediate bones (synthetic XPMSSE-like skeleton); existing A2 tests still pass.
- [ ] Plugin: per-frame hand IK meets a marker in game (toddhow judges); hook points cited; toggle off restores the unmodified animation.
- [ ] Plugin: the second framework adapter starts and stops alignment for its scenes (after A4d research).

Out of scope: finger contact, mesh-sampled body surfaces, an in-game marker editor, community presets (all "later" in the guide).
