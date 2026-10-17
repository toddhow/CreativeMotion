# Task: A4 Contacts and detection
Status: decisions made by toddhow on 2026-10-17 (below); A4a in progress.
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
- **Pose hook:** per-frame IK after the animation graph has updated. The candidate hook point is the virtual `UpdateAnimation` (vtable slot `0x7D`; CommonLibSSE-NG `include/RE/T/TESObjectREFR.h:305`, overridden by `Actor` at `include/RE/A/Actor.h:344`). It would be patched in `RE::VTABLE_Character[0]` (`include/RE/Offsets_VTABLE.h:2215`; NPCs are `Character`, `include/RE/C/Character.h:7`) and `RE::VTABLE_PlayerCharacter[0]` (`:2231`), calling the original first. (Corrected 2026-10-17: an earlier draft named `VTABLE_Actor`, the base class's vtable.) The A4c-2 design is in the plugin repository, `docs/tasks/0002-a4c2-pose-hook-ik.md`. **Unverified:**
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

## Decisions (toddhow, 2026-10-17)
1. **Thresholds:** start with distance 3 cm, duration 0.25 s, relative speed 0.2 m/s, stored in a tunable JSON detection profile.
2. **Recording:** F11 starts recording at 30 Hz and F11 again stops it, capped at 60 s.
3. **Chains:** the solver supports intermediate bones between a chain's root, mid and end.
4. **Second framework:** SexLab P+. Research its API and license before designing the adapter.
5. **Order of work:** as proposed, A4a → A4b → A4c → A4d.

## Reference: Native Animation Framework (read 2026-10-17)
NAF (`github.com/Deweh/Native-Animation-Framework`, commit `9d8efbfea0e2f6048a59f7a7af8ccf814dd61791`) is an F4SE plugin for Fallout 4. `LICENSE.txt` is GPL-3.0, which is compatible with this project, but its `COPYRIGHT.txt` also says "All rights reserved"; ask toddhow before reusing any code. Only its design is used here: no code is copied, and Fallout 4 addresses do not apply to Skyrim. What informs A4c:
- **Per-frame pose hook:**
  - NAF detours `IAnimationGraphManagerHolder::UpdateAnimationGraphManager`, calls the original, then writes its bone transforms. Its `HookedGraphUpdate` also checks `ShouldUpdateAnimation()` (`f4se-plugin/src/BodyAnimation/GraphHook.h`, `RegisterHook` and `HookedGraphUpdate`).
  - It writes **local** transforms only, and relies on the engine's later world update in the same frame (`NodeAnimationGraph.h:224–235`, `PushOutput`).
  - The same function exists in Skyrim's CommonLibSSE-NG with Address Library IDs `RELOCATION_ID(32155, 32899)` (`src/RE/I/IAnimationGraphManagerHolder.cpp:69`). It is a **second candidate hook point** for A4c, next to the `UpdateAnimation` virtual. Whether Skyrim also computes world transforms after it within the frame is **unverified** and must be tested in game.
- **3D reloads:** NAF also hooks `TESObjectREFR::Set3D`, to re-find bone nodes when an actor's 3D is rebuilt (same file). In Skyrim CommonLib, `Set3D` is virtual slot `0x6C` (`include/RE/T/TESObjectREFR.h:288`). A4c should handle 3D reloads the same way, or look nodes up again every frame.
- **Chains that skip bones** (A4 decision 3): NAF solves in world space, then converts each bone back to a local transform using its actual parent's world transform, found by walking up the hierarchy (`IK.h`, header comment and `CopyIKNodeToNiNode`). That is the planned approach for CreativeMotion: after setting a chain bone's new rotation, recompute the world pose, so the next chain bone's real parent (an intermediate bone) is up to date.

## A4d research: SexLab P+ (read 2026-10-17)
Sources: the native backend `github.com/SLP-Community/SexLabpp` (commit `d578f8176142e0e77de7734e86e0017763de6a31`) and the Papyrus framework `github.com/SLP-Community/SexLab` (branch `main`).
- **Licenses:**
  - The native backend is GPL-3.0-or-later with a Modding Exception and a GPL-3.0 linking exception (`NOTICE`, `EXCEPTIONS.md`). Compatible.
  - The Papyrus scripts and assets keep the original SexLab permissions (`Readme - SexLab Framework.txt`, "Permissions"): no redistribution outside LoversLab without permission, and no editing except translations.
  - Our plugin must therefore never ship, copy or modify SexLab's scripts. It only listens to the events SexLab sends.
- **No native API for other plugins.** The backend exports nothing besides the SKSE plugin entry points, dispatches no plugin messages, and only *consumes* RaceMenu's interface (`src/API/IPluginInterface.h`).
- **Scene events are Papyrus mod events.**
  - `sslThreadModel.SendThreadEvent` (`scripts/Source/sslThreadModel.psc:1713–1735`) sends, for each event, a mod event `Hook<Event>` (thread id and has-player), a plain `SendModEvent(<Event>, thread_id)`, and `Player<Event>` when the player takes part.
  - Events sent (`sslThreadController.psc`): `AnimationStart`, `StageStart`, `StageEnd`, `AnimationChange`, `PositionChange`, `ActorsRelocated`, `OrgasmStart/End`, `AnimationEnding`, `AnimationEnd`, plus `AnimationStarting`, `ActorChangeStart/End` and `LeadInStart/End` (`sslThreadModel.psc`).
- **Receiving them in SKSE.** CommonLibSSE-NG exposes plain `SendModEvent` calls as `SKSE::ModCallbackEvent` (event name, string argument, number argument, sender) through `SKSE::GetModCallbackEventSource()` (`include/SKSE/API.h:66`, `include/SKSE/Events.h:12`). The thread id arrives in the string argument.
- **Open design points (before A4d's spec):**
  - how to get a thread's actors, stage and scene id without SexLab's scripts calling us. Options are reading its Papyrus objects from C++, or a small Papyrus bridge script of our own;
  - how SexLab places actors, since `ActorsRelocated` shows it repositions them itself, and so what plays OStim's alignment-offset role there;
  - whether SexLab P+ offers an alignment adjustment we should feed instead of moving actors ourselves (the "consume the framework's API" rule).

## Decision (toddhow, 2026-10-17): A4d deferred
The second framework adapter (SexLab P+) is skipped for now. The research above stays for when it resumes. A4 is complete without it.
