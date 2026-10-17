# Task: A4b Marker validation pass
Status: in progress (A4 decisions made by toddhow on 2026-10-17).
Design doc section: CLAUDE.md → "Alignment plugin track" → A4 ("a validation pass flags stages whose markers cannot be met"); `docs/tasks/0005-a4-contacts-detection.md` (A4b).
Goal: For a scene's markers and a recorded clip, report which markers cannot be met when the two actors have different body sizes.
Inputs / outputs: header `kinematic/markers/include/opas/markers/Validation.hpp` (`scaleActorPose`, `validateMarkers`); a command-line tool `opas_validate_markers` (`tools/validate_markers/`).
Constraints: deterministic; never throws; root alignment only (see rule 3).

## Rules
1. **Scaling an actor.** `scaleActorPose(frame, scale)` scales one actor's bone world transforms about its **ground point** `g`. Here `g` = (x and y of the actor's first bone, lowest z over all its bones). Each position becomes `g + scale · (p − g)`; rotations are unchanged. This stands in for a smaller or larger body playing the same animation.
2. **Sweep.** `ValidationSettings::scales` lists body scales (default {0.8, 1.0, 1.25}). Every ordered pair (s0, s1) is tried, so the default gives 9 pairings. The scales must be finite and > 0, and the list non-empty.
3. **Evaluating a marker.**
   - **Frame:** for each marker, the clip frame is `f = round(window.startS × sampleRateHz)`, clamped to the clip.
   - **Inputs:** each actor's bones at that frame are scaled (rule 1). They form a flat skeleton: every bone is a root, and its world transform is its local pose, with an identity root transform.
   - **Body maps:** used with their IK chains removed, because A3 aligns roots only. A4c will extend validation to limb IK.
   - **Solve:** `alignPair` runs with `clipTimeS = window.startS` and default settings.
   - **Residual:** the marker's own residual from that run, so other markers active at the same time count, as they would in game.
4. **Report.**
   - For each marker in MarkerSet order: its id, the worst residual across all pairings, and the list of failing pairings. A pairing fails when the marker is unmet, `alignPair` clamps its root shift, or `alignPair` returns `valid == false` (its error is kept).
   - A marker passes when it has no failing pairing.
   - `ValidationReport::allPassed()` is true when every marker passes.
   - Failing pairings are listed in sweep order: s0 ascending as given, then s1.
5. **Tool.**
   - **Arguments:** `opas_validate_markers --markers <file> --clip <file> --bodymap <file> [--bodymap1 <file>] [--scales 0.8,1,1.25]`. Actor 1 uses `--bodymap1` if given, otherwise the same map as actor 0.
   - **Output:** one line per marker (`PASS id worst=0.012m` or `FAIL id worst=0.081m at 0.8/1.25, …`), then a summary line.
   - **Exit code:** 0 if all pass, 1 if any fails, 2 for bad arguments or files. Load errors print `file:line: field: message`.

## Acceptance criteria
- [ ] `scaleActorPose` matches hand-computed results (ground point, scale 1 is identity, rotations unchanged).
- [ ] A synthetic scene with a reachable marker passes at every pairing of the default sweep; a deliberately unreachable marker fails, listed at exactly the pairings where it is unmet (hand-derived), with its worst residual.
- [ ] A marker whose `alignPair` call is invalid (e.g. a body map missing its region) fails with the error kept.
- [ ] Invalid settings (empty scales, a scale ≤ 0 or non-finite) give `valid == false` with a message; never throws; deterministic (bit-identical reports).
- [ ] The tool runs on small synthetic files in `tests/data/`: exit code 0 when all pass, 1 when any fails, 2 for a missing file or bad argument; its output lines match the formats above.
- [ ] All presets pass; format and lint clean.

Out of scope: limb IK in validation (A4c), the plugin.
