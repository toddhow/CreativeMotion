# Task: A7 Automatic marker placement
Status: approved by toddhow 2026-10-18 with all proposals. Lighter workflow (CLAUDE.md, "Lighter loop for A4 to A6", extended to A7 unless toddhow says otherwise).
Design doc section: CLAUDE.md → "Alignment plugin track". toddhow's request (2026-10-17/18): scenes should not need a hand-made marker file; markers start from where the animation puts the limbs and adapt to each actor's real body ("like a point on the base BodySlide body carried through the body changes", "like skinned clothing adapting to a body").
Goal: a scene node with no marker file gets markers automatically, and every marker's target lands on the actual skin (or clothing) of the actor it touches.
Inputs / outputs: main repository: `autoAuthorMarkers` and `snapOffsetToSurface` in `kinematic/markers` (engine-independent, tested). Plugin: a silent auto-recording, an `auto` marker folder, surface snapping when a scene is published.
Constraints: as A4 to A6. Auto files are game-derived only in the sense of positions relative to bones; they stay in the user's game folder like recordings and are never committed.

## A7a Authoring markers from a clip (main repository)
1. **Detection:** run A4a `detectContacts` on the clip with an **auto profile**: wider than the authoring defaults because bodies in game differ from the ones the animation was made for (proposal: distance 8 cm, duration 0.5 s, relative speed 0.2 m/s). Keep High and Medium contacts.
2. **Steady contacts only:** OStim exposes no animation time, so in game an auto marker is active for the whole node. A contact that comes and goes during the loop would then pull the hand in all the time, so only contacts lasting at least half the recording are kept (a consequence of the approved design, noted 2026-10-18). The kept markers' windows start at 0 and last for the node, as the editor's captures do.
3. **Validation:** run A4b `validateMarkers` on the same clip; drop markers that fail at the 1.0/1.0 pairing (the bodies as recorded). Failing at other pairings is allowed and reported.
4. **Output:** a `MarkerSet` whose ids start with `auto-`, plus a per-contact report (kept or dropped, and why). Deterministic; never throws.

## A7b Adapting a marker to the real surface (main repository)
toddhow's surface-binding idea, topology-free version (works for any body mod; binding to mesh triangles of a known body is a later step).
1. **Input:** the marker's offset `p` and inward `contactDirection` `d`, both in the target region bone's frame, and surface points of that region in the same frame (from A5b sampling, stored relative to their bone so they follow the animation).
2. **Snap:** surface points within `snapRadiusM` (3 cm) of the line `p + s·d` count. Find the nearest surface layer along the line (smallest `|s|`, plus 1 cm of depth), and in it the point closest to the line; move `p` along the line only, to that point's `s` minus `contactOffsetM` (2 cm, roughly half a hand's thickness, since the hand bone sits inside the palm). Refined while implementing (2026-10-18): taking simply the nearest point along the line let points at the edge of the tube pull the marker sideways or short of a curved surface (caught by the cylinder test).
3. **No surface near the line:** the offset is unchanged and the result says so.

## A7c In game (plugin)
1. **When:** a player scene node starts, `bAutoMarkers=1`, and no marker file exists for the scene (user, preset or auto).
2. **Record:** silently record the two actors for `fAutoRecordS` (proposal: 4 s) with the A4c-3 recorder; nothing is shown except one console line.
3. **Author:** run A7a; save to `SKSE/Plugins/CreativeAlignment/markers/auto/<sceneId>.markers.json`; republish the scene's IK. Auto files are the lowest-priority source: the user's own file and presets always win (A6b order, auto last).
4. **Snap:** whenever a scene is published (any marker source), each marker's offset is snapped to the target actor's sampled surface (A7b), per actor pair, before IK uses it. `bSurfaceSnap=1` turns this on.
5. **Fix by hand:** the F9 editor works on auto markers like any other; saving writes the user's own file, which then wins.

## Acceptance criteria
- [ ] A7a: synthetic clips with a steady hand-on-chest contact give one `auto-` marker that validates; a passing-by hand gives none; deterministic.
- [ ] A7b: synthetic surfaces (a cylinder of known radius) snap an offset onto the surface within 1 mm, minus the contact offset; no points near the line leaves it unchanged.
- [ ] In game: a scene without markers gets an auto file within a few seconds, IK uses it, and the hands land on the partner's body (toddhow judges) at two body-size pairings.
- [ ] User and preset files still win over auto files.

## Decisions for toddhow
1. **Auto profile:** 8 cm, 0.5 s, 0.2 m/s; keep High and Medium (proposal).
2. **Recording length:** 4 s per node (proposal).
3. **Snap values:** search radius 3 cm, contact offset 2 cm (proposal; per-chain values could come later).
4. **Milestone:** call this A7 in CLAUDE.md, after A6 (proposal).

Out of scope: an ML model (only if rules plus snapping fall short), binding markers to mesh triangles of known bodies (later), animation-time-aware windows (no time source in OStim).

## Milestone review fixes (2026-10-18)
- Validation checks each candidate in the middle of its own contact run (validation evaluates a marker at `round(startS·rate)`); checking every candidate at frame 0 dropped contacts that start late or drift.
- One contact per limb end: with the 8 cm auto distance a hand can be near several regions at once; the best fit wins (tier, then distance, then duration); the others are reported as shadowed.
- Snapping ignores surface more than 12 cm along the line, so a missing front layer cannot pull a marker through the body.
- In game: snapping uses every sampled surface point within the region's capsule radius plus a margin (not only points weighted to the region's start bone, which missed breasts and clavicles); it skips markers from the user's own file and markers being edited, so hand fixes are not undone (rule 5); body samples are cached per scene node; a queued length-limit stop only stops the recording it was queued for.

## A7 v2: per-frame contact correction (toddhow, 2026-10-18)
In game, pinned automatic markers made scenes worse: each marker is one fixed point (the recording's mean), active for the whole node because OStim has no animation time, so the IK dragged limbs to it whatever the animation did, and root alignment shifted the actors to match. toddhow's goal is to adapt the animation at runtime, not to pin bones. Decisions (all approved 2026-10-18):
1. **Per-frame correction:** every frame, each hand and foot keeps the position the animation gives it; only its depth against the partner's real surface (A5b samples, moving with their bones) is corrected, along the direction to the body. Sliding across the body stays as animated.
2. **Any near pair:** every hand and foot against any of the partner's regions, whenever the animation brings it close. No marker file is needed. A user or preset marker file still pins the limb ends it names (authored positions win).
3. **Range:** full correction when the animated hand is within 4 cm of the surface, fading to none at 10 cm; a hand inside the body is always pushed out. The hand ends 2 cm outside the surface (the contact offset).
4. **Root alignment:** only from the user's own or preset marker files; automatic contacts never move the actors.
Consequences: automatic pinned markers (A7a, A7c rules 1 to 3) are off by default (`bAutoMarkers=0`) and their files are not used as positions; the authoring code stays for later use (e.g. finding pairs). Rules for the correction itself:
- **Nearest region:** for each limb end, the partner region whose surface is nearest along the direction from the limb end to that region's axis; regions whose capsule is farther than the fade distance are skipped first.
- **Gap:** the signed distance along that direction to the nearest surface layer (A7b's layer rule); negative inside the body.
- **Target:** limb end + weight x (gap - contact offset) along the direction, with weight 1 up to 4 cm (and inside), falling smoothly to 0 at 10 cm.
- **Solving:** each target becomes a per-frame marker for the IK (roots fixed), so joint limits, poles and penetration avoidance still apply. At weight 0 the target equals the animated position, so contacts fade in and out without popping.