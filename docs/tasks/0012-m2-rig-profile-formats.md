# Task: M2 Rig and profile formats
Status: done. Implemented and reviewed 2026-10-18; toddhow approved the merge on 2026-10-18 (merged locally into main, not pushed). Approved by toddhow on 2026-10-18 with all four proposals: the lighter loop continues, core loaders may use nlohmann/json privately, version 1 rigs carry their own bones, and the sources are de Leva 1996 plus a published range-of-motion table.
Design doc section: CLAUDE.md → "First milestones" → M2; OpenMW Animation System — Implementation Doc → "Engine integration points" (rig profiles load through the VFS like other content) and "Animation storage and formats" (skeleton profiles). **The Living Design Doc is not in the repository, so the file layouts below are drafted from the guide, as M1's interface was. Reconcile them once it is exported.**
Goal: a character's physical body (skeleton, rigid bodies, masses, joint limits) and its tuning (motor gains, materials) come from two JSON files. Both files load with located errors and write back byte-for-byte, and a sample humanoid ships with them.
Inputs / outputs:
- `core/rig` gains the rig and profile data model, validation, loaders and canonical writers (`loadRig`/`writeRig`, `loadProfile`/`writeProfile`).
- Shared JSON support moves out of `kinematic/markers` into a new internal module (see "Shared JSON support").
- New `docs/formats/rig.md` and `docs/formats/profile.md`.
- New `data/rigs/humanoid.rig.json` and `data/profiles/humanoid-default.profile.json`.
- Tests in `tests/unit/core_rig/` and `tests/roundtrip/core_rig/`.
Constraints:
- Same rules as the existing formats (`docs/formats/bodymap.md`). JSON (decision 0002), UTF-8. Every field is required, unknown keys are errors and numbers must be finite. Errors carry file, 1-based line and JSON pointer, and nothing is thrown.
- SI units, right-handed, Z up. Converting to OpenMW units (1 m ≈ 69.99 units, `components/misc/constants.hpp` in OpenMW) is the host's job, never the file's.
- `core/rig` never includes physics or engine types. The rig has its own shape and joint-limit types, chosen so they map one-to-one onto `opas::physics::Shape` and `JointDesc`; M3 does that conversion.
- Loading is not a hot path, so it may allocate. Validation is deterministic: arrays are processed in file order, and any map used for lookups never decides output order.

## Rig file (`*.rig.json`, format `"opas.rig"`, version 1)
The rig is character data: what the body is.
```json
{
  "format": "opas.rig",
  "version": 1,
  "name": "humanoid",
  "bones": [
    {"name": "Pelvis", "parent": "", "translationM": [0, 0, 0.95], "rotation": [0, 0, 0, 1]},
    {"name": "Spine", "parent": "Pelvis", "translationM": [0, 0, 0.1], "rotation": [0, 0, 0, 1]}
  ],
  "bodies": [
    {
      "bone": "Pelvis",
      "shape": {"type": "capsule", "halfHeightM": 0.08, "radiusM": 0.12},
      "translationM": [0, 0, 0.05],
      "rotation": [0.7071068, 0, 0, 0.7071068],
      "massKg": 11.2
    }
  ],
  "joints": [
    {
      "bone": "Spine",
      "twistAxis": [0, 0, 1],
      "planeAxis": [1, 0, 0],
      "normalHalfConeRad": 0.3,
      "planeHalfConeRad": 0.4,
      "twistMinRad": -0.3,
      "twistMaxRad": 0.3
    }
  ]
}
```
- **Bones** give each bone's rest transform relative to its parent, as `opas::rig::Skeleton` holds it today. They are listed parents first. Parents are named rather than given by index, so files stay readable and merge cleanly. `""` marks a root.
- **Bodies** are at most one per bone, placed relative to that bone. Shapes are `capsule` (half height and radius, axis along the body's local Z), `box` (`halfExtentsM`) or `sphere` (`radiusM`), the same three shapes the physics interface has. Bones without a body (fingers, twist bones) are animated but not simulated.
- **Joints** are keyed by the child bone. The joint connects that bone's body to the body of its nearest ancestor that has one, so the rig cannot name a parent body that disagrees with the skeleton. The anchor is the child bone's origin in the rest pose. The axes are in the child bone's frame, and the limits are the swing-twist limits of `JointDesc` (M1).
- **Structure rules:** exactly one body has no ancestor body (the root, normally the pelvis). Every other body has exactly one joint, and a joint on a bone without a body is an error.

## Profile file (`*.profile.json`, format `"opas.profile"`, version 1)
The profile is tuning: how the body behaves. One rig can have several profiles (default, drunk, wounded).
```json
{
  "format": "opas.profile",
  "version": 1,
  "rig": "humanoid",
  "defaults": {
    "motor": {"stiffnessNmPerRad": 300, "dampingNmsPerRad": 30, "maxTorqueNm": 500},
    "material": {"friction": 0.8, "restitution": 0, "linearDamping": 0.05, "angularDamping": 0.05}
  },
  "joints": [
    {"bone": "Spine", "motor": {"stiffnessNmPerRad": 600, "dampingNmsPerRad": 60, "maxTorqueNm": 900}}
  ],
  "bodies": [
    {"bone": "LeftFoot", "material": {"friction": 1.0, "restitution": 0, "linearDamping": 0.05, "angularDamping": 0.05}}
  ]
}
```
- **Defaults and overrides:** `defaults` covers every joint and body, and the `joints`/`bodies` arrays override single bones. An override replaces the whole `motor` or `material` object, with no field-by-field merging, so what applies is always visible in one place.
- **Field meanings** match `JointMotor` and `BodyDesc` (M1). Gains must be `>= 0`, `maxTorqueNm > 0`, and friction, restitution and damping `>= 0`.
- **Checking against a rig:** `rig` names the rig the profile was tuned for. Checking it, and checking that every overridden bone has a joint or body, needs the rig, so `validateProfile(profile, rig)` does it separately from loading, as body maps are checked against a skeleton.
- **Later sections:** M5 adds balance and hit-reaction tuning as new sections under version 2. Version 1 files keep loading.
- **Sample values:** the gains above are placeholders until M3 measures tracking. The sample profile's values are marked as "to be tuned in M3" in `docs/formats/profile.md`.

## Sample humanoid (`data/rigs/humanoid.rig.json`)
- **Bodies:** 16: pelvis, spine, chest, head, plus upper arm, forearm, hand, thigh, shin and foot on each side. (Corrected 2026-10-18 during implementation: the draft said 15, but the listed parts are 4 + 2 × 6 = 16, and three trunk bodies match de Leva's three trunk segments.)
- **Neck and fingers:** these are bones without bodies.
- **Size:** synthetic, 1.75 m and 75 kg.
- **Masses:** split by published segment-mass fractions (proposal: de Leva 1996, "Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters", J. Biomech. 29(9)).
- **Joint limits:** taken from a published table of normal ranges of motion. Each value and its source go in `data/rigs/README.md`. If a source can't be confirmed, the value is marked as an estimate rather than invented silently.
- **Names:** generic. No game skeleton or game-derived rest pose is committed.

## Shared JSON support
- **Why it moves:** the rig loader needs the line-tracking reader and the canonical number writer that `kinematic/markers/src/JsonSupport.hpp` already has. `markers` depends on `rig`, so `rig` cannot reach back into `markers`.
- **New module:** `core/format`, a static library holding:
  - `LoadError`, moved from `opas::markers` and kept there as a `using` alias, so no public signature changes;
  - the JSON reader and writer helpers.
  `nlohmann/json` stays a PRIVATE dependency, and no JSON type appears in a public header.
- **Guide conflict:** the guide says "`core` depends only on `physics/interface` and the standard library", but it also puts loaders in `core/rig` and names nlohmann/json as the parser. Proposal: amend the rule to "plus nlohmann/json, privately, for loaders only". This needs toddhow's decision (see Questions).

## Acceptance criteria
- [x] `docs/formats/rig.md` and `docs/formats/profile.md` document every field, its rule, the error rules and the canonical form, in the style of `bodymap.md`.
- [x] The sample rig and profile load, validate against each other and give a skeleton that passes `validateSkeleton`: 16 bodies (see "Sample humanoid"), total mass 75 kg ± 0.01, the root body on `Pelvis`, and a joint on every other body.
- [x] The loader rejects each of these with the right file, line and pointer:
  - a syntax error, an unknown key or a missing field;
  - a wrong `format` or `version`;
  - a duplicate bone, or a parent that is unknown or listed after its child;
  - a non-unit rotation or a non-finite number;
  - a body on an unknown bone, or two bodies on one bone;
  - a bad shape type or a size that isn't `> 0`;
  - `massKg <= 0`;
  - a joint on a bone without a body, a missing joint, or two root bodies;
  - axes that are zero, not unit length within 1e-6, or not perpendicular within 1e-6;
  - limits outside `[0, pi]` for cones, or outside `[-pi, pi]` for twist, or a twist minimum above its maximum;
  - in a profile: negative gains or material values, `maxTorqueNm <= 0`, or a duplicate override;
  - when checking a profile against a rig: a rig name mismatch, or an override on a bone with no joint or body.
- [x] Writing the sample files, and synthetic rigs and profiles covering every shape type, then reading them back gives byte-for-byte identical text. Loading and then writing a canonical file reproduces it.
- [x] All existing tests still pass, and the `kinematic/markers` public headers are unchanged.

Out of scope:
- Building a ragdoll from the rig (M3).
- Balance and behavior tuning sections (M5).
- The bone-correspondence "skeleton profile" files from the implementation doc that map Bip01, Skyrim and XPMSE bones onto one rig. They come with the animation runtime in the fork.
- Reading a host's skeleton instead of the file's bones (see Questions).
- Any OpenMW code.

## Questions for toddhow
1. **Workflow:** should the lighter loop (no test-writer subagent, one review per milestone) continue for M2? Proposal: yes.
2. **Dependency rule:** should the guide's rule on `core` dependencies be amended so `core` may use nlohmann/json privately for loaders? Proposal: yes. The alternative is a separate `formats/` module outside `core`.
3. **Skeleton source in game:** in the fork, the real rest pose comes from Morrowind's skeleton NIF, which can't be committed. Proposal: version 1 always carries its own bones, for the sandbox and tests. A later version lets a rig attach bodies to a host-supplied skeleton by bone name, with the bone list optional. Is that right, or should version 1 already allow it?
4. **Sources:** de Leva 1996 for masses, plus a published range-of-motion table for joint limits. Are you fine with these, or do you have preferred sources?
