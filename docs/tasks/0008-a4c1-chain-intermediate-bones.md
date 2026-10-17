# Task: A4c-1 IK chains with intermediate bones
Status: in progress (A4 decision 3, toddhow 2026-10-17).
Design doc section: CLAUDE.md → "Alignment plugin track" → A4; `docs/tasks/0005-a4-contacts-detection.md` (A4c, and the NAF reference notes).
Goal: An `IkChain`'s three bones may have other bones between them (XPMSSE puts `CME` nodes between calf and foot); the solver bends the chain correctly anyway.
Inputs / outputs: `IkChain` rules in `kinematic/markers/include/opas/markers/BodyMeasurements.hpp`; `alignPair` in `Alignment.cpp`. No format change: body map files already name only the three bones.
Constraints: no change in results for chains that are direct parent and child (all A2 tests unchanged); deterministic; never throws.

## Rules
1. **Validation:** `midBone` must be a descendant of `rootBone`, and `endBone` a descendant of `midBone`, at any depth. A chain that breaks this is rejected with a message naming the chain. This replaces A2's "direct child" rule.
2. **Solving:** the three bones' world positions feed `solveTwoBone` as before.
3. **Applying, in order:**
   1. Set the root bone's new world rotation, `rootRotation · oldRootWorld`. Convert it to local using the root bone's parent's world transform, then recompute the actor's world pose.
   2. Set the mid bone's new world rotation, `midRotation · oldMidWorld` (old means before this chain was solved). Convert it to local using the mid bone's *actual* parent's world transform from the recomputed pose, then recompute again.

   The bones in between keep their local transforms, so they rotate with their parents. That is why the solved mid and end positions are reached.
4. **Results:** after applying, the end bone's world position equals the solved end (within 1e-9 m), and every bone's distance to its parent is unchanged.

## Acceptance criteria
- [ ] A synthetic skeleton like XPMSSE's leg, with two intermediate nodes between calf and foot and one between upper arm and forearm (each with a non-identity local transform), aligns a foot marker and a hand marker: the end bone reaches the solved end within 1e-9 m, parent distances are kept, and markers are met as in A2.
- [ ] The same scene built with direct parent-child chains gives bit-identical results to before this change (A2 behaviour preserved).
- [ ] Chains whose mid is not below root, or whose end is not below mid (e.g. a sibling, or the reverse order), are rejected with the chain named; a chain through intermediates is accepted.
- [ ] All existing tests pass; all presets; format and lint clean.

Out of scope: the in-game pose hook (A4c-2, plugin), validation with IK (a later A4b extension).
