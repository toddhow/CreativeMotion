# Task: A2 Alignment solver
Status: done — approved by toddhow on 2026-10-16, keeping the one-pass solver and the documented limits for now.
Design doc section: CLAUDE.md → "Alignment plugin track" → A2.
Goal: Given two posed actors of different body sizes and a marker set, move their roots so the markers come within reach (sharing the correction), then bend limbs with two-bone IK to meet each marker, keeping limbs out of the other actor's body capsules.

## Decisions for toddhow
1. **Markers must say which actor they belong to.** The A1 format has no actor field, so a marker cannot say whether actor 0's hand touches actor 1 or the reverse. Proposal: format **version 2** adds two required integer fields, `sourceActor` and `targetActor` (actor indices, different from each other). Version 1 is dropped rather than kept loadable, because no v1 files exist outside the tests. The A1 marker tests and canonical texts are updated to v2 by the test writer. Using indices rather than "A"/"B" leaves room for scenes with more than two actors later; the A2 solver itself handles exactly two.
2. **IK chains are data.** Which bones form an arm or a leg chain (shoulder, elbow, wrist), their hinge limits, their pole direction and limb thickness go into `BodyMapping` as `ikChains`, next to limbs and capsules, because skeletons name bones differently.

## Design
**Inputs** (per actor): skeleton, body mapping, local pose (one transform per bone, e.g. one animation sample), root transform in world, and an alignment weight. Plus the marker set, the clip time, and settings.

**Region frames.** A marker's `offsetM` is expressed in the frame of the target region's `CapsuleRule::bone`, in the posed skeleton: target point = that bone's posed world transform applied to `offsetM`.

**Step 1 — root alignment.** Active markers are those whose window contains the clip time (inclusive). For each, e_i = target point − source bone position, both in world. Moving actor 0 by Δ0 and actor 1 by Δ1 changes the relative shift δ = Δ0 − Δ1. With σ_i = +1 when actor 0 is the source and −1 otherwise, least squares over the active markers gives δ = mean(σ_i · e_i), using only the horizontal (X, Y) part; Z stays 0, because both actors stand on the ground. |δ| is clamped to `maxRootShiftM`. The correction is split by alignment weight: Δ0 = w0/(w0+w1) · δ and Δ1 = −w1/(w0+w1) · δ. A weight of 0 pins that actor, e.g. a seated actor or one in furniture. No active markers means no shift.

**Step 2 — limb IK.** After the root shift, every IK chain whose end bone is the source bone of an active marker on that actor is solved towards that marker's target point. If several markers drive the same chain, the highest `priority` wins, and ties go to the smaller `id`. The pole is the chain's `poleDirection`, rotated by the actor's root rotation. Hinge limits come from the chain.

**Step 3 — penetration avoidance.** The solved upper and lower bone segments are tested against the *other* actor's posed capsules, excluding the capsule of the marker's own target region, where contact is intended. Penetration = max(0, capsuleRadius + chain.limbRadiusM − segment-to-capsule-axis distance), summed. If it is > 0, the pole is rotated about the root→target axis in `poleSamples` equal steps. The pose with the least penetration is kept; ties go to the smallest rotation, and the original pole wins on a tie. Remaining penetration is reported, not hidden.

**Output.** Each actor's root shift (and whether it was clamped). For each solved chain: actor, chain name, driving marker id, world-space delta rotations for the chain's root and mid bones (as `TwoBoneResult` gives them), reached, and remaining penetration. For each active marker, in file order: the residual distance after both steps, and `met` (residual ≤ the marker's `toleranceM`). A marker without a chain is measured after the root shift only.

## Acceptance criteria
- [x] Format v2 (`sourceActor`, `targetActor`) documented, with load/round-trip/error tests (actor index not an integer, negative, or the same as the other one). The A1 marker tests are moved to v2.
- [x] `rig::computeWorldPose` matches hand-computed world transforms for a posed 3-bone chain with a non-identity root.
- [x] Root alignment, hand-computed: a single marker with a horizontal error d is fully closed. Weights 1:1 split it half and half; 1:0 moves only actor 0; 0:1 only actor 1. Z error is ignored. A shift larger than `maxRootShiftM` is clamped and reported.
- [x] **Body-size scenes** (synthetic humanoids built in tests, scaled 0.8 and 1.25 — a 1.56× height ratio — in both pairings): (a) a hug, with both actors' hands on the other's upper back; (b) a handshake, right hand to right hand; (c) one actor's hand on the other's shoulder. Every active marker is met within its `toleranceM` (0.02 m in the scenes), and no limb penetrates more than 0.005 m.
- [x] **Penetration avoidance:** a scene where the default pole drives the elbow through the other actor's torso capsule ends with penetration ≤ 0.005 m and the marker still met.
- [x] **Priorities:** two markers on one hand; the higher priority drives the chain. On equal priority, the smaller id wins.
- [x] **Windows:** markers outside the clip time are ignored; a marker at exactly `startS` or `endS` is active.
- [x] **Determinism:** the same inputs give bit-identical results.
- [x] **Invalid input** is rejected with a message (`valid == false`): mismatched pose length, an invalid skeleton or mapping, both weights 0, a negative weight, a marker naming a missing bone, region or actor index, a chain naming a missing bone, non-finite input.
- [x] All presets pass on Windows; format and lint clean. (Linux CI not yet run: not pushed yet.)

Out of scope: root rotation (yaw) alignment, vertical alignment, chains longer than two bones, collisions with the actor's own body, more than two actors, smoothing over time (A3 handles frame-to-frame), and any game or SKSE code.

## Clarifications (from the test writer's questions, 2026-10-16)
- **Whole mapping validated:** `alignPair` rejects any mapping `measureBody` would reject (e.g. an empty limb), even parts it does not use.
- **Other actor's pose during the penetration test:** the other actor as it is at that moment. Actor 0's chains are solved first against actor 1's shifted pose; actor 1's chains are then solved against actor 0's pose including its bent limbs.
- **Losing marker's residual:** measured like every marker, from the source bone's final world position (after all IK) to the target point. A loser on a shared chain is usually not met.
- **Inactive markers:** validated too (actors, bones, regions, numbers).
- **Settings:** `maxRootShiftM < 0` or non-finite is invalid; `poleSamples < 1` is invalid.
- **Chain structure:** `midBone` must be a direct child of `rootBone`, and `endBone` of `midBone`. The solver converts world rotations back to local ones.

## Amendments after the fresh review (2026-10-16)
Known limits (documented, not fixed; a decision for toddhow if they matter in A3):
- **One pass:** actor 0's chains aim at actor 1's pose before actor 1's IK. If both actors' hands target each other (two hand-to-hand markers), the second solve can move the first actor's target, so a marker can end unmet. `met` reports this honestly. Iterating 2–3 passes would fix it, but that changes the approved design.
- **Pole parallel to root→target:** IK falls back to the chain's current plane, so rotating the pole cannot change the pose, and remaining penetration is reported without being searched.

Added criteria (tests to be added by the test writer):
- [x] Chain structure is validated: `midBone` not a child of `rootBone`, `endBone` not a child of `midBone`, invalid hinge limits, and a zero or non-finite `poleDirection` are each rejected.
- [x] Two chains with the same name or the same `endBone` are rejected.
- [x] Huge finite input (e.g. a root at 1e308 m) gives `valid == false`, never NaN results marked valid.
