# Task: A2 Alignment solver
Status: draft — awaiting toddhow's review of the design and the two decisions below.
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
- [ ] Format v2 (`sourceActor`, `targetActor`) documented, with load/round-trip/error tests (actor index not an integer, negative, or the same as the other one). The A1 marker tests are moved to v2.
- [ ] `rig::computeWorldPose` matches hand-computed world transforms for a posed 3-bone chain with a non-identity root.
- [ ] Root alignment, hand-computed: a single marker with a horizontal error d is fully closed. Weights 1:1 split it half and half; 1:0 moves only actor 0; 0:1 only actor 1. Z error is ignored. A shift larger than `maxRootShiftM` is clamped and reported.
- [ ] **Body-size scenes** (synthetic humanoids built in tests, scaled 0.8 and 1.25 — a 1.56× height ratio — in both pairings): (a) a hug, with both actors' hands on the other's upper back; (b) a handshake, right hand to right hand; (c) one actor's hand on the other's shoulder. Every active marker is met within its `toleranceM` (0.02 m in the scenes), and no limb penetrates more than 0.005 m.
- [ ] **Penetration avoidance:** a scene where the default pole drives the elbow through the other actor's torso capsule ends with penetration ≤ 0.005 m and the marker still met.
- [ ] **Priorities:** two markers on one hand; the higher priority drives the chain. On equal priority, the smaller id wins.
- [ ] **Windows:** markers outside the clip time are ignored; a marker at exactly `startS` or `endS` is active.
- [ ] **Determinism:** the same inputs give bit-identical results.
- [ ] **Invalid input** is rejected with a message (`valid == false`): mismatched pose length, an invalid skeleton or mapping, both weights 0, a negative weight, a marker naming a missing bone, region or actor index, a chain naming a missing bone, non-finite input.
- [ ] All presets pass on Windows; format and lint clean.

Out of scope: root rotation (yaw) alignment, vertical alignment, chains longer than two bones, collisions with the actor's own body, more than two actors, smoothing over time (A3 handles frame-to-frame), and any game or SKSE code.
