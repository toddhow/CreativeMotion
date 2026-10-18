# Task: A5 Body surfaces from mesh sampling, and finger contact
Status: approved by toddhow 2026-10-17 with all proposals. Lighter workflow (CLAUDE.md, "Lighter loop for A4 to A6").
Design doc section: CLAUDE.md → "Alignment plugin track" (A5 replaces the "Later, not scheduled" items "body surfaces from mesh sampling" and "finger contact", toddhow 2026-10-17).
Goal: contacts land on each actor's real body surface instead of a fixed-ratio capsule, and hands close around what they touch.
Inputs / outputs: main repository: a capsule fit (`kinematic/markers`) and a finger curl solver (`kinematic/ik`), both engine-independent. Plugin: reading an actor's skinned body mesh, per-actor body maps, finger curls after arm IK.
Constraints: as A4 (never crash, deterministic core, no game data in repositories). Measuring runs once per actor and 3D load, never per frame.

## A5a Capsule fit from surface points (main repository)
1. **Input:** points on an actor's skin in model space, each with the index of the bone that weighs it most, plus the skeleton's world rest pose and a body map.
2. **Fit:** for each capsule rule, take the points whose main bone is the rule's `bone` and that project onto the segment from `bone` to `endBone`. The radius is a high percentile (proposal: 90th) of their distances to the segment, so a few stray vertices (hair, jewellery) do not inflate it. Fewer than a minimum count (proposal: 20) keeps the rule's ratio radius.
3. **Output:** a copy of the body map whose `radiusRatio` per capsule is `fitted radius / capsule length`, so `alignPair` and `measureBody` use it unchanged (no API or format change).
4. Deterministic (sorted inputs, `std::nth_element` on a copy is fine); never throws.

## A5b Sampling the body mesh in game (plugin)
Engine facts (CommonLibSSE-NG v12.1.0):
- A skinned mesh is a `BSGeometry` with a `skinInstance` (`include/RE/B/BSGeometry.h:63`). The skin instance has `skinData`, `skinPartition`, `rootParent` and the `bones` array (`include/RE/N/NiSkinInstance.h:41-44`).
- `NiSkinData` has per-bone `skinToBone` transforms (`include/RE/N/NiSkinData.h:36`, accessor `GetBoneDataSkinToBone`, `:77`).
- `NiSkinPartition` partitions hold bone lists, weights and vertex counts (`include/RE/N/NiSkinPartition.h:36-50`), and the partition holds `rawVertexData` (`:23`).
- The vertex layout is described by `BSGraphics::VertexDesc` with position (`VA_POSITION`) and skinning (`VA_SKINNING`) attributes and a full-precision flag (`include/RE/V/VertexDesc.h:13-39`).

Rules:
1. At scene start (and after a 3D reload), find the actor's body shape: the skinned `BSGeometry` under `Get3D()` whose skin bones include the spine bones (proposal: the one with the most vertices among those).
2. Read each vertex's position and main bone from `rawVertexData` with the layout from `VertexDesc`, and move it into skeleton space with that bone's `skinToBone` inverse and the bone's current world transform.
3. Hand the points to A5a and use the fitted body map for this actor in IK and alignment. Cache per form ID; log the fitted radii once.

**Unverified, checked in game first (as the A4c-2 smoke test was):** that `rawVertexData` is filled on the CPU side for actor bodies; the exact vertex layout (offsets of position and bone indices or weights); that partition bone indices refer to `skinInstance->bones`. Step 1 is a debug build that logs vertex counts and a few decoded positions next to the bone positions, before any fitting.

## A5c Finger contact
1. **Fingers:** five chains per hand, three joints each. Bone names come from the shipped skeleton (vanilla `NPC R Finger00 [RF00]` to `NPC R Finger42 [RF42]`; checked in game by the "bone not found" log before relying on them).
2. **Curl solver (main repository, `kinematic/ik`):** given a finger's joint world transforms, a hinge axis per joint, joint limits, and a target capsule, find one curl parameter `t` in [0, 1] (each joint turns by `t` times its limit, distal joints weighted more) where the fingertip first touches the capsule surface. The fingertip is the last joint plus the previous segment's length along it (the skeleton has no tip bone). Bisection, so it is deterministic and bounded. No contact within the limits keeps `t = 1` (a closed fist is not forced; proposal: cap at the limit).
3. **When:** only for a hand whose marker is met (residual within tolerance), against that marker's target capsule; blended in and out with the A4c-2 blender.
4. **Hinge axes:** per finger in the body map (new optional `fingers` section, body map format v2), so other skeletons can set their own; the shipped map gets values found by a debug build that bends each finger joint about each local axis in turn (like the elbow smoke test).

## Acceptance criteria
- [ ] A5a: unit tests on synthetic point clouds (a cylinder of known radius gives that radius within 1 mm; outliers above the percentile do not change it; too few points keep the ratio).
- [ ] A5b: the debug build's decoded vertices lie on the body in game (toddhow judges against the log); fitted radii differ visibly between a slim and a heavy body.
- [ ] A5c: unit tests for the curl solver (a capsule placed at a known distance is touched at the computed `t`; limits respected; no contact gives `t = 1`); in game, a hand on a shoulder closes around it (toddhow judges).

## Decisions for toddhow
1. **Percentile and minimum count** for the fit: 90th percentile, 20 points (proposal).
2. **Which mesh is "the body":** the skinned shape with the spine bones and the most vertices (proposal); armour on top is ignored.
3. **Finger model:** one curl parameter per finger (proposal), not full per-joint IK.
4. **Body map format v2** with an optional `fingers` section (proposal).

Out of scope: per-vertex contact (surfaces stay capsules), feet and toes, deforming the mesh.
