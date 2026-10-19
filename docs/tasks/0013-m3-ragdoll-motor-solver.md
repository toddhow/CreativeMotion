# Task: M3 Ragdoll and motor solver
Status: approved by toddhow on 2026-10-18 with all four proposals: collision groups in the interface, 0.10/0.20 rad tolerances with a kinematic pelvis, lopsided limits as a separate task after M3, and the builder in core/motor. Lighter loop: the implementing agent writes the tests and one fresh review runs at the end of the milestone.
Design doc section: CLAUDE.md → "First milestones" → M3; OpenMW Animation System — Implementation Doc → "Animation component" (the motor solver row turns targets into Jolt motor settings).
Goal: the sample rig becomes a ragdoll in a physics world, and a motor solver makes it follow an animated target pose every physics step, stably at 30, 60 and 120 Hz and without heap allocation per step.
Inputs / outputs:
- New module `core/motor` (`opas::motor`). It depends on `core/math`, `core/rig` and `physics/interface`, as the guide allows `core` to.
- A small addition to `physics/interface` and `physics/jolt`: collision groups (see "Physics interface change").
- Tests in `tests/unit/core_motor/`, `tests/property/core_motor/` and `tests/unit/physics_jolt/`, plus a second test executable for the allocation check.
Constraints:
- **Deterministic:** identical calls in identical order give bit-identical body states, as M1 already guarantees for the world.
- **No heap allocation in the per-step update:** buffers are sized when the ragdoll is built.
- **Threading:** a ragdoll is used from the thread that owns its world.
- **Handles, not pointers:** the ragdoll stores body and joint handles, never pointers into the world, and every call takes the world as a parameter (guide: "store handles or IDs, never raw pointers, to anything a host or backend can destroy").

## Physics interface change: collision groups
- **The problem:** today every moving body collides with every other. A ragdoll's neighbouring bodies overlap at their shared joint, for example where the thigh capsule meets the pelvis, so they would push each other apart from the first step. Jolt's own ragdolls solve this with a group filter (`Ragdoll::DisableParentChildCollisions` in `Jolt/Physics/Ragdoll/Ragdoll.h`, built on `GroupFilterTable` in `Jolt/Physics/Collision/GroupFilterTable.h`, Jolt v5.6.0 as pinned).
- **The proposal, in `opas::physics`:**
  - a new handle type, `CollisionGroupId`;
  - `CreateResult<CollisionGroupId> createCollisionGroup(std::uint32_t subGroupCount)`;
  - `bool setSubGroupsCollide(CollisionGroupId, std::uint32_t a, std::uint32_t b, bool collide)`;
  - `bool destroyCollisionGroup(CollisionGroupId)`.
- **Bodies join a group** through new `BodyDesc` fields: `collisionGroup` and `subGroup`. The default is an invalid group, meaning "collides with everything", so existing code is unchanged.
- **Rules:**
  - two bodies in the same group collide unless their sub-groups were set not to;
  - bodies in different groups, or in no group, always collide;
  - a group may be destroyed while bodies still use it, and those bodies then keep its last settings. In Jolt each body holds a reference to the table, so this is safe.
  - The same rule as for other structural calls applies: creating or destroying a group, or changing its settings, from a step callback fails with `CalledDuringStep`.
- **Jolt backend:** one `GroupFilterTable` per group, set on each member body's `CollisionGroup`.

## Building a ragdoll (`core/motor/Ragdoll.hpp`)
- **The call:** `CreateResult<Ragdoll> buildRagdoll(PhysicsWorld&, const rig::Rig&, const rig::Profile&, const math::Transform& rootWorld, const RagdollOptions&)`. Rig and profile must have passed `validateRig` and `validateProfile`; the build checks this again and fails with the problem's message.
- **What it creates:**
  - one dynamic body per rig body, placed in the rest pose under `rootWorld`, with its shape, mass and the profile's material;
  - one joint per rig joint, anchored at the joint bone's rest origin, with the rig's axes turned into world space, its limits, and the profile's motor gains;
  - one collision group with a sub-group per body, in which each jointed pair does not collide.
- **Options:** `RagdollOptions::rootMotion` is `Dynamic` (default) or `Kinematic`. A kinematic root lets tests and the sandbox hold the pelvis like a puppet while the limbs track, until the balance work in M5 can keep a free body standing.
- **Ownership:** a `Ragdoll` is a plain value holding the handles plus the per-step buffers. The host owns it and calls `destroyRagdoll(world, ragdoll)` before destroying the world. There is no destructor that reaches into the world, because the ragdoll holds no pointer to it.
- **Failure:** if any create call fails, everything already created is destroyed again and the error is returned.
- **Precision:** rig values are `double` and physics values `float`. The conversion happens only here and in the solver.

## Motor solver (`core/motor/MotorSolver.hpp`)
- **The call:** `void driveTowardsPose(PhysicsWorld&, Ragdoll&, std::span<const math::Transform> localPose)`.
  - `localPose` has one local transform per skeleton bone, in the same form as `Skeleton::localRest`.
  - It computes each body's target world rotation by forward kinematics, so bones without bodies (neck, fingers) still count.
  - From those it sets each joint's target in joint-frame axes, as `setJointTarget` defines them.
- **The target formula:** with `A` and `B` the parent and child body world rotations, `_rest` their rest values, `_t` their targets and `J` the joint's world frame at creation,
  `target = J⁻¹ · A_rest · (A_t⁻¹ · B_t) · B_rest⁻¹ · J`.
  This is identity at the rest pose. Quaternions are normalized, and the shorter path is taken (`q` or `-q`), following the guide's quaternion rule.
- **Kinematic root:** if the root body is kinematic, the solver also moves it to its target with `setKinematicTarget`.
- **When it runs:** from a pre-step listener, once per physics step. `RagdollDriver` is a small `StepListener` that holds a pose source (a callback that fills `localPose` for the current time) and calls `driveTowardsPose`. The host registers it with the world.
- **Translation targets:** ignored. Joints only rotate; bone translations come from the rig.

## Acceptance criteria
- [ ] **Collision groups:**
  - two boxes in one group with collision turned off between their sub-groups pass through each other;
  - with collision turned back on, they collide;
  - bodies in different groups, or in none, collide;
  - creating or destroying a group, or changing its settings, during a step fails with `CalledDuringStep`.
- [ ] **Build:** the sample rig and profile give 16 bodies and 15 joints. Their poses match the rig's rest pose within 1e-5 m and 1e-5 rad, and the total mass is 75 kg. A failed create in the middle of the build leaves no bodies or joints behind (tested with `WorldConfig::maxJoints` set too small).
- [ ] **Falls and settles:** the sample ragdoll is released standing above a static floor, with the default profile's motors turned off (damping only). After 5 s at 60 Hz:
  - every body is asleep, or slower than 0.05 m/s and 0.05 rad/s;
  - no body is more than 0.02 m into the floor (Jolt's penetration slop, as in M1);
  - every joint is within its limits plus 0.05 rad;
  - no value is NaN.
- [ ] **Tracking:** the pelvis is kinematic and held still, and every other joint tracks a procedural target defined in the test, with the default profile gains.
  - The target is an arm wave: the right shoulder swings 0.6 rad about its plane axis and the right elbow bends between 0.3 and 1.2 rad, both at 0.5 Hz. Every other joint holds the rest pose.
  - After 1 s of settling, over the next 4 s, the angle between each joint's actual and target relative rotation stays at most 0.10 rad RMS and 0.20 rad at its peak, for every joint.
  - This holds at 30, 60 and 120 Hz.
- [ ] **Stability:** the same run, plus 10 s of the full ragdoll falling with motors on, at each of 30, 60 and 120 Hz:
  - no value is NaN;
  - no body is faster than 20 m/s or 50 rad/s;
  - the tracking error at 30 Hz is at most twice the error at 120 Hz.
- [ ] **Target formula:** a property test with seeded random poses within the joint limits sets the target, steps with very stiff motors and a kinematic root, and checks each joint's measured relative rotation matches its target within 0.02 rad. This also confirms the hinge twist signs that `data/rigs/README.md` left for M3.
- [ ] **No heap allocation per step:** a separate test executable (`opas_alloc_tests`) replaces global `operator new` and `operator delete` with counting versions. After one warm-up step, 600 steps of `RagdollDriver` plus `world.step()` count zero allocations. If Jolt's own step is found to allocate through `operator new`, that is reported to toddhow rather than hidden; Jolt allocates through its own hooks and a preallocated `TempAllocatorImpl` in our backend.
- [ ] **Determinism:** two worlds running the same tracking scenario give bit-identical body states every step.
- [ ] The module has its own `CLAUDE.md`, and all existing tests still pass.

Out of scope:
- Balance, sensors and hit reactions (M5).
- Recording and replay (M4).
- Gravity compensation, and gains that change with the pose.
- Lopsided joint limits (see Questions).
- Building a ragdoll from an animated pose instead of the rest pose.
- Multithreading.
- Any OpenMW code.

## Questions for toddhow
1. **Interface change:** is the collision-group addition to `physics/interface` and `physics/jolt` approved as proposed? The alternative is a ragdoll-specific call that only disables collision between jointed pairs. It is simpler, but the behavior core will later need groups anyway, for example for a character holding an object.
2. **Tolerances:** are 0.10 rad RMS and 0.20 rad peak right for "tracks within tolerance", and is a kinematic pelvis right for the tracking test until M5?
3. **Lopsided limits:** joint cones stay symmetric in M3, because Jolt's cone limits are symmetric around 0 (`ESwingType::Cone` in `SwingTwistConstraintPart.h`). The fix gives the joint a separate frame on the parent side, centred on the middle of the range. Jolt supports that through `mTwistAxis1`/`mPlaneAxis1` versus `mTwistAxis2`/`mPlaneAxis2` in `SwingTwistConstraintSettings`. Proposal: a separate task straight after M3, version 2 of the rig format. Should it be done instead as part of M3?
4. **Module:** should the ragdoll builder live in `core/motor` next to the solver, as proposed, or in a new `core/ragdoll`? The guide's layout lists only `core/motor`.
