# Task: M1 Physics interface and Jolt backend
Status: done — approved by toddhow on 2026-10-16, including the CMake include check and the review amendments.
Design doc section: CLAUDE.md → "First milestones" → M1. **The design doc's interface section is not in the repository; toddhow approved drafting the interface here from the guide (2026-10-16). Reconcile with the design doc once it is exported.**
Goal: A thin, engine-neutral physics interface (headers only) and a Jolt implementation of it, good enough to build, drive, read, query and step a ragdoll's rigid bodies and joints.
Inputs / outputs: new `physics/interface/` (headers, CMake INTERFACE target `opas::physics_interface`); `physics/jolt/` gains the backend and a factory; `tests/unit/physics_interface/`, `tests/unit/physics_jolt/`; new `tools/CheckJoltIncludes.cmake` (a CMake script, since it then runs anywhere CMake does) with ctest self-tests and a CI job. The M0 `BackendInfo` helpers are kept.
Constraints:
- `physics/interface` includes only standard-library headers; no Jolt (or any third-party) type appears in its signatures.
- Deterministic: identical calls in identical order give bit-identical results (Jolt guarantees this under the same conditions, see "Deterministic Simulation" in Jolt `Docs/Architecture.md` at the pinned commit).
- Single-threaded from the caller's view: one thread owns a world; all callbacks run on that thread. M1 uses Jolt's `JobSystemSingleThreaded`. Multithreading is a later task.
- Units SI; right-handed, Z up; default gravity (0, 0, −9.81) m/s²; default fixed step 1/60 s.
- No heap allocation in `step()` from our own code is a goal but is only *enforced* from M3.

## Interface draft (for review)

Names are illustrative; the shape is what is being approved. All in namespace `opas::physics`.

**Value types** — plain structs, `float` to match Jolt's single-precision build:
- `Vec3 {x, y, z}`, `Quat {x, y, z, w}` (unit quaternion, identity = {0,0,0,1}).
- `BodyId`, `JointId`, `StepListenerId` — opaque handles (index + generation). Default-constructed = invalid. A handle to a destroyed object never aliases a new one.

**Shapes and bodies (build)**
- `Shape` = one of `Box{halfExtentsM}`, `Sphere{radiusM}`, `Capsule{halfHeightM, radiusM}` (capsule axis is the body's local **Z**, matching our Z-up convention; the backend converts).
- `BodyDesc { Shape shape; MotionType motion (Static | Dynamic | Kinematic); Vec3 positionM; Quat rotation; float massKg (Dynamic only, > 0); float friction; float restitution; float linearDamping; float angularDamping; }`
- `CreateResult<BodyId> createBody(const BodyDesc&)` — on failure the id is invalid and `error` is a code plus a message naming the bad field (e.g. `"massKg must be > 0 for a dynamic body"`).
- `bool destroyBody(BodyId)` — also destroys every joint attached to the body. Returns false for a stale handle.

**Joints (build + drive)**
- `JointDesc { BodyId bodyA, bodyB; Vec3 anchorM (world); Vec3 twistAxis (world, unit); Vec3 planeAxis (world, unit, ⟂ twist); float normalHalfConeRad, planeHalfConeRad, twistMinRad, twistMaxRad; }` — a swing-twist joint (Jolt's `SwingTwistConstraint`, the joint Jolt's own ragdolls use). A hinge is a swing-twist with zero cones.
- `CreateResult<JointId> createJoint(const JointDesc&)`; `bool destroyJoint(JointId)`.
- `JointMotor { float stiffnessNmPerRad; float dampingNmsPerRad; float maxTorqueNm; }` — both stiffness and damping 0 = motor off (joint is free within its limits).
- `bool setJointMotor(JointId, const JointMotor&)`
- `bool setJointTarget(JointId, Quat targetRotation)` — target rotation of body B's joint frame relative to body A's joint frame; identity = the relative pose at creation.

**Drive bodies**
- `bool applyImpulse(BodyId, Vec3 impulseNs, Vec3 worldPointM)`
- `bool setVelocity(BodyId, Vec3 linearMps, Vec3 angularRadps)`
- `bool setKinematicTarget(BodyId, Vec3 positionM, Quat rotation)` — reached at the end of the next step.

**Read**
- `BodyState { Vec3 positionM; Quat rotation; Vec3 linearVelocityMps; Vec3 angularVelocityRadps; bool awake; }`
- `bool readBody(BodyId, BodyState& out)`
- `std::size_t readBodies(std::span<const BodyId>, std::span<BodyState> out)` — no allocation; returns how many were valid; stale entries are left untouched and flagged by the return count.

**Query**
- `std::optional<RayHit> castRay(Vec3 originM, Vec3 directionTimesLengthM, BodyId ignore = {})` → closest hit `{ BodyId body; Vec3 pointM; Vec3 normal; float fraction; }`. (Contacts for sensors come in M5.)

**Step**
- `WorldConfig { float fixedStepSeconds = 1/60; Vec3 gravityMps2; uint32 maxBodies; uint32 maxJoints; }`
- `void step()` — advances exactly one fixed step.
- `StepListener` with `onPreStep(PhysicsWorld&, float dtSeconds)` and `onPostStep(...)`; `StepListenerId addStepListener(StepListener&)`, `bool removeStepListener(StepListenerId)`. Called once per step, in registration order. Listeners may drive and read, but creating or destroying bodies/joints during a callback fails with `ErrorCode::CalledDuringStep`. This is the hook the behavior core runs in.

**Ownership**
- `PhysicsWorld` is an abstract class; the host owns it through `std::unique_ptr`. Destroying the world destroys all its bodies and joints; handles are then meaningless.
- `physics/jolt` exposes `std::unique_ptr<PhysicsWorld> createJoltWorld(const WorldConfig&, std::string* errorOut = nullptr)`. Several worlds may coexist; Jolt's process-wide registration is reference-counted by the backend.
- Listeners are not owned: the caller must remove a listener before destroying it.

## Acceptance criteria
- [x] Every header in `physics/interface` compiles in a test that does not link or include Jolt (the test target links only `opas::physics_interface`).
- [x] **Box drop:** 1 m cube (half extents 0.5, 10 kg) released at rest from centre height 2.0 m above a static floor whose top is z = 0. After 3 s at 60 Hz: centre z ∈ [0.48, 0.50] m (Jolt allows up to 0.02 m penetration, `mPenetrationSlop`), linear speed < 0.03 m/s (Jolt's sleep threshold), tilt < 1°.
- [x] **Determinism:** two worlds built with the same calls (a tilted, spinning box falling onto three stacked boxes, 300 steps), stepped interleaved, give bit-identical `BodyState` for every body at every step.
- [x] **Impulse:** in zero gravity, impulse J at the centre of a body of mass m gives linear velocity J/m (relative error ≤ 1e-5).
- [x] **Kinematic:** after `setKinematicTarget` and one step, the kinematic body is at the target (≤ 1e-5 m, ≤ 1e-4 rad).
- [x] **Raycast:** a ray straight down from (0, 0, 5) hits the floor at (0, 0, 0) with normal (0, 0, 1) (≤ 1e-4); a ray that misses returns no hit; `ignore` skips the given body.
- [x] **Joint motor:** in zero gravity, a 1 kg dynamic box (half extents 0.1 m) jointed to a static body, twist limits ±90°, motor stiffness 50 N·m/rad, damping 5 N·m·s/rad, unlimited torque, target 45° about the twist axis: after 2 s at 60 Hz the relative twist is 45° ± 1°.
- [x] **Joint limits:** same setup with the motor off and twist limits ±30°; give the box 10 rad/s about the twist axis; for every step over 1 s the twist stays within 30° + 2°.
- [x] **Handles:** after `destroyBody`, reading, driving or destroying that handle again returns false; joints attached to it are gone; a newly created body never gets an id equal to the destroyed one.
- [x] **Validation:** each invalid `BodyDesc`/`JointDesc` field (non-positive mass on a dynamic body, non-positive extents/radius, non-unit or non-perpendicular axes, invalid body handle, twistMin > twistMax) is rejected with a message naming the field.
- [x] **Step listeners:** pre and post are each called once per step, in registration order, with dt = fixedStepSeconds; structural changes during a callback fail with `CalledDuringStep`; a removed listener is not called.
- [x] **Several worlds:** two worlds can be created, used and destroyed in any order without affecting each other.
- [x] **CI check:** `tools/CheckJoltIncludes.cmake` fails when any file outside `physics/jolt/` includes a Jolt header, passes on the repository, and is run by a CI job (`jolt-includes`; the job has not run yet: no GitLab project).
- [x] All presets build and pass on Windows locally; clang-format and clang-tidy clean.

## Amendments after the fresh review (2026-10-16)

The reviewer found behaviors the draft left unstated; the interface comments now define them. Added criteria (tests to be added by the test writer):
- [x] **Kinematic hold:** after `setKinematicTarget` and one step, further steps without a new target leave the body at the target (≤ 1e-5 m) with zero velocity (≤ 1e-6, a test choice).
- [x] **Damping-only motor:** stiffness 0, damping 5 N·m·s/rad; a jointed box spun at 2 rad/s about the twist axis (zero gravity) slows to < 0.5 rad/s within 1 s. (Motor off: it keeps spinning.)
- [x] **Motor validation:** `setJointMotor` returns false for negative or non-finite (NaN, infinite) stiffness, damping or maxTorqueNm; unlimited torque is float max.
- [x] **Listener changes during a step:** a listener that removes itself in `onPreStep` does not stop the next listener being called that step, and is not called again; a listener added during a step is first called (pre and post) on the next step.
- [x] **Capsule axis:** a capsule (halfHeight 0.5, radius 0.25) created with identity rotation is long along Z: a ray down from (0, 0, 5) hits its top at z = 0.75 above its centre (≤ 1e-3).
- [x] **Drive wakes sleeping bodies:** a box that has gone to sleep on the floor starts moving after `applyImpulse`.

Out of scope: collision filtering between bodies of one ragdoll (M3), contact reporting (M5), multithreaded stepping, shapes beyond box/sphere/capsule, heap-allocation counter (M3), any `core/` code.
