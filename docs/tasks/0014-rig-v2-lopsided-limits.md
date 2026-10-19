# Task: Rig format version 2, lopsided joint limits
Status: draft, waiting for toddhow's approval. No code yet. toddhow approved doing this as its own task straight after M3 (`docs/tasks/0013-m3-ragdoll-motor-solver.md`, Question 3). Lighter loop: the implementing agent writes the tests, and one fresh review runs at the end.
Design doc section: CLAUDE.md → "First milestones" → M2 (rig format) and M3 (ragdoll); `docs/formats/rig.md` → "Limitation of version 1"; `data/rigs/README.md` → the rows marked *approx.*
Goal: a joint's swing range can be wider one way than the other (hip flexion 120° against extension 30°), so the rig states the published range instead of an averaged cone.
Inputs / outputs:
- `physics/interface`: `JointDesc` gets a swing shape and a minimum and maximum for each swing axis (see "Interface change").
- `physics/jolt`: `createJoint` builds a `SixDOFConstraint` for lopsided joints and keeps the `SwingTwistConstraint` for the others; `setJointMotor` and `setJointTarget` handle both.
- `core/rig`: `rig::Joint`, the loader, writer and validation; rig format version 2, with version 1 files still loading.
- `core/motor`: `buildRagdoll` fills the new `JointDesc` fields. `jointRotation`, `frameModel` and the target formula do not change (see "What identity means").
- `data/rigs/humanoid.rig.json` moves to version 2 with the published ranges; `data/rigs/README.md` and `docs/formats/rig.md` are updated.
- Tests in `tests/unit/core_rig/`, `tests/roundtrip/core_rig/`, `tests/unit/physics_jolt/`, `tests/unit/core_motor/` and `tests/property/core_motor/`.
Constraints:
- **Deterministic:** as M1 and M3: identical calls in identical order give bit-identical body states.
- **No heap allocation per step:** `setJointTarget` and `setJointMotor` stay allocation-free for both joint kinds; `opas_alloc_tests` keeps counting zero.
- **Interface stays thin:** no Jolt types or Jolt words (no "pyramid constraint part") leak into `physics/interface`; the interface describes ranges, the backend picks the Jolt constraint.
- **Symmetric joints are unchanged:** a joint that is symmetric today keeps today's Jolt constraint and settings, so M3's tuned behaviour for those joints does not move.

## What Jolt offers (v5.6.0 as pinned, `build/debug/_deps/joltphysics-src`)
All paths below are under `Jolt/Physics/Constraints/`.

- **Separate frames per body:** `SwingTwistConstraintSettings` has `mTwistAxis1`/`mPlaneAxis1` for body 1 and `mTwistAxis2`/`mPlaneAxis2` for body 2 (`SwingTwistConstraint.h`, lines 35–42). The constructor builds each body's constraint frame from its own pair, X = twist, Y = plane × twist, Z = plane (`SwingTwistConstraint.cpp`, lines 105–113). Our backend sets both pairs to the same axes today (`physics/jolt/src/JoltWorld.cpp`, `createJoint`).
- **Cone is symmetric:** `ESwingType::Cone` is documented as "Cone limits only support limits that are symmetric around 0" (`ConstraintPart/SwingTwistConstraintPart.h`, line 16). It clamps the swing to an ellipse built from the two maximum half-angles only (`Ellipse ellipse(mSinSwingYHalfMaxAngle, mSinSwingZHalfMaxAngle)`, same file, lines 285–297).
- **The constraint part takes separate minimums and maximums:** `SwingTwistConstraintPart::SetLimits(twistMin, twistMax, swingYMin, swingYMax, swingZMin, swingZMax)` (same file, line 48). The pyramid branch clamps each swing axis between its own minimum and maximum half-angle (lines 299–320).
- **But `SwingTwistConstraint` cannot reach asymmetric pyramid limits:** it has `mSwingType` (`SwingTwistConstraint.h`, line 45), but `UpdateLimits` always passes `-mPlaneHalfConeAngle, mPlaneHalfConeAngle, -mNormalHalfConeAngle, mNormalHalfConeAngle` (`SwingTwistConstraint.cpp`, line 89). Through its public settings a pyramid is always symmetric too.
- **`SixDOFConstraint` can:** `SixDOFConstraintSettings` has `mSwingType` and per-axis `mLimitMin`/`mLimitMax` (`SixDOFConstraint.h`, lines 63–79). Its comment says that for a pyramid the Y and Z rotation limits "does not need to be symmetrical around zero", while a cone "needs to be" (lines 75–77). `UpdateRotationLimits` forces symmetry only when the type is `Cone`, then passes all six values to `SetLimits` (`SixDOFConstraint.cpp`, lines 93–119).
  - Its frame is X = `mAxisX1`, Y = `mAxisY1`, Z = X × Y (`SixDOFConstraint.cpp`, lines 160–168). With `mAxisX = twist` and `mAxisY = plane × twist`, Z = twist × (plane × twist) = plane, which is exactly the swing-twist frame that `setJointTarget` documents today.
  - `SetTargetOrientationCS` decomposes and clamps the target to the limits the same way `SwingTwistConstraint` does (`SixDOFConstraint.cpp`, lines 337–349; `SwingTwistConstraint.cpp`, lines 186–198).
  - Motors are per axis: `mMotorSettings[EAxis::Num]` and `SetMotorState(EAxis, ...)` (`SixDOFConstraint.h`, lines 97 and 172), instead of one swing and one twist motor.
- **What the pyramid limits measure:** the clamp writes the swing as `Rz(z) · Ry(y)` and limits `y` and `z` separately (comment in `SwingTwistConstraintPart.h`, lines 300–304). A pure swing about one axis is limited at exactly its angle; a combined swing is limited by a box in those two angles. Jolt notes that both cone and pyramid "deform" at large swing angles (lines 16–17).

## Two ways to get lopsided limits
### A. Parent-side frame centred on the range (keeps `SwingTwistConstraint` and the cone)
- **How:** for each swing axis, centre `c = (min + max) / 2` and half-width `h = (max - min) / 2`. Body 1's frame (`mTwistAxis1`/`mPlaneAxis1`) is turned by the swing-only rotation `C` (no twist component) that points the cone at the centre; body 2's frame stays at rest. The cone half-angles are the `h` values.
- **Effect on targets:** Jolt's constraint-space rotation at rest becomes `C⁻¹` instead of identity. The backend would keep the interface's meaning by turning each target into `C⁻¹ · target` (one quaternion multiply, no allocation).
- **For:** one Jolt constraint type for every joint; M3's motor code and tuning stay; the limit shape stays an ellipse, with no corners.
- **Against:**
  - The ellipse is centred on the range, so the rest pose must lie inside it. Roughly, in the small-angle picture, `(c_y/h_y)² + (c_z/h_z)² < 1`. When both axes are offset, rest can fall outside. Example: a right shoulder with flexion 180°/extension 60° and abduction 180°/adduction 0° has `c_z/h_z = 60/120` and `c_y/h_y = 90/90`, so rest is outside the cone and the arm would be pushed off its rest pose at build. Even with an estimated 30° adduction the sum is about 0.76, close to the edge.
  - A swing about one axis alone is no longer limited at its stated angle when the other axis is offset, because the ellipse narrows away from its centre.
  - Combined offsets are not exact: swing composition is not additive, so the edges are only approximately where the file says.

### B. Asymmetric pyramid limits through `SixDOFConstraint` (recommended)
- **How:** joints with lopsided swing become a `SixDOFConstraint`: translation axes fixed (`MakeFixedAxis`), `mSwingType = Pyramid`, `RotationX` limited to the twist range, `RotationY` to the plane swing range and `RotationZ` to the normal swing range, with `mAxisX1 = mAxisX2 = twist` and `mAxisY1 = mAxisY2 = plane × twist`. Symmetric joints keep today's `SwingTwistConstraint` with a cone.
- **For:**
  - The rest pose is Jolt's constraint-space identity, as it is today. Nothing in `core/motor` changes.
  - The rest pose is inside the limits exactly when each range contains 0, which is a simple rule to validate.
  - Each axis alone is limited at exactly its stated angle.
- **Against:**
  - A second constraint type in the backend: `JointSlot` holds either kind, and `setJointMotor` sets three per-axis motors instead of a swing and a twist motor. Whether `SixDOFConstraint`'s `PositionAndVelocity` mode still damps at zero stiffness, as M3 found for `SwingTwistConstraint`, must be checked in its source during implementation, not assumed.
  - A box has corners: a hip can be fully flexed and fully abducted at once, which a cone would cut off. For the sample's ranges this is plausible for the hip and less so near the shoulder's extremes.
  - Lopsided joints change Jolt constraint type, so M3's tracking and settling results must be re-checked for them. The per-step cost of `SixDOFConstraint` against `SwingTwistConstraint` is not measured; `docs/budgets.md` has no budget yet.

### Recommendation
B, with the interface describing ranges rather than a Jolt mechanism, so that switching to A later (or to another host's joint) changes only `physics/jolt`. The deciding reason is A's rest-pose problem at the shoulder.

## Interface change (`physics/interface`)
- **New enum** in `Descriptions.hpp`: `enum class SwingShape { Cone, Pyramid };`.
- **`JointDesc`** replaces `normalHalfConeRad` and `planeHalfConeRad` with:
  - `SwingShape swingShape = SwingShape::Cone;`
  - `float planeSwingMinRad = 0, planeSwingMaxRad = 0;` swing about Y (plane × twist), positive by the right-hand rule;
  - `float normalSwingMinRad = 0, normalSwingMaxRad = 0;` swing about Z (the plane axis), positive by the right-hand rule.
  - Twist fields are unchanged.
- **Rules, checked by `createJoint` (`InvalidDescription`):**
  - every value in `[-pi, pi]`, `min <= max`;
  - `Cone`: `min == -max` for both swing axes, so a cone is exactly today's joint;
  - `Pyramid`: each swing range contains 0, so the joint is not created outside its own limits.
- **`setJointTarget`:** the contract does not change. Identity is still "the relative pose the joint was created in", in the same frame (X = twist, Y = plane × twist, Z = plane). Its comment gains one sentence: a target outside the limits is clamped to them.
- **Naming the axes by plane and normal** matches the rig file and the v1 meaning: swing about Y keeps the twist axis in the plane containing `planeAxis`; swing about Z keeps it in the plane normal to `planeAxis`.

## What identity means once the rest pose is not the cone centre
- **With B:** identity stays the rest pose everywhere: in `setJointTarget`, in Jolt's constraint space, and in `jointRotation`. The centre of a range has no special role. `frameModel`, `jointRotation` and the target formula `J⁻¹ · A_rest · (A_t⁻¹ · B_t) · B_rest⁻¹ · J` from task 0013 are unchanged. Motors with a zero target pull towards the rest pose, not the middle of the range.
- **With A, if chosen instead:** the interface still means "identity = rest", and the backend applies `C⁻¹` per joint, so `core/motor` would still not change. The alternative, making identity the cone centre in the interface, would push `C` into `RagdollJoint` and `jointRotation` and change every caller's targets; it is not proposed.

## Rig format version 2 (`docs/formats/rig.md`)
- **Header:** `"version": 2`. The loader accepts 1 and 2; anything else is `unsupported version (expected 1 or 2)` at `/version`.
- **Joint keys depend on a new `swing` key:**
  - `"swing": "cone"`: `normalHalfConeRad` and `planeHalfConeRad` as in version 1;
  - `"swing": "pyramid"`: `planeSwingMinRad`, `planeSwingMaxRad`, `normalSwingMinRad`, `normalSwingMaxRad` instead of the half-cone keys.
  - Keys of the other shape are errors (unknown key), as are unknown `swing` values.
- **Canonical key order:** `bone`, `twistAxis`, `planeAxis`, `swing`, then the swing keys in the order listed above, then `twistMinRad`, `twistMaxRad`.
- **Rules:** swing minimums and maximums in `[-pi, pi]`, `min <= max` (reported at the minimum), and each pyramid range contains 0 (reported at the side that excludes it). Twist rules are unchanged.
- **Version 1 files** load as if every joint said `"swing": "cone"`, and validate exactly as today.
- **In memory:** `rig::Joint` holds a `rig::SwingShape swing` (the same two values as the interface enum) and the four min/max doubles; a cone stores `-half` and `+half`. `rig.md` gains a sign table for the swing axes.
- **Writing:** `writeRig` always writes version 2. A version 1 file therefore reads, but writes back as version 2 (the only differences are `"version": 2` and `"swing": "cone"` in each joint). The round-trip gate applies to version 2 files. See Question 3.

## The sample humanoid's ranges
Signs follow from the sample's joint frames (`data/rigs/humanoid.rig.json`). For the limbs (twist axis `[0, 0, -1]`, plane axis `[1, 0, 0]`), Y is world +Y (forward) and Z is world +X (right). A positive normal swing (about +X) moves a hanging limb forward. A positive plane swing (about +Y) moves it towards −X, which is inwards for the right side and outwards for the left. For the trunk and head (twist axis `[0, 0, 1]`), Y is world −Y, so a positive normal swing bends backwards. The sources are as in `data/rigs/README.md` (AAOS 1994, still not checked against the book).

| Joint | Shape | Normal swing (rad) | Plane swing (rad) | Twist (rad) | Notes |
| --- | --- | --- | --- | --- | --- |
| Spine, Chest (each) | pyramid | −0.698 to 0.218 (flexion 40°, extension 12.5°) | −0.305 to 0.305 | ±0.39 | each joint still takes half the trunk |
| Head | cone | 0.785 | 0.785 | ±1.047 | symmetric, unchanged |
| Right UpperArm | pyramid | −1.047 to 2.967 (extension 60°, flexion 170°) | −2.967 to 0.524 (abduction 170°, adduction 30°) | −1.222 to 1.571 (internal 70°, external 90°) | 180° capped at 170°; adduction is an estimate (Question 4) |
| Left UpperArm | pyramid | −1.047 to 2.967 | −0.524 to 2.967 | −1.571 to 1.222 | mirror of the right |
| Forearm (each) | cone (hinge) | 0 | 0 | 0 to 2.618 | unchanged |
| Right Hand | pyramid | −1.222 to 1.396 (extension 70°, flexion 80°) | −0.349 to 0.524 (radial 20°, ulnar 30°) | ±1.396 | assumes palms facing forward (Question 4) |
| Left Hand | pyramid | −1.222 to 1.396 | −0.524 to 0.349 | ±1.396 | mirror of the right |
| Right Thigh | pyramid | −0.524 to 2.094 (extension 30°, flexion 120°) | −0.785 to 0.524 (abduction 45°, adduction 30°) | ±0.785 | |
| Left Thigh | pyramid | −0.524 to 2.094 | −0.524 to 0.785 | ±0.785 | mirror of the right |
| Shin (each) | cone (hinge) | 0 | 0 | −2.356 to 0 | unchanged |
| Right Foot | pyramid | −0.873 to 0.349 (plantar flexion 50°, dorsiflexion 20°) | −0.262 to 0.611 (eversion 15°, inversion 35°) | ±0.175 | |
| Left Foot | pyramid | −0.873 to 0.349 | −0.611 to 0.262 | ±0.175 | mirror of the right |

- **Shoulder twist:** version 1 used ±70° so the sides needed no sign convention. With the convention above, external rotation of the right arm is positive twist about `[0, 0, -1]` and of the left arm negative, so the published 70°/90° can be used.
- **170° cap:** at a 180° swing from the hanging rest pose the swing-twist decomposition has no unique twist, and Jolt warns that pyramids deform at large swings. 170° is an engineering margin, not a published value.
- **Signs are derived, not yet measured:** a test confirms each sign in Jolt, as M3 did for the knee (`RagdollHinges.KneeBendsBackwardsWithNegativeTwist`).

## Acceptance criteria
- [ ] **Loader, version 2:**
  - the version 2 sample loads, validates and writes back byte-for-byte;
  - every version 1 test file still loads, and gives the same `rig::Joint` values as before (cones as `-half`/`+half`);
  - located errors (file, line, JSON pointer) for: version 3; unknown `swing` value; half-cone keys on a pyramid joint and min/max keys on a cone joint; `min > max` (at the minimum); a pyramid range that excludes 0 (at the offending side); a value outside `[-pi, pi]`.
- [ ] **Interface rules:** `createJoint` rejects, with `InvalidDescription`, a cone with `min != -max`, a pyramid range that excludes 0, `min > max`, and values outside `[-pi, pi]`.
- [ ] **Pyramid limits in Jolt:** a two-body test (kinematic parent box, dynamic child capsule, very stiff motor as in M3's property test, 60 Hz) on a pyramid joint with plane swing −0.5 to 1.5 and normal swing −0.3 to 0.9 rad:
  - a target on one axis at 1.2 rad (inside the range, but outside the symmetric cone of half-width 1.0 that version 1 would have used) is reached within 0.02 rad after 2 s;
  - a target 0.3 rad beyond each of the four limits settles within 0.05 rad of that limit (M3's "limits plus 0.05 rad") and within 0.05 rad of 0 on the other axis;
  - at rest the joint measures identity within 1e-5 rad.
- [ ] **Cone joints unchanged:** a cone `JointDesc` still creates a `SwingTwistConstraint` with the same settings as before (checked through the backend's test hook or by behaviour: the existing M1 and M3 joint tests pass unedited apart from the field rename).
- [ ] **Signs on the sample:** with a kinematic pelvis, a right hip target of normal swing +1.9 rad moves the right foot forward (+Y) and is reached within 0.05 rad; a target of −0.9 rad is held at the −0.524 rad limit within 0.05 rad. One test per sample joint kind (trunk, shoulder, wrist, hip, ankle) confirms the sign of each swing axis by the direction a body moves.
- [ ] **Rest pose inside the limits:** every sample joint's rest pose is at least 0.2 rad inside each of its swing and twist limits, except hinge zero ends (elbow and knee at 0, as today).
- [ ] **M3 still holds with the version 2 sample:** builds 16 bodies, 15 joints, 75 kg; falls and settles (limits plus 0.05 rad, against the new limits); tracks the arm wave within 0.10 rad RMS and 0.20 rad peak at 30, 60 and 120 Hz; the 30 Hz against 120 Hz rule as signed off in task 0013.
- [ ] **Property test:** M3's seeded random-pose test, with each pyramid joint's angles drawn independently inside its box, reaches every target within 0.02 rad.
- [ ] **No heap allocation per step:** `opas_alloc_tests` counts zero `operator new` calls over 600 driven steps with the version 2 sample.
- [ ] **Determinism:** two worlds running the tracking scenario with the version 2 sample give bit-identical body states every step.
- [ ] **Docs:** `docs/formats/rig.md` describes version 2 and its sign table and drops the "Limitation of version 1" note; `data/rigs/README.md` replaces the *approx.* rows with the table above; the `CLAUDE.md` files of `core/rig`, `core/motor` and `physics/jolt` note the change; all existing tests pass.

Out of scope:
- Limits that depend on each other beyond a box (for example a hip whose abduction shrinks when it is flexed).
- Changing gains or the profile format, unless M3's tolerances fail for a lopsided joint (then stop and report, see Question 6).
- Target angular velocity for the motors (the M3 follow-up).
- Checking the AAOS values against the book.
- Any OpenMW code.

## Questions for toddhow
1. **Approach:** B (asymmetric pyramid through `SixDOFConstraint`, symmetric joints unchanged) as recommended, or A (cone centred by turning the parent's frame)? A keeps one constraint type and a cone shape, but its rest pose can fall outside the cone at the shoulder.
2. **Shapes per joint:** keep cones for symmetric joints (head, elbows, knees), as proposed, so they behave exactly as in M3? The alternative is pyramids everywhere: one code path, but every joint's shape changes and M3's results must be re-checked for all of them.
3. **Version 1 writing:** is "version 1 reads, the writer always writes version 2, the round-trip gate covers version 2" acceptable? Keeping a version 1 writer would let old files round-trip byte-for-byte, at the cost of `rig::Rig` remembering its version.
4. **Sample values that are not published:**
   - shoulder adduction 30° (the table in `data/rigs/README.md` gives none, and 0° would put the hanging arm on its limit);
   - the 170° cap on shoulder flexion and abduction;
   - the wrist's sign assumes palms facing forward. Version 1 put flexion on the normal cone, which implies it, but the README does not state the rest palm orientation. Is that the intended pose?
5. **Test edits:** existing tests build `JointDesc` and `rig::Joint` with the half-cone fields (`tests/common/RagdollTestSupport.hpp`, `tests/common/RigTestData.hpp`, `tests/unit/physics_jolt/PhysicsTestHelpers.hpp`, `tests/unit/physics_interface/ValueTypesTest.cpp`, `tests/unit/core_rig/RigLoadTest.cpp`, `tests/roundtrip/core_rig/RigProfileRoundTripTest.cpp`, `tests/property/core_motor/JointTargetPropertyTest.cpp`). Does approving this spec approve renaming those fields in them, with no tolerance changed?
6. **If a lopsided joint misses M3's tolerances** after moving to `SixDOFConstraint` (for example because its per-axis motors or torque limits act differently): may the implementer retune that joint's gains in the sample profile, or stop and report?
