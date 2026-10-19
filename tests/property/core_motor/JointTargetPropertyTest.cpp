#include "RagdollTestSupport.hpp"
#include "TestSupport.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

// The target formula (docs/tasks/0013): for seeded random joint rotations
// within the limits, a pose built to produce them, driven with very stiff
// motors, ends with every joint at its target. This checks the formula and
// the joint-frame convention against Jolt, including hinge signs.

namespace {

using namespace opas;

// A random joint-frame rotation q = swing * twist (Jolt's split), using at
// most half of each limit so the swing stays well inside Jolt's elliptical
// cone and bodies rarely touch.
math::Quat randomWithinLimits(test::SeededRandom& random, const rig::Joint& joint)
{
    const double mid = 0.5 * (joint.twistMinRad + joint.twistMaxRad);
    const double half = 0.25 * (joint.twistMaxRad - joint.twistMinRad);
    const double twist = random.uniform(mid - half, mid + half);
    const double swingY = random.uniform(-0.5, 0.5) * joint.planeHalfConeRad;
    const double swingZ = random.uniform(-0.5, 0.5) * joint.normalHalfConeRad;
    const double swingAngle = std::hypot(swingY, swingZ);
    const math::Quat swing =
        swingAngle > 0.0
            ? math::fromAxisAngle(math::Vec3{0.0, swingY / swingAngle, swingZ / swingAngle},
                                  swingAngle)
            : math::Quat{};
    return math::normalized(swing * math::fromAxisAngle(math::Vec3{1.0, 0.0, 0.0}, twist));
}

// Local bone rotations that give each joint its chosen rotation: inverting
// jointRotation, B = A * A_rest^-1 * J * q * J^-1 * B_rest, bone by bone,
// parents first. Bones without a joint keep their rest rotation.
std::vector<math::Transform> poseFor(const rig::Rig& rig, const motor::Ragdoll& ragdoll,
                                     const std::vector<math::Quat>& jointTargets)
{
    const std::size_t n = rig.skeleton.bones.size();
    std::vector<math::Quat> boneModel(n);
    std::vector<math::Transform> pose(n);
    for (std::size_t i = 0; i < n; ++i) {
        const rig::Bone& bone = rig.skeleton.bones[i];
        const math::Quat parentModel =
            bone.parent < 0 ? math::Quat{} : boneModel[static_cast<std::size_t>(bone.parent)];
        boneModel[i] = math::normalized(parentModel * bone.localRest.rotation);
        if (const auto j = rig::findJoint(rig, bone.name)) {
            const motor::RagdollJoint& joint = ragdoll.joints[*j];
            const motor::RagdollBody& parentBody = ragdoll.bodies[joint.parentBody];
            const motor::RagdollBody& childBody = ragdoll.bodies[joint.childBody];
            const math::Quat parentBodyModel =
                boneModel[parentBody.bone] * parentBody.local.rotation;
            const math::Quat childBodyModel =
                parentBodyModel * math::conjugate(parentBody.restModel) * joint.frameModel *
                jointTargets[*j] * math::conjugate(joint.frameModel) * childBody.restModel;
            boneModel[i] =
                math::normalized(childBodyModel * math::conjugate(childBody.local.rotation));
        }
        pose[i].translation = bone.localRest.translation;
        pose[i].rotation = math::normalized(math::conjugate(parentModel) * boneModel[i]);
    }
    return pose;
}

TEST(JointTargetProperty, StiffMotorsReachRandomTargetsWithinLimits)
{
    const rig::Rig rig = test::sampleRig();
    rig::Profile stiff = test::sampleProfile();
    // Stiff relative to the default profile (300), and well damped. Measured
    // while writing M3 (8 s, worst joint error over the last 3 s, 60 Hz):
    // 2000/60 kept the light arms oscillating (0.81 rad) while 2000/200,
    // 2000/600, 600/60 and 300/30 all settled to 0.001 rad or better, as did
    // 2000/60 at 240 Hz. Underdamped stiff motors on light bodies do not
    // settle at 60 Hz; see core/motor/CLAUDE.md.
    stiff.defaultMotor = rig::MotorGains{2000.0, 200.0, 1e6};
    stiff.joints.clear();
    test::SeededRandom random(20261018U);

    for (int trial = 0; trial < 8; ++trial) {
        SCOPED_TRACE("trial " + std::to_string(trial));
        const auto world = test::makeWorldAtHz(60, false);
        ASSERT_NE(world, nullptr);
        motor::Ragdoll ragdoll =
            test::buildOrFail(*world, rig, stiff, math::Transform{}, motor::RootMotion::Kinematic);
        // Contacts between limbs and trunk are not what this test is about.
        test::disableSelfCollision(*world, ragdoll);
        std::vector<math::Quat> targets;
        targets.reserve(rig.joints.size());
        for (const rig::Joint& joint : rig.joints) {
            targets.push_back(randomWithinLimits(random, joint));
        }
        const std::vector<math::Transform> pose = poseFor(rig, ragdoll, targets);

        // The inversion is exact: the solver's own targets match the chosen ones.
        const std::vector<math::Quat> solverTargets = test::targetJointRotations(ragdoll, pose);
        for (std::size_t j = 0; j < targets.size(); ++j) {
            ASSERT_LT(math::angleBetween(solverTargets[j], targets[j]), 1e-9) << rig.joints[j].bone;
        }

        for (int step = 0; step < 300; ++step) {
            motor::driveTowardsPose(*world, ragdoll, pose);
            world->step();
        }
        for (std::size_t j = 0; j < targets.size(); ++j) {
            EXPECT_LT(math::angleBetween(test::actualJointRotation(*world, ragdoll, j), targets[j]),
                      0.02)
                << rig.joints[j].bone;
        }
    }
}

} // namespace
