#include "RagdollTestSupport.hpp"

#include <gtest/gtest.h>

// Building and destroying a ragdoll (M3, docs/tasks/0013).

namespace {

using namespace opas;
using opas::test::bodyRotation;
using opas::test::buildOrFail;
using opas::test::makeWorldAtHz;
using opas::test::sampleProfile;
using opas::test::sampleRig;

// The model root is placed away from the origin and turned, so a bug that
// ignores rootWorld shows up.
math::Transform turnedRoot()
{
    return math::Transform{math::Vec3{1.0, -2.0, 0.05},
                           math::fromAxisAngle(math::Vec3{0.0, 0.0, 1.0}, 0.7)};
}

// Can `count` bodies and groups be created? True after a ragdoll is gone in a
// world sized for exactly one ragdoll.
bool roomFor(physics::PhysicsWorld& world, std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i) {
        physics::BodyDesc desc;
        desc.positionM = physics::Vec3{static_cast<float>(i) * 3.0f, 0.0f, 50.0f};
        if (!world.createBody(desc).ok() || !world.createCollisionGroup(1).ok()) {
            return false;
        }
    }
    return true;
}

TEST(RagdollBuild, SampleRigGivesOneBodyPerRigBodyAtTheRestPose)
{
    const auto world = makeWorldAtHz(60);
    ASSERT_NE(world, nullptr);
    const rig::Rig rig = sampleRig();
    const motor::Ragdoll ragdoll = buildOrFail(*world, rig, sampleProfile(), turnedRoot());
    ASSERT_EQ(ragdoll.bodies.size(), 16U);
    ASSERT_EQ(ragdoll.joints.size(), 15U);
    EXPECT_EQ(rig.bodies[ragdoll.rootBody].bone, "Pelvis");

    const std::vector<math::Transform> boneRest = rig::computeWorldRest(rig.skeleton);
    for (std::size_t i = 0; i < ragdoll.bodies.size(); ++i) {
        const math::Transform expected =
            turnedRoot() * boneRest[ragdoll.bodies[i].bone] * rig.bodies[i].local;
        physics::BodyState state;
        ASSERT_TRUE(world->readBody(ragdoll.bodies[i].id, state));
        EXPECT_NEAR(state.positionM.x, expected.translation.x, 1e-5) << rig.bodies[i].bone;
        EXPECT_NEAR(state.positionM.y, expected.translation.y, 1e-5) << rig.bodies[i].bone;
        EXPECT_NEAR(state.positionM.z, expected.translation.z, 1e-5) << rig.bodies[i].bone;
        EXPECT_LT(math::angleBetween(bodyRotation(*world, ragdoll.bodies[i].id), expected.rotation),
                  1e-5)
            << rig.bodies[i].bone;
    }
    double totalKg = 0.0;
    for (const auto& body : rig.bodies) {
        totalKg += body.massKg;
    }
    EXPECT_NEAR(totalKg, 75.0, 1e-9);
}

TEST(RagdollBuild, EveryJointReadsIdentityAtRest)
{
    const auto world = makeWorldAtHz(60);
    ASSERT_NE(world, nullptr);
    const motor::Ragdoll ragdoll = buildOrFail(*world, sampleRig(), sampleProfile(), turnedRoot());
    for (std::size_t j = 0; j < ragdoll.joints.size(); ++j) {
        EXPECT_LT(math::angleBetween(test::actualJointRotation(*world, ragdoll, j), math::Quat{}),
                  2e-5)
            << j;
    }
}

TEST(RagdollBuild, FailedBuildLeavesNothingBehind)
{
    physics::WorldConfig config;
    config.maxBodies = 16;
    config.maxJoints = 5; // the sixth joint fails
    const auto world = physics::jolt::createJoltWorld(config);
    ASSERT_NE(world, nullptr);
    const motor::BuildResult result =
        motor::buildRagdoll(*world, sampleRig(), sampleProfile(), math::Transform{});
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error, physics::ErrorCode::CapacityExceeded);
    EXPECT_NE(result.message.find("joint on"), std::string::npos) << result.message;
    EXPECT_TRUE(roomFor(*world, 16)); // all 16 bodies and the group were freed
}

TEST(RagdollBuild, InvalidRigOrProfileIsRejectedBeforeAnythingIsCreated)
{
    const auto world = makeWorldAtHz(60);
    ASSERT_NE(world, nullptr);
    rig::Rig rig = sampleRig();
    rig.bodies[3].massKg = 0.0;
    motor::BuildResult result =
        motor::buildRagdoll(*world, rig, sampleProfile(), math::Transform{});
    EXPECT_EQ(result.error, physics::ErrorCode::InvalidDescription);
    EXPECT_EQ(result.message.rfind("rig: /bodies/3/massKg", 0), 0U) << result.message;

    rig::Profile profile = sampleProfile();
    profile.rig = "someone-else";
    result = motor::buildRagdoll(*world, sampleRig(), profile, math::Transform{});
    EXPECT_EQ(result.message.rfind("profile: /rig", 0), 0U) << result.message;
}

TEST(RagdollBuild, DestroyFreesEverything)
{
    physics::WorldConfig config;
    config.maxBodies = 16;
    config.maxJoints = 15;
    const auto world = physics::jolt::createJoltWorld(config);
    ASSERT_NE(world, nullptr);
    motor::Ragdoll ragdoll = buildOrFail(*world, sampleRig(), sampleProfile(), math::Transform{});
    EXPECT_TRUE(motor::destroyRagdoll(*world, ragdoll));
    EXPECT_TRUE(ragdoll.bodies.empty());
    EXPECT_FALSE(motor::destroyRagdoll(*world, ragdoll)); // nothing left to destroy
    EXPECT_TRUE(roomFor(*world, 16));
}

TEST(RagdollBuild, KinematicRootHoldsThePelvisWhileTheRestHangs)
{
    const auto world = makeWorldAtHz(60);
    ASSERT_NE(world, nullptr);
    const rig::Rig rig = sampleRig();
    const motor::Ragdoll ragdoll =
        buildOrFail(*world, rig, sampleProfile(), math::Transform{}, motor::RootMotion::Kinematic);
    physics::BodyState pelvisBefore;
    physics::BodyState footBefore;
    const auto foot = rig::findBody(rig, "LeftFoot").value_or(0);
    ASSERT_TRUE(world->readBody(ragdoll.bodies[ragdoll.rootBody].id, pelvisBefore));
    ASSERT_TRUE(world->readBody(ragdoll.bodies[foot].id, footBefore));
    for (int i = 0; i < 60; ++i) {
        world->step();
    }
    physics::BodyState pelvisAfter;
    physics::BodyState footAfter;
    ASSERT_TRUE(world->readBody(ragdoll.bodies[ragdoll.rootBody].id, pelvisAfter));
    ASSERT_TRUE(world->readBody(ragdoll.bodies[foot].id, footAfter));
    EXPECT_FLOAT_EQ(pelvisAfter.positionM.z, pelvisBefore.positionM.z);
    // No floor: the legs stay attached (joints), held near the rest pose by
    // the motors, so the foot drops only by the joints' give under gravity.
    EXPECT_NEAR(footAfter.positionM.z, footBefore.positionM.z, 0.05);
}

TEST(RagdollBuild, JointedNeighboursDoNotPushEachOtherApart)
{
    // Zero gravity, motors on: if neighbours collided at their shared joints
    // the ragdoll would push itself apart from the first step.
    const auto world = makeWorldAtHz(60, false);
    ASSERT_NE(world, nullptr);
    const motor::Ragdoll ragdoll =
        buildOrFail(*world, sampleRig(), sampleProfile(), math::Transform{});
    for (int i = 0; i < 60; ++i) {
        world->step();
    }
    for (const auto& body : ragdoll.bodies) {
        physics::BodyState state;
        ASSERT_TRUE(world->readBody(body.id, state));
        EXPECT_LT(test::length(state.linearVelocityMps), 0.01);
        EXPECT_LT(test::length(state.angularVelocityRadps), 0.01);
    }
}

} // namespace
