#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

TEST(JoltHandles, DestroyedBodyHandleIsRejectedEverywhere)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    const auto body = world->createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 1.0f, Vec3{}));
    ASSERT_TRUE(body.ok()) << body.message;

    ASSERT_TRUE(world->destroyBody(body.id));

    BodyState state;
    EXPECT_FALSE(world->readBody(body.id, state));
    EXPECT_FALSE(world->applyImpulse(body.id, Vec3{1.0f, 0.0f, 0.0f}, Vec3{}));
    EXPECT_FALSE(world->setVelocity(body.id, Vec3{1.0f, 0.0f, 0.0f}, Vec3{}));
    EXPECT_FALSE(world->setKinematicTarget(body.id, Vec3{1.0f, 0.0f, 0.0f}, Quat{}));
    EXPECT_FALSE(world->destroyBody(body.id));
}

TEST(JoltHandles, DestroyedKinematicBodyRejectsKinematicTarget)
{
    // setKinematicTarget on a dynamic body may fail for another reason, so
    // check the stale-handle case on a body where it would otherwise succeed.
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    BodyDesc desc;
    desc.motion = MotionType::Kinematic;
    const auto body = world->createBody(desc);
    ASSERT_TRUE(body.ok()) << body.message;
    ASSERT_TRUE(world->setKinematicTarget(body.id, Vec3{0.1f, 0.0f, 0.0f}, Quat{}));

    ASSERT_TRUE(world->destroyBody(body.id));
    EXPECT_FALSE(world->setKinematicTarget(body.id, Vec3{0.2f, 0.0f, 0.0f}, Quat{}));
}

TEST(JoltHandles, DefaultConstructedHandlesAreRejected)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    BodyState state;
    EXPECT_FALSE(world->readBody(BodyId{}, state));
    EXPECT_FALSE(world->destroyBody(BodyId{}));
    EXPECT_FALSE(world->setVelocity(BodyId{}, Vec3{}, Vec3{}));
    EXPECT_FALSE(world->destroyJoint(JointId{}));
    EXPECT_FALSE(world->setJointMotor(JointId{}, JointMotor{}));
    EXPECT_FALSE(world->removeStepListener(StepListenerId{}));
}

TEST(JoltHandles, DestroyingBodyDestroysAttachedJoints)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const JointedPair pair = createJointedPair(*world, degToRad(-30.0f), degToRad(30.0f));
    const auto joint = world->createJoint(pair.jointDesc);
    ASSERT_TRUE(joint.ok()) << joint.message;
    ASSERT_TRUE(world->setJointMotor(joint.id, JointMotor{}));

    ASSERT_TRUE(world->destroyBody(pair.box));

    EXPECT_FALSE(world->setJointMotor(joint.id, JointMotor{}));
    EXPECT_FALSE(world->setJointTarget(joint.id, Quat{}));
    EXPECT_FALSE(world->destroyJoint(joint.id));
    // The other body is untouched and the world still steps.
    BodyState state;
    EXPECT_TRUE(world->readBody(pair.anchor, state));
    world->step();
}

TEST(JoltHandles, DestroyedJointHandleIsRejected)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const JointedPair pair = createJointedPair(*world, degToRad(-30.0f), degToRad(30.0f));
    const auto joint = world->createJoint(pair.jointDesc);
    ASSERT_TRUE(joint.ok()) << joint.message;

    ASSERT_TRUE(world->destroyJoint(joint.id));
    EXPECT_FALSE(world->setJointMotor(joint.id, JointMotor{}));
    EXPECT_FALSE(world->setJointTarget(joint.id, Quat{}));
    EXPECT_FALSE(world->destroyJoint(joint.id));
    // Destroying a joint leaves its bodies alive.
    BodyState state;
    EXPECT_TRUE(world->readBody(pair.box, state));
}

std::pair<std::uint32_t, std::uint32_t> key(BodyId id)
{
    return {id.index, id.generation};
}

std::pair<std::uint32_t, std::uint32_t> key(JointId id)
{
    return {id.index, id.generation};
}

template <typename Id>
bool allDistinct(const std::vector<Id>& ids)
{
    std::vector<std::pair<std::uint32_t, std::uint32_t>> keys;
    keys.reserve(ids.size());
    for (const Id& id : ids) {
        keys.push_back(key(id));
    }
    std::sort(keys.begin(), keys.end());
    return std::adjacent_find(keys.begin(), keys.end()) == keys.end();
}

TEST(JoltHandles, NewBodyNeverGetsIdOfDestroyedBody)
{
    // Test choice: a tiny capacity forces the backend to reuse slots, which is
    // where a missing generation bump would show up.
    WorldConfig config;
    config.maxBodies = 2;
    const auto world = makeWorld(config);
    ASSERT_NE(world, nullptr);

    std::vector<BodyId> ids;
    for (int i = 0; i < 200; ++i) {
        const auto body = world->createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 1.0f, Vec3{}));
        ASSERT_TRUE(body.ok()) << "create " << i << ": " << body.message;
        ids.push_back(body.id);
        ASSERT_TRUE(world->destroyBody(body.id));
    }
    EXPECT_TRUE(allDistinct(ids));
}

TEST(JoltHandles, NewJointNeverGetsIdOfDestroyedJoint)
{
    WorldConfig config = zeroGravityConfig();
    config.maxJoints = 2;
    const auto world = makeWorld(config);
    ASSERT_NE(world, nullptr);
    const JointedPair pair = createJointedPair(*world, degToRad(-30.0f), degToRad(30.0f));

    std::vector<JointId> ids;
    for (int i = 0; i < 200; ++i) {
        const auto joint = world->createJoint(pair.jointDesc);
        ASSERT_TRUE(joint.ok()) << "create " << i << ": " << joint.message;
        ids.push_back(joint.id);
        ASSERT_TRUE(world->destroyJoint(joint.id));
    }
    EXPECT_TRUE(allDistinct(ids));
}

TEST(JoltHandles, ReadBodiesSkipsStaleEntriesAndCountsValidOnes)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    const BodyId first =
        world->createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 1.0f, Vec3{0.0f, 0.0f, 1.0f})).id;
    const BodyId stale =
        world->createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 1.0f, Vec3{0.0f, 0.0f, 3.0f})).id;
    const BodyId last =
        world->createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 1.0f, Vec3{0.0f, 0.0f, 5.0f})).id;
    ASSERT_TRUE(world->destroyBody(stale));

    const std::array<BodyId, 3> ids{first, stale, last};
    std::array<BodyState, 3> states{};
    // A sentinel shows whether the stale entry was written.
    states[1].positionM = Vec3{-7.0f, -7.0f, -7.0f};

    EXPECT_EQ(world->readBodies(ids, states), 2U);
    EXPECT_LE(distance(states[0].positionM, Vec3{0.0f, 0.0f, 1.0f}), 1e-6);
    EXPECT_EQ(states[1].positionM, (Vec3{-7.0f, -7.0f, -7.0f}));
    EXPECT_LE(distance(states[2].positionM, Vec3{0.0f, 0.0f, 5.0f}), 1e-6);
}

} // namespace
