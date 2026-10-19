#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

// Collision groups (M3, docs/tasks/0013): bodies in one group skip
// collisions between sub-groups that were set not to collide.

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

// Box A slides at 2 m/s towards box B, which rests 2 m away, without gravity.
// After 1.5 s A has passed through B if they do not collide (A ends near
// x = 2, B untouched), or has hit it (A stopped short, B pushed along).
struct Approach {
    BodyId a;
    BodyId b;
};

Approach slideIntoEachOther(PhysicsWorld& world, CollisionGroupId groupA, std::uint32_t subA,
                            CollisionGroupId groupB, std::uint32_t subB)
{
    BodyDesc descA = dynamicBox(Vec3{0.25f, 0.25f, 0.25f}, 1.0f, Vec3{-1.0f, 0.0f, 0.0f});
    descA.collisionGroup = groupA;
    descA.subGroup = subA;
    BodyDesc descB = dynamicBox(Vec3{0.25f, 0.25f, 0.25f}, 1.0f, Vec3{1.0f, 0.0f, 0.0f});
    descB.collisionGroup = groupB;
    descB.subGroup = subB;
    const auto a = world.createBody(descA);
    const auto b = world.createBody(descB);
    EXPECT_TRUE(a.ok()) << a.message;
    EXPECT_TRUE(b.ok()) << b.message;
    EXPECT_TRUE(world.setVelocity(a.id, Vec3{2.0f, 0.0f, 0.0f}, Vec3{}));
    for (int i = 0; i < 90; ++i) {
        world.step();
    }
    return {a.id, b.id};
}

bool passedThrough(const PhysicsWorld& world, const Approach& approach)
{
    BodyState a;
    BodyState b;
    EXPECT_TRUE(world.readBody(approach.a, a));
    EXPECT_TRUE(world.readBody(approach.b, b));
    return a.positionM.x > 1.5f && std::abs(b.positionM.x - 1.0f) < 1e-4f;
}

TEST(JoltCollisionGroups, DisabledSubGroupsPassThroughEachOther)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const auto group = world->createCollisionGroup(2);
    ASSERT_TRUE(group.ok()) << group.message;
    ASSERT_TRUE(world->setSubGroupsCollide(group.id, 1, 0, false)); // either order
    EXPECT_TRUE(passedThrough(*world, slideIntoEachOther(*world, group.id, 0, group.id, 1)));
}

TEST(JoltCollisionGroups, ReEnabledSubGroupsCollide)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const auto group = world->createCollisionGroup(2);
    ASSERT_TRUE(group.ok()) << group.message;
    ASSERT_TRUE(world->setSubGroupsCollide(group.id, 0, 1, false));
    ASSERT_TRUE(world->setSubGroupsCollide(group.id, 0, 1, true));
    EXPECT_FALSE(passedThrough(*world, slideIntoEachOther(*world, group.id, 0, group.id, 1)));
}

TEST(JoltCollisionGroups, NewGroupCollidesAcrossSubGroups)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const auto group = world->createCollisionGroup(2);
    ASSERT_TRUE(group.ok()) << group.message;
    EXPECT_FALSE(passedThrough(*world, slideIntoEachOther(*world, group.id, 0, group.id, 1)));
}

TEST(JoltCollisionGroups, DifferentGroupsOrNoGroupCollide)
{
    {
        const auto world = makeWorld(zeroGravityConfig());
        ASSERT_NE(world, nullptr);
        const auto g1 = world->createCollisionGroup(2);
        const auto g2 = world->createCollisionGroup(2);
        ASSERT_TRUE(g1.ok() && g2.ok());
        ASSERT_TRUE(world->setSubGroupsCollide(g1.id, 0, 1, false));
        ASSERT_TRUE(world->setSubGroupsCollide(g2.id, 0, 1, false));
        EXPECT_FALSE(passedThrough(*world, slideIntoEachOther(*world, g1.id, 0, g2.id, 1)));
    }
    {
        const auto world = makeWorld(zeroGravityConfig());
        ASSERT_NE(world, nullptr);
        const auto g = world->createCollisionGroup(2);
        ASSERT_TRUE(g.ok());
        ASSERT_TRUE(world->setSubGroupsCollide(g.id, 0, 1, false));
        EXPECT_FALSE(passedThrough(*world, slideIntoEachOther(*world, g.id, 0, {}, 0)));
    }
}

TEST(JoltCollisionGroups, BodiesKeepSettingsAfterTheGroupIsDestroyed)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const auto group = world->createCollisionGroup(2);
    ASSERT_TRUE(group.ok());
    ASSERT_TRUE(world->setSubGroupsCollide(group.id, 0, 1, false));
    BodyDesc descA = dynamicBox(Vec3{0.25f, 0.25f, 0.25f}, 1.0f, Vec3{-1.0f, 0.0f, 0.0f});
    descA.collisionGroup = group.id;
    descA.subGroup = 0;
    BodyDesc descB = dynamicBox(Vec3{0.25f, 0.25f, 0.25f}, 1.0f, Vec3{1.0f, 0.0f, 0.0f});
    descB.collisionGroup = group.id;
    descB.subGroup = 1;
    const auto a = world->createBody(descA);
    const auto b = world->createBody(descB);
    ASSERT_TRUE(a.ok() && b.ok());
    ASSERT_TRUE(world->destroyCollisionGroup(group.id));
    // A new group may reuse the slot; it must not capture the old bodies.
    const auto reused = world->createCollisionGroup(2);
    ASSERT_TRUE(reused.ok());
    ASSERT_TRUE(world->setVelocity(a.id, Vec3{2.0f, 0.0f, 0.0f}, Vec3{}));
    for (int i = 0; i < 90; ++i) {
        world->step();
    }
    EXPECT_TRUE(passedThrough(*world, Approach{a.id, b.id}));
}

TEST(JoltCollisionGroups, RejectsBadArguments)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    EXPECT_EQ(world->createCollisionGroup(0).error, ErrorCode::InvalidDescription);
    // More sub-groups than bodies, or so many that Jolt's bit table would
    // overflow (N * (N - 1) wraps 32 bits at 70000), are rejected.
    EXPECT_EQ(world->createCollisionGroup(1025).error, ErrorCode::InvalidDescription);
    {
        WorldConfig big;
        big.maxBodies = 100000;
        const auto bigWorld = makeWorld(big);
        ASSERT_NE(bigWorld, nullptr);
        EXPECT_EQ(bigWorld->createCollisionGroup(70000).error, ErrorCode::InvalidDescription);
        // (65536 itself would be accepted, but its table is 268 MB.)
        EXPECT_EQ(bigWorld->createCollisionGroup(65537).error, ErrorCode::InvalidDescription);
    }
    const auto group = world->createCollisionGroup(2);
    ASSERT_TRUE(group.ok());
    EXPECT_FALSE(world->setSubGroupsCollide(group.id, 0, 0, false));
    EXPECT_FALSE(world->setSubGroupsCollide(group.id, 0, 2, false));
    EXPECT_FALSE(world->setSubGroupsCollide(CollisionGroupId{}, 0, 1, false));

    BodyDesc desc = dynamicBox(Vec3{0.25f, 0.25f, 0.25f}, 1.0f, Vec3{});
    desc.collisionGroup = group.id;
    desc.subGroup = 2;
    EXPECT_EQ(world->createBody(desc).error, ErrorCode::InvalidDescription);

    ASSERT_TRUE(world->destroyCollisionGroup(group.id));
    EXPECT_FALSE(world->destroyCollisionGroup(group.id));
    EXPECT_FALSE(world->setSubGroupsCollide(group.id, 0, 1, false));
    desc.subGroup = 0;
    EXPECT_EQ(world->createBody(desc).error, ErrorCode::InvalidHandle);
}

class GroupChangesDuringStep final : public StepListener {
public:
    explicit GroupChangesDuringStep(CollisionGroupId group) : group_(group) {}
    void onPreStep(PhysicsWorld& world, float /*dtSeconds*/) override
    {
        createError = world.createCollisionGroup(2).error;
        setResult = world.setSubGroupsCollide(group_, 0, 1, false);
        destroyResult = world.destroyCollisionGroup(group_);
    }
    void onPostStep(PhysicsWorld& /*world*/, float /*dtSeconds*/) override {}

    ErrorCode createError = ErrorCode::None;
    bool setResult = true;
    bool destroyResult = true;

private:
    CollisionGroupId group_;
};

TEST(JoltCollisionGroups, StructuralChangesDuringAStepFail)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const auto group = world->createCollisionGroup(2);
    ASSERT_TRUE(group.ok());
    GroupChangesDuringStep listener(group.id);
    const StepListenerId id = world->addStepListener(listener);
    world->step();
    EXPECT_TRUE(world->removeStepListener(id));
    EXPECT_EQ(listener.createError, ErrorCode::CalledDuringStep);
    EXPECT_FALSE(listener.setResult);
    EXPECT_FALSE(listener.destroyResult);
    EXPECT_TRUE(world->destroyCollisionGroup(group.id)); // still alive after the step
}

} // namespace
