#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

constexpr int kSteps = 60;

// A box dropped onto the floor; returns the box.
BodyId buildBoxDrop(PhysicsWorld& world)
{
    createFloor(world);
    return world.createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 10.0f, Vec3{0.0f, 0.0f, 2.0f})).id;
}

std::vector<BodyState> stepAndRecord(PhysicsWorld& world, BodyId body)
{
    std::vector<BodyState> states(static_cast<std::size_t>(kSteps));
    for (auto& state : states) {
        world.step();
        EXPECT_TRUE(world.readBody(body, state));
    }
    return states;
}

// The box has fallen if it is clearly below its start height of 2 m.
void expectBoxFell(PhysicsWorld& world, BodyId box)
{
    for (int i = 0; i < kSteps; ++i) {
        world.step();
    }
    BodyState state;
    ASSERT_TRUE(world.readBody(box, state));
    EXPECT_LT(state.positionM.z, 1.9f);
}

TEST(JoltSeveralWorlds, SteppingAnotherWorldDoesNotChangeResults)
{
    std::vector<BodyState> reference;
    {
        const auto alone = makeWorld();
        ASSERT_NE(alone, nullptr);
        const BodyId box = buildBoxDrop(*alone);
        ASSERT_TRUE(box.isValid());
        reference = stepAndRecord(*alone, box);
    }

    const auto world = makeWorld();
    // Test choice: the other world differs in gravity and content and is
    // stepped twice as often, so any shared state would show.
    WorldConfig otherConfig;
    otherConfig.gravityMps2 = Vec3{1.0f, 0.0f, -3.0f};
    const auto other = makeWorld(otherConfig);
    ASSERT_NE(world, nullptr);
    ASSERT_NE(other, nullptr);

    createFloor(*other);
    const BodyId otherBox =
        other->createBody(dynamicBox(Vec3{0.2f, 0.3f, 0.4f}, 2.0f, Vec3{0.0f, 0.0f, 1.0f}, Quat{}))
            .id;
    ASSERT_TRUE(otherBox.isValid());
    const BodyId box = buildBoxDrop(*world);
    ASSERT_TRUE(box.isValid());

    for (int i = 0; i < kSteps; ++i) {
        other->step();
        world->step();
        other->step();
        BodyState state;
        ASSERT_TRUE(world->readBody(box, state));
        ASSERT_TRUE(bitEqual(state, reference[static_cast<std::size_t>(i)])) << "step " << i;
    }

    // And the other world followed its own gravity, not this one's.
    BodyState otherState;
    ASSERT_TRUE(other->readBody(otherBox, otherState));
    EXPECT_GT(otherState.positionM.x, 0.0f);
}

TEST(JoltSeveralWorlds, DestroyingFirstCreatedWorldLeavesSecondUsable)
{
    auto first = makeWorld();
    const auto second = makeWorld();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    buildBoxDrop(*first);
    const BodyId box = buildBoxDrop(*second);
    first->step();
    second->step();

    first.reset();

    expectBoxFell(*second, box);
}

TEST(JoltSeveralWorlds, DestroyingLastCreatedWorldLeavesFirstUsable)
{
    const auto first = makeWorld();
    auto second = makeWorld();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    const BodyId box = buildBoxDrop(*first);
    buildBoxDrop(*second);
    first->step();
    second->step();

    second.reset();

    expectBoxFell(*first, box);
}

TEST(JoltSeveralWorlds, WorldCanBeCreatedAfterAllWorldsWereDestroyed)
{
    // Jolt's process-wide setup is reference counted; dropping to zero and
    // back up must work.
    for (int round = 0; round < 3; ++round) {
        const auto world = makeWorld();
        ASSERT_NE(world, nullptr) << "round " << round;
        expectBoxFell(*world, buildBoxDrop(*world));
    }
}

TEST(JoltSeveralWorlds, WorldCreatedWhileAnotherExistsWorksAfterOlderIsDestroyed)
{
    auto older = makeWorld();
    ASSERT_NE(older, nullptr);
    buildBoxDrop(*older);
    const auto newer = makeWorld();
    ASSERT_NE(newer, nullptr);
    older.reset();
    const auto newest = makeWorld();
    ASSERT_NE(newest, nullptr);

    const BodyId newerBox = buildBoxDrop(*newer);
    const BodyId newestBox = buildBoxDrop(*newest);
    expectBoxFell(*newer, newerBox);
    expectBoxFell(*newest, newestBox);
}

} // namespace
