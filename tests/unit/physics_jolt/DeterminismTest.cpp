#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

constexpr std::size_t kBodyCount = 5; // floor, three stacked boxes, falling box
constexpr std::size_t kFallingBox = 4;

// Builds the scene with a fixed sequence of calls, so two worlds built by it
// must evolve identically.
std::array<BodyId, kBodyCount> buildTiltedBoxOntoStack(PhysicsWorld& world)
{
    std::array<BodyId, kBodyCount> ids{};
    ids[0] = createFloor(world);

    // Test choice: three 1 m cubes, 5 kg each, stacked exactly touching.
    for (std::size_t i = 0; i < 3; ++i) {
        const float z = 0.5f + static_cast<float>(i);
        ids[i + 1] =
            world.createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 5.0f, Vec3{0.0f, 0.0f, z})).id;
    }

    // Test choice: tilted 0.4 rad about a diagonal axis, slightly off-centre
    // so the impact is asymmetric, and spinning at (1, 2, 3) rad/s.
    const float invSqrt2 = 0.70710678f;
    const Quat tilt = axisAngle(Vec3{invSqrt2, invSqrt2, 0.0f}, 0.4f);
    ids[kFallingBox] =
        world.createBody(dynamicBox(Vec3{0.4f, 0.3f, 0.2f}, 3.0f, Vec3{0.15f, -0.1f, 5.0f}, tilt))
            .id;
    world.setVelocity(ids[kFallingBox], Vec3{0.0f, 0.0f, 0.0f}, Vec3{1.0f, 2.0f, 3.0f});
    return ids;
}

TEST(JoltDeterminism, TwoWorldsSteppedInterleavedStayBitIdentical)
{
    const auto worldA = makeWorld();
    const auto worldB = makeWorld();
    ASSERT_NE(worldA, nullptr);
    ASSERT_NE(worldB, nullptr);

    const auto idsA = buildTiltedBoxOntoStack(*worldA);
    const auto idsB = buildTiltedBoxOntoStack(*worldB);
    for (std::size_t i = 0; i < kBodyCount; ++i) {
        ASSERT_TRUE(idsA[i].isValid());
        ASSERT_TRUE(idsB[i].isValid());
    }

    BodyState fallingBoxStart;
    ASSERT_TRUE(worldA->readBody(idsA[kFallingBox], fallingBoxStart));

    std::array<BodyState, kBodyCount> statesA{};
    std::array<BodyState, kBodyCount> statesB{};
    for (int stepIndex = 0; stepIndex < 300; ++stepIndex) {
        worldA->step();
        worldB->step();

        ASSERT_EQ(worldA->readBodies(idsA, statesA), kBodyCount);
        ASSERT_EQ(worldB->readBodies(idsB, statesB), kBodyCount);
        for (std::size_t i = 0; i < kBodyCount; ++i) {
            ASSERT_TRUE(bitEqual(statesA[i], statesB[i]))
                << "body " << i << " differs at step " << stepIndex;
        }
    }

    // Guards against a vacuous pass: the falling box must actually have moved.
    EXPECT_LT(statesA[kFallingBox].positionM.z, fallingBoxStart.positionM.z - 1.0f);
}

} // namespace
