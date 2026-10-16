#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

TEST(JoltBoxDrop, CubeSettlesFlatOnFloorAfterThreeSeconds)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(createFloor(*world).isValid());

    // 1 m cube, 10 kg, centre 2.0 m above the floor top, at rest.
    const auto box =
        world->createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 10.0f, Vec3{0.0f, 0.0f, 2.0f}));
    ASSERT_TRUE(box.ok()) << box.message;

    for (int i = 0; i < 3 * kStepsPerSecond; ++i) {
        world->step();
    }

    BodyState state;
    ASSERT_TRUE(world->readBody(box.id, state));

    // Resting on the floor: centre at half height, minus up to 0.02 m of
    // penetration that Jolt allows (mPenetrationSlop).
    EXPECT_GE(state.positionM.z, 0.48f);
    EXPECT_LE(state.positionM.z, 0.50f);
    // Below Jolt's sleep threshold.
    EXPECT_LT(length(state.linearVelocityMps), 0.03);
    EXPECT_LT(radToDeg(tiltRad(state.rotation)), 1.0);
}

} // namespace
