#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

TEST(JoltImpulse, ImpulseAtCentreGivesVelocityImpulseOverMass)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);

    const float massKg = 2.5f;
    BodyDesc desc = dynamicBox(Vec3{0.3f, 0.2f, 0.1f}, massKg, Vec3{1.0f, -2.0f, 3.0f});
    // Test choice: damping off and one step taken before reading, so the
    // result does not depend on whether the backend applies the impulse
    // immediately or during the next step.
    desc.linearDamping = 0.0f;
    desc.angularDamping = 0.0f;
    const auto body = world->createBody(desc);
    ASSERT_TRUE(body.ok()) << body.message;

    const Vec3 impulseNs{3.0f, -1.5f, 0.75f};
    ASSERT_TRUE(world->applyImpulse(body.id, impulseNs, desc.positionM));
    world->step();

    BodyState state;
    ASSERT_TRUE(world->readBody(body.id, state));
    const Vec3 expected{impulseNs.x / massKg, impulseNs.y / massKg, impulseNs.z / massKg};
    const double relativeError = distance(state.linearVelocityMps, expected) / length(expected);
    EXPECT_LE(relativeError, 1e-5);
}

TEST(JoltKinematic, KinematicBodyReachesTargetAfterOneStep)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    BodyDesc desc;
    desc.shape = BoxShape{Vec3{0.2f, 0.2f, 0.2f}};
    desc.motion = MotionType::Kinematic;
    desc.positionM = Vec3{0.0f, 0.0f, 1.0f};
    const auto body = world->createBody(desc);
    ASSERT_TRUE(body.ok()) << body.message;

    const Vec3 targetPositionM{0.5f, -0.25f, 1.3f};
    const Quat targetRotation = axisAngle(Vec3{0.0f, 0.6f, 0.8f}, degToRad(30.0f));
    ASSERT_TRUE(world->setKinematicTarget(body.id, targetPositionM, targetRotation));
    world->step();

    BodyState state;
    ASSERT_TRUE(world->readBody(body.id, state));
    EXPECT_LE(distance(state.positionM, targetPositionM), 1e-5);
    EXPECT_LE(angleBetween(state.rotation, targetRotation), 1e-4);
}

} // namespace
