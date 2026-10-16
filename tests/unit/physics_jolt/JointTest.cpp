#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

constexpr Vec3 kTwistAxis{1.0f, 0.0f, 0.0f}; // matches createJointedPair

// Twist of the box relative to the anchor about the joint's twist axis.
double readTwistDeg(const PhysicsWorld& world, const JointedPair& pair)
{
    BodyState anchorState;
    BodyState boxState;
    EXPECT_TRUE(world.readBody(pair.anchor, anchorState));
    EXPECT_TRUE(world.readBody(pair.box, boxState));
    return radToDeg(twistAngleRad(anchorState.rotation, boxState.rotation, kTwistAxis));
}

TEST(JoltJointMotor, MotorDrivesTwistToFortyFiveDegreesWithinTwoSeconds)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);

    const JointedPair pair = createJointedPair(*world, degToRad(-90.0f), degToRad(90.0f));
    ASSERT_TRUE(pair.anchor.isValid());
    ASSERT_TRUE(pair.box.isValid());
    const auto joint = world->createJoint(pair.jointDesc);
    ASSERT_TRUE(joint.ok()) << joint.message;

    JointMotor motor;
    motor.stiffnessNmPerRad = 50.0f;
    motor.dampingNmsPerRad = 5.0f;
    // maxTorqueNm keeps its default: unlimited.
    ASSERT_TRUE(world->setJointMotor(joint.id, motor));
    ASSERT_TRUE(world->setJointTarget(joint.id, axisAngle(kTwistAxis, degToRad(45.0f))));

    for (int i = 0; i < 2 * kStepsPerSecond; ++i) {
        world->step();
    }

    EXPECT_NEAR(readTwistDeg(*world, pair), 45.0, 1.0);
}

TEST(JoltJointLimits, TwistStaysWithinLimitWhenSpunWithMotorOff)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);

    const JointedPair pair = createJointedPair(*world, degToRad(-30.0f), degToRad(30.0f));
    ASSERT_TRUE(pair.anchor.isValid());
    ASSERT_TRUE(pair.box.isValid());
    const auto joint = world->createJoint(pair.jointDesc);
    ASSERT_TRUE(joint.ok()) << joint.message;

    // Both zero = motor off; set explicitly rather than rely on the default.
    ASSERT_TRUE(world->setJointMotor(joint.id, JointMotor{}));
    ASSERT_TRUE(world->setVelocity(pair.box, Vec3{0.0f, 0.0f, 0.0f}, Vec3{10.0f, 0.0f, 0.0f}));

    double maxAbsTwistDeg = 0.0;
    for (int i = 0; i < kStepsPerSecond; ++i) {
        world->step();
        const double twistDeg = readTwistDeg(*world, pair);
        EXPECT_LE(std::abs(twistDeg), 30.0 + 2.0) << "at step " << i;
        maxAbsTwistDeg = std::max(maxAbsTwistDeg, std::abs(twistDeg));
    }

    // Test choice: guards against a vacuous pass. At 10 rad/s the box covers
    // 30 degrees in about 3 steps, so it must have come close to the limit.
    EXPECT_GE(maxAbsTwistDeg, 25.0);
}

} // namespace
