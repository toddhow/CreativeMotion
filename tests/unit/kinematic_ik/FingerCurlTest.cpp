// A5c finger curl solver (docs/tasks/0009-a5-body-surfaces-fingers.md). Written
// by the implementer under the lighter A4-A6 workflow (CLAUDE.md).

#include "opas/ik/FingerCurl.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace {

using opas::ik::CapsuleShape;
using opas::ik::FingerJoint;
using opas::ik::solveFingerCurl;
using opas::math::Vec3;

// A straight finger along +X from the origin: joints at x = 0, 0.03, 0.05,
// tip at 0.07, all bending about +Y (curling the tip toward -Z).
std::vector<FingerJoint> straightFinger(double maxCurlRad)
{
    const Vec3 axis{0.0, 1.0, 0.0};
    return {FingerJoint{{0.0, 0.0, 0.0}, axis, maxCurlRad},
            FingerJoint{{0.03, 0.0, 0.0}, axis, maxCurlRad},
            FingerJoint{{0.05, 0.0, 0.0}, axis, maxCurlRad}};
}

// A thin capsule along Y below the finger at height z.
CapsuleShape barAt(double x, double z)
{
    return CapsuleShape{{x, -1.0, z}, {x, 1.0, z}, 0.005};
}

TEST(FingerCurl, ATargetTouchedAtRestGivesZero)
{
    const auto r =
        solveFingerCurl(straightFinger(1.0), CapsuleShape{{0.07, -1, 0}, {0.07, 1, 0}, 0.01});
    ASSERT_TRUE(r.valid);
    EXPECT_TRUE(r.touched);
    EXPECT_EQ(r.t, 0.0);
}

TEST(FingerCurl, TheTipStopsOnTheSurface)
{
    // At 0.3 rad per joint (t = 0.25) the tip is at (0.0576, -0.0358) in x and z:
    // 0.03 (cos 0.3, -sin 0.3) + 0.02 (cos 0.6, -sin 0.6) + 0.02 (cos 0.9, -sin 0.9).
    const auto target = barAt(0.0576, -0.0358);
    const auto r = solveFingerCurl(straightFinger(1.2), target);
    ASSERT_TRUE(r.valid);
    ASSERT_TRUE(r.touched);
    EXPECT_GT(r.t, 0.15); // touches a little before t = 0.25 (the bar has a radius)
    EXPECT_LT(r.t, 0.25);
    // The tip lies on the capsule surface (within the bisection's resolution).
    const Vec3 onAxis{target.start.x, r.tip.y, target.start.z};
    EXPECT_NEAR(opas::math::distance(r.tip, onAxis), target.radius, 1e-4);
    ASSERT_EQ(r.jointDeltas.size(), 3U);
}

TEST(FingerCurl, NoContactWithinTheLimitsCurlsToTheLimits)
{
    const auto r = solveFingerCurl(straightFinger(0.3), barAt(0.0, -1.0)); // far away
    ASSERT_TRUE(r.valid);
    EXPECT_FALSE(r.touched);
    EXPECT_EQ(r.t, 1.0);
}

TEST(FingerCurl, EachJointsDeltaIncludesTheJointsBefore)
{
    const auto r = solveFingerCurl(straightFinger(0.4), barAt(0.0, -1.0));
    ASSERT_EQ(r.jointDeltas.size(), 3U);
    // All joints bend about the same axis here, so the angles add up: 0.4, 0.8, 1.2.
    for (std::size_t i = 0; i < 3; ++i) {
        const double angle = 2.0 * std::acos(std::min(1.0, std::abs(r.jointDeltas[i].w)));
        EXPECT_NEAR(angle, 0.4 * static_cast<double>(i + 1), 1e-9);
    }
}

TEST(FingerCurl, LimitsAreRespectedAndZeroLimitsNeverMove)
{
    const auto r = solveFingerCurl(straightFinger(0.0), barAt(0.05, -0.03));
    ASSERT_TRUE(r.valid);
    EXPECT_FALSE(r.touched);
    EXPECT_NEAR(r.tip.x, 0.07, 1e-12);
    EXPECT_NEAR(r.tip.z, 0.0, 1e-12);
}

TEST(FingerCurl, BadInputIsInvalid)
{
    auto finger = straightFinger(1.0);
    EXPECT_FALSE(solveFingerCurl(std::span(finger).first(1), barAt(0.05, -0.03)).valid);
    finger[1].hingeAxis = Vec3{};
    EXPECT_FALSE(solveFingerCurl(finger, barAt(0.05, -0.03)).valid);
    finger = straightFinger(4.0); // more than pi
    EXPECT_FALSE(solveFingerCurl(finger, barAt(0.05, -0.03)).valid);
    EXPECT_FALSE(
        solveFingerCurl(straightFinger(1.0), CapsuleShape{{0, 0, 0}, {0, 1, 0}, 0.0}).valid);
}

} // namespace
