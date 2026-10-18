// A5a capsule fit (docs/tasks/0009-a5-body-surfaces-fingers.md). Written by the
// implementer under the lighter A4-A6 workflow (CLAUDE.md).

#include "opas/markers/CapsuleFit.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace {

using opas::markers::BodyMapping;
using opas::markers::CapsuleRule;
using opas::markers::fitCapsules;
using opas::markers::SurfacePoint;
using opas::math::Vec3;

// Two bones: "A" at the origin and "B" 1 m above it, one capsule from A to B.
opas::rig::Skeleton segmentSkeleton()
{
    opas::rig::Skeleton s;
    s.bones.push_back(opas::rig::Bone{"A", -1, {}});
    opas::math::Transform up;
    up.translation = Vec3{0.0, 0.0, 1.0};
    s.bones.push_back(opas::rig::Bone{"B", 0, up});
    return s;
}

BodyMapping segmentMap()
{
    BodyMapping m;
    m.capsules.push_back(CapsuleRule{"Seg", "A", "B", 0.15});
    return m;
}

// `count` points on a cylinder of `radius` around the Z axis, heights in (0, 1).
std::vector<SurfacePoint> cylinder(int count, double radius, int bone = 0)
{
    std::vector<SurfacePoint> out;
    for (int i = 0; i < count; ++i) {
        const double angle = 2.0 * std::numbers::pi * i / count;
        const double z = (i + 0.5) / count;
        out.push_back(
            SurfacePoint{Vec3{radius * std::cos(angle), radius * std::sin(angle), z}, bone});
    }
    return out;
}

TEST(CapsuleFit, ACylinderGivesItsRadius)
{
    const auto r = fitCapsules(segmentSkeleton(), segmentMap(), cylinder(200, 0.1));
    ASSERT_TRUE(r.value) << r.error;
    EXPECT_NEAR(r.value->capsules[0].radiusRatio * 1.0, 0.1, 0.001); // within 1 mm
    EXPECT_TRUE(r.fitted[0]);
    EXPECT_EQ(r.pointsUsed[0], 200U);
}

TEST(CapsuleFit, OutliersAboveThePercentileDoNotChangeTheRadius)
{
    auto points = cylinder(200, 0.1);
    for (const auto& p : cylinder(10, 0.5)) { // 10 of 210 points: under 10 %
        points.push_back(p);
    }
    const auto r = fitCapsules(segmentSkeleton(), segmentMap(), points);
    ASSERT_TRUE(r.value) << r.error;
    EXPECT_NEAR(r.value->capsules[0].radiusRatio, 0.1, 0.001);
}

TEST(CapsuleFit, TooFewPointsKeepTheRulesRatio)
{
    const auto r = fitCapsules(segmentSkeleton(), segmentMap(), cylinder(10, 0.1));
    ASSERT_TRUE(r.value) << r.error;
    EXPECT_EQ(r.value->capsules[0].radiusRatio, 0.15);
    EXPECT_FALSE(r.fitted[0]);
    EXPECT_EQ(r.pointsUsed[0], 10U);
}

TEST(CapsuleFit, OtherBonesAndPointsBeyondTheEndsAreIgnored)
{
    auto points = cylinder(200, 0.1);
    for (const auto& p : cylinder(100, 0.4, 1)) { // weighted to bone B
        points.push_back(p);
    }
    for (auto p : cylinder(100, 0.4)) { // bone A, but above the segment
        p.positionM.z += 2.0;
        points.push_back(p);
    }
    const auto r = fitCapsules(segmentSkeleton(), segmentMap(), points);
    ASSERT_TRUE(r.value) << r.error;
    EXPECT_EQ(r.pointsUsed[0], 200U);
    EXPECT_NEAR(r.value->capsules[0].radiusRatio, 0.1, 0.001);
}

TEST(CapsuleFit, BadInputIsRejectedWithAMessage)
{
    const auto skeleton = segmentSkeleton();
    opas::markers::CapsuleFitSettings bad;
    bad.percentile = 0.0;
    EXPECT_FALSE(fitCapsules(skeleton, segmentMap(), cylinder(30, 0.1), bad).value);

    BodyMapping unknown = segmentMap();
    unknown.capsules[0].endBone = "Missing";
    const auto r = fitCapsules(skeleton, unknown, cylinder(30, 0.1));
    EXPECT_FALSE(r.value);
    EXPECT_NE(r.error.find("Missing"), std::string::npos) << r.error;

    auto points = cylinder(30, 0.1);
    points[3].positionM.x = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(fitCapsules(skeleton, segmentMap(), points).value);
    points = cylinder(30, 0.1);
    points[0].bone = 7;
    EXPECT_FALSE(fitCapsules(skeleton, segmentMap(), points).value);
}

TEST(CapsuleFit, ItIsDeterministic)
{
    const auto points = cylinder(97, 0.08);
    const auto a = fitCapsules(segmentSkeleton(), segmentMap(), points);
    const auto b = fitCapsules(segmentSkeleton(), segmentMap(), points);
    ASSERT_TRUE(a.value && b.value);
    EXPECT_EQ(a.value->capsules[0].radiusRatio, b.value->capsules[0].radiusRatio);
}

} // namespace
