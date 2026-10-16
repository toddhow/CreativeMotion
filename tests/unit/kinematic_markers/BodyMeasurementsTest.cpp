#include "opas/markers/BodyMeasurements.hpp"
#include "TestSupport.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"
#include "opas/rig/Skeleton.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace {

using opas::markers::BodyMapping;
using opas::markers::CapsuleRule;
using opas::markers::LimbDefinition;
using opas::markers::measureBody;
using opas::markers::MeasureResult;
using opas::math::fromAxisAngle;
using opas::math::Quat;
using opas::math::Transform;
using opas::math::Vec3;
using opas::rig::Bone;
using opas::rig::Skeleton;
using opas::test::contains;
using opas::test::kPi;
using opas::test::Vec3Near;

constexpr double kTol = 1e-12;

Bone makeBone(std::string name, int parent, Vec3 translation, Quat rotation = {})
{
    return Bone{std::move(name), parent, Transform{translation, rotation}};
}

// Synthetic humanoid, built here (no game data). The pelvis is placed at (1, 2, 1) and turned
// 90 degrees about Z, so every world position below needs the rotation Rz90: (x,y,z) -> (-y,x,z).
// All other bones are identity-rotated except the right forearm (Rx90: (x,y,z) -> (x,-z,y)).
//
// Bone                parent  local translation   hand-computed world position
//  0 Pelvis            -1     (1, 2, 1)           (1, 2, 1)
//  1 Spine              0     (0, 0, 0.2)         (1, 2, 1.2)
//  2 Chest              1     (0, 0, 0.3)         (1, 2, 1.5)
//  3 Head               2     (0, 0, 0.4)         (1, 2, 1.9)
//  4 RightUpperArm      2     (-0.2, 0, 0.1)      (1, 1.8, 1.6)
//  5 RightForearm       4     (-0.3, 0, 0)        (1, 1.5, 1.6)    local rotation Rx90
//  6 RightHand          5     (0, 0, -0.25)       (0.75, 1.5, 1.6)
//  7 LeftUpperArm       2     (0.2, 0, 0.1)       (1, 2.2, 1.6)
//  8 LeftForearm        7     (0.3, 0, 0)         (1, 2.5, 1.6)
//  9 LeftHand           8     (0.35, 0, 0)        (1, 2.85, 1.6)
// 10 RightThigh         0     (-0.1, 0, -0.05)    (1, 1.9, 0.95)
// 11 RightShin         10     (0, 0, -0.45)       (1, 1.9, 0.5)
// 12 RightFoot         11     (0, 0, -0.45)       (1, 1.9, 0.05)
// 13 LeftThigh          0     (0.1, 0, -0.05)     (1, 2.1, 0.95)
// 14 LeftShin          13     (0, 0, -0.45)       (1, 2.1, 0.5)
// 15 LeftFoot          14     (0, 0, -0.45)       (1, 2.1, 0.05)
//
// RightHand: the forearm's world rotation is Rz90*Rx90, which sends (0,0,-0.25) to
// Rx90 -> (0,0.25,0), Rz90 -> (-0.25,0,0), so the hand is at (1,1.5,1.6) + (-0.25,0,0).
Skeleton makeHumanoid()
{
    const Quat rz90 = fromAxisAngle(Vec3{0.0, 0.0, 1.0}, kPi / 2.0);
    const Quat rx90 = fromAxisAngle(Vec3{1.0, 0.0, 0.0}, kPi / 2.0);
    Skeleton s;
    s.bones = {
        makeBone("Pelvis", -1, Vec3{1.0, 2.0, 1.0}, rz90),
        makeBone("Spine", 0, Vec3{0.0, 0.0, 0.2}),
        makeBone("Chest", 1, Vec3{0.0, 0.0, 0.3}),
        makeBone("Head", 2, Vec3{0.0, 0.0, 0.4}),
        makeBone("RightUpperArm", 2, Vec3{-0.2, 0.0, 0.1}),
        makeBone("RightForearm", 4, Vec3{-0.3, 0.0, 0.0}, rx90),
        makeBone("RightHand", 5, Vec3{0.0, 0.0, -0.25}),
        makeBone("LeftUpperArm", 2, Vec3{0.2, 0.0, 0.1}),
        makeBone("LeftForearm", 7, Vec3{0.3, 0.0, 0.0}),
        makeBone("LeftHand", 8, Vec3{0.35, 0.0, 0.0}),
        makeBone("RightThigh", 0, Vec3{-0.1, 0.0, -0.05}),
        makeBone("RightShin", 10, Vec3{0.0, 0.0, -0.45}),
        makeBone("RightFoot", 11, Vec3{0.0, 0.0, -0.45}),
        makeBone("LeftThigh", 0, Vec3{0.1, 0.0, -0.05}),
        makeBone("LeftShin", 13, Vec3{0.0, 0.0, -0.45}),
        makeBone("LeftFoot", 14, Vec3{0.0, 0.0, -0.45}),
    };
    return s;
}

BodyMapping makeMapping()
{
    BodyMapping m;
    m.limbs = {
        {"RightArm", {"RightUpperArm", "RightForearm", "RightHand"}},
        {"LeftArm", {"LeftUpperArm", "LeftForearm", "LeftHand"}},
        {"RightLeg", {"RightThigh", "RightShin", "RightFoot"}},
        {"Spine", {"Pelvis", "Spine", "Chest", "Head"}},
        {"RightLowerArm", {"RightForearm", "RightHand"}},
    };
    m.capsules = {
        CapsuleRule{"RightForearm", "RightForearm", "RightHand", 0.2},
        CapsuleRule{"Torso", "Pelvis", "Chest", 0.5},
        CapsuleRule{"LeftForearm", "LeftForearm", "LeftHand", 0.1},
    };
    return m;
}

MeasureResult measureHumanoid()
{
    const Skeleton skeleton = makeHumanoid();
    EXPECT_EQ(opas::rig::validateSkeleton(skeleton), "")
        << "the test skeleton itself must be valid";
    return measureBody(skeleton, makeMapping());
}

TEST(MarkersBodyMeasurements, LimbLengthsAreSumsOfConsecutiveBoneDistances)
{
    const MeasureResult r = measureHumanoid();
    ASSERT_TRUE(r.value.has_value()) << r.error;
    EXPECT_EQ(r.error, "");
    ASSERT_EQ(r.value->limbs.size(), 5U);

    // Same order as the mapping. The right arm bends at the elbow, so its length is the sum of
    // the two segments (0.3 + 0.25), not the 0.39 m straight line from shoulder to hand.
    EXPECT_EQ(r.value->limbs[0].name, "RightArm");
    EXPECT_NEAR(r.value->limbs[0].lengthM, 0.3 + 0.25, kTol);
    EXPECT_EQ(r.value->limbs[1].name, "LeftArm");
    EXPECT_NEAR(r.value->limbs[1].lengthM, 0.3 + 0.35, kTol);
    EXPECT_EQ(r.value->limbs[2].name, "RightLeg");
    EXPECT_NEAR(r.value->limbs[2].lengthM, 0.45 + 0.45, kTol);
    EXPECT_EQ(r.value->limbs[3].name, "Spine");
    EXPECT_NEAR(r.value->limbs[3].lengthM, 0.2 + 0.3 + 0.4, kTol);
    EXPECT_EQ(r.value->limbs[4].name, "RightLowerArm");
    EXPECT_NEAR(r.value->limbs[4].lengthM, 0.25, kTol);
}

TEST(MarkersBodyMeasurements, CapsulesRunBetweenWorldRestPositionsWithRatioTimesLengthRadius)
{
    const MeasureResult r = measureHumanoid();
    ASSERT_TRUE(r.value.has_value()) << r.error;
    ASSERT_EQ(r.value->capsules.size(), 3U);

    const auto& rightForearm = r.value->capsules[0];
    EXPECT_EQ(rightForearm.region, "RightForearm");
    EXPECT_PRED_FORMAT3(Vec3Near, rightForearm.startM, (Vec3{1.0, 1.5, 1.6}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rightForearm.endM, (Vec3{0.75, 1.5, 1.6}), kTol);
    EXPECT_NEAR(rightForearm.radiusM, 0.2 * 0.25, kTol); // ratio 0.2 * length 0.25

    const auto& torso = r.value->capsules[1];
    EXPECT_EQ(torso.region, "Torso");
    EXPECT_PRED_FORMAT3(Vec3Near, torso.startM, (Vec3{1.0, 2.0, 1.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, torso.endM, (Vec3{1.0, 2.0, 1.5}), kTol);
    EXPECT_NEAR(torso.radiusM, 0.5 * 0.5, kTol); // ratio 0.5 * length 0.5

    const auto& leftForearm = r.value->capsules[2];
    EXPECT_EQ(leftForearm.region, "LeftForearm");
    EXPECT_PRED_FORMAT3(Vec3Near, leftForearm.startM, (Vec3{1.0, 2.5, 1.6}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, leftForearm.endM, (Vec3{1.0, 2.85, 1.6}), kTol);
    EXPECT_NEAR(leftForearm.radiusM, 0.1 * 0.35, kTol);
}

TEST(MarkersBodyMeasurements, HeightIsHighestMinusLowestBoneWorldZ)
{
    const MeasureResult r = measureHumanoid();
    ASSERT_TRUE(r.value.has_value()) << r.error;
    // Head z = 1.9, feet z = 0.05.
    EXPECT_NEAR(r.value->heightM, 1.9 - 0.05, kTol);
}

TEST(MarkersBodyMeasurements, EmptyMappingGivesEmptyListsButStillAHeight)
{
    const MeasureResult r = measureBody(makeHumanoid(), BodyMapping{});
    ASSERT_TRUE(r.value.has_value()) << r.error;
    EXPECT_TRUE(r.value->limbs.empty());
    EXPECT_TRUE(r.value->capsules.empty());
    EXPECT_NEAR(r.value->heightM, 1.85, kTol);
}

// ---- Rejections ------------------------------------------------------------------

void expectRejectedNaming(const BodyMapping& mapping, const std::string& name)
{
    const MeasureResult r = measureBody(makeHumanoid(), mapping);
    EXPECT_FALSE(r.value.has_value());
    EXPECT_FALSE(r.error.empty());
    EXPECT_TRUE(contains(r.error, name)) << "error \"" << r.error << "\" should name " << name;
}

TEST(MarkersBodyMeasurements, LimbNamingAMissingBoneIsRejected)
{
    BodyMapping m;
    m.limbs = {{"Tentacle", {"RightUpperArm", "NoSuchBone"}}};
    expectRejectedNaming(m, "NoSuchBone");

    // Also when the missing bone is the first or the last of a longer chain.
    m.limbs = {{"Tentacle", {"GhostRoot", "RightForearm", "RightHand"}}};
    expectRejectedNaming(m, "GhostRoot");
    m.limbs = {{"Tentacle", {"RightUpperArm", "RightForearm", "GhostTip"}}};
    expectRejectedNaming(m, "GhostTip");
}

TEST(MarkersBodyMeasurements, CapsuleNamingAMissingBoneIsRejected)
{
    BodyMapping start;
    start.capsules = {CapsuleRule{"Tail", "NoSuchStart", "RightHand", 0.1}};
    expectRejectedNaming(start, "NoSuchStart");

    BodyMapping end;
    end.capsules = {CapsuleRule{"Tail", "RightForearm", "NoSuchEnd", 0.1}};
    expectRejectedNaming(end, "NoSuchEnd");
}

TEST(MarkersBodyMeasurements, EmptyLimbIsRejected)
{
    BodyMapping m;
    m.limbs = {{"Phantom", {}}};
    expectRejectedNaming(m, "Phantom");
}

TEST(MarkersBodyMeasurements, LimbWithASingleBoneIsRejected)
{
    // BodyMeasurements.hpp: a limb "needs >= 2 bones".
    BodyMapping m;
    m.limbs = {{"Stump", {"RightHand"}}};
    expectRejectedNaming(m, "Stump");
}

TEST(MarkersBodyMeasurements, NonPositiveRadiusRatioIsRejected)
{
    for (const double ratio : {0.0, -0.1, -1.0}) {
        SCOPED_TRACE(ratio);
        BodyMapping m;
        m.capsules = {CapsuleRule{"FatArm", "RightForearm", "RightHand", ratio}};
        expectRejectedNaming(m, "FatArm");
    }
}

TEST(MarkersBodyMeasurements, NonFiniteRadiusRatioIsRejected)
{
    // Test choice: NaN is not "> 0".
    BodyMapping m;
    m.capsules = {CapsuleRule{"NanArm", "RightForearm", "RightHand",
                              std::numeric_limits<double>::quiet_NaN()}};
    expectRejectedNaming(m, "NanArm");
}

TEST(MarkersBodyMeasurements, AnInvalidSkeletonIsRejected)
{
    Skeleton s = makeHumanoid();
    s.bones[3].parent = 3; // parent must come before the bone
    const MeasureResult r = measureBody(s, makeMapping());
    EXPECT_FALSE(r.value.has_value());
    EXPECT_FALSE(r.error.empty());
}

TEST(MarkersBodyMeasurements, OneBadEntryRejectsTheWholeMappingEvenAfterGoodOnes)
{
    BodyMapping m = makeMapping();
    m.limbs.push_back({"Broken", {"RightHand", "NoSuchBone"}});
    expectRejectedNaming(m, "NoSuchBone");
}

} // namespace
