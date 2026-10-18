// A7 automatic marker placement (docs/tasks/0011-a7-automatic-markers.md). Written
// by the implementer under the lighter workflow (CLAUDE.md).

#include "opas/markers/AutoMarkers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

namespace {

using opas::markers::autoAuthorMarkers;
using opas::markers::BodyMapping;
using opas::markers::CapsuleRule;
using opas::markers::LimbDefinition;
using opas::markers::PoseClip;
using opas::markers::snapOffsetToSurface;
using opas::math::Transform;
using opas::math::Vec3;

constexpr double kRateHz = 30.0;
constexpr int kFrames = 60; // 2 s

// Both actors: an arm (Shoulder -> Hand) and a chest capsule from ChestA to
// ChestB, 0.4 m long with radius 0.25 * 0.4 = 0.1 m.
BodyMapping map()
{
    BodyMapping m;
    m.limbs.push_back(LimbDefinition{"Arm", {"Shoulder", "Hand"}});
    m.capsules.push_back(CapsuleRule{"Chest", "ChestA", "ChestB", 0.25});
    return m;
}

Transform at(double x, double y, double z)
{
    Transform t;
    t.translation = Vec3{x, y, z};
    return t;
}

// Actor 1 stands at x = 1 (chest axis from z 1 to 1.4) with its hand far away;
// actor 0's chest is at x = 0 and its hand position per frame comes from `hand`.
template <typename HandAt>
PoseClip clip(HandAt hand)
{
    PoseClip c;
    c.sampleRateHz = kRateHz;
    for (auto& a : c.actors) {
        a.bodyMap = "humanoid";
        a.bones = {"BellyA", "BellyB", "ChestA", "ChestB", "Hand", "Shoulder"};
    }
    for (int f = 0; f < kFrames; ++f) {
        c.actors[0].frames.push_back(
            {at(0, 0, 0.8), at(0, 0, 1), at(0, 0, 1), at(0, 0, 1.4), hand(f), at(0.5, 0, 1.3)});
        // Actor 1's belly sits just below its chest (z 0.8 to 1).
        c.actors[1].frames.push_back(
            {at(1, 0, 0.8), at(1, 0, 1), at(1, 0, 1), at(1, 0, 1.4), at(5, 5, 1.2), at(5, 5, 1.3)});
    }
    return c;
}

TEST(AutoAuthor, ASteadyHandOnTheChestBecomesOneValidatedMarker)
{
    // 2 cm from the partner's chest surface for the whole clip.
    const auto r = autoAuthorMarkers(clip([](int) { return at(0.88, 0, 1.2); }), {map(), map()});
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.markers.size(), 1U);
    const auto& m = r.markers.markers[0];
    EXPECT_EQ(m.id.rfind("auto-", 0), 0U);
    EXPECT_EQ(m.sourceActor, 0);
    EXPECT_EQ(m.targetActor, 1);
    EXPECT_EQ(m.sourceBone, "Hand");
    EXPECT_EQ(m.targetRegion, "Chest");
    EXPECT_EQ(m.window.startS, 0.0); // active for the whole node (rule 2)
    ASSERT_EQ(r.report.size(), 1U);
    EXPECT_TRUE(r.report[0].kept);
}

TEST(AutoAuthor, AHandPassingByGivesNoMarker)
{
    // Sweeps through the chest region in a few frames, far from it otherwise.
    const auto r = autoAuthorMarkers(clip([](int f) { return at(0.88, -1.0 + 0.04 * f, 1.2); }),
                                     {map(), map()});
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.markers.markers.empty());
}

TEST(AutoAuthor, AContactForLessThanHalfTheClipIsDroppedAsIntermittent)
{
    // On the chest for 24 of 60 frames (0.8 s), far away otherwise.
    const auto r = autoAuthorMarkers(
        clip([](int f) { return f < 24 ? at(0.88, 0, 1.2) : at(0.3, 0, 0.2); }), {map(), map()});
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.markers.markers.empty());
    ASSERT_FALSE(r.report.empty());
    EXPECT_FALSE(r.report[0].kept);
    EXPECT_NE(r.report[0].reason.find("intermittent"), std::string::npos);
}

TEST(AutoAuthor, AContactThatStartsLateIsStillKept)
{
    // Away for the first 20 frames, then on the chest for 40 (1.33 s of 2 s):
    // validation must check it where it is in contact, not at frame 0.
    const auto r = autoAuthorMarkers(
        clip([](int f) { return f < 20 ? at(0.3, 0, 0.2) : at(0.88, 0, 1.2); }), {map(), map()});
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.markers.size(), 1U);
    EXPECT_EQ(r.markers.markers[0].window.startS, 0.0);
}

// The arm map plus a belly capsule (0.2 m long, radius 0.05) under the chest.
BodyMapping mapWithBelly()
{
    BodyMapping m = map();
    m.capsules.push_back(CapsuleRule{"Belly", "BellyA", "BellyB", 0.25});
    return m;
}

TEST(AutoAuthor, AHandNearTwoRegionsKeepsOnlyTheClosest)
{
    // 2 cm from the chest surface, about 7 cm from the belly surface: both are
    // within the auto distance, but only the chest is kept.
    const auto r = autoAuthorMarkers(clip([](int) { return at(0.88, 0, 1.02); }),
                                     {mapWithBelly(), mapWithBelly()});
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.markers.size(), 1U);
    EXPECT_EQ(r.markers.markers[0].targetRegion, "Chest");
    bool shadowed = false;
    for (const auto& entry : r.report) {
        shadowed = shadowed || entry.reason.find("shadowed") != std::string::npos;
    }
    EXPECT_TRUE(shadowed);
}

TEST(AutoAuthor, ItIsDeterministic)
{
    const auto c = clip([](int) { return at(0.88, 0, 1.2); });
    const auto a = autoAuthorMarkers(c, {map(), map()});
    const auto b = autoAuthorMarkers(c, {map(), map()});
    ASSERT_TRUE(a.valid && b.valid);
    ASSERT_EQ(a.markers.markers.size(), b.markers.markers.size());
    EXPECT_EQ(a.markers.markers[0].offsetM.x, b.markers.markers[0].offsetM.x);
}

// A ring of points on a cylinder of radius 0.1 around the Z axis, at heights
// 0.4 to 0.6: a stand-in for the skin around a bone.
std::vector<Vec3> cylinder()
{
    std::vector<Vec3> points;
    for (int h = 0; h <= 20; ++h) {
        for (int i = 0; i < 72; ++i) {
            const double angle = 2.0 * std::numbers::pi * i / 72;
            points.push_back(Vec3{0.1 * std::cos(angle), 0.1 * std::sin(angle), 0.4 + 0.01 * h});
        }
    }
    return points;
}

TEST(SurfaceSnap, AnOffsetOutsideTheBodyMovesOntoTheSurfaceMinusTheContactOffset)
{
    const auto points = cylinder();
    // 20 cm out on +X, pointing inward (-X): the surface is at x = 0.1.
    const auto r = snapOffsetToSurface(Vec3{0.2, 0, 0.5}, Vec3{-1, 0, 0}, points);
    ASSERT_TRUE(r.snapped);
    EXPECT_NEAR(r.offsetM.x, 0.1 + 0.02, 0.001); // 2 cm outside the skin
    EXPECT_NEAR(r.offsetM.y, 0.0, 0.001);
    EXPECT_NEAR(r.offsetM.z, 0.5, 0.001);
    EXPECT_NEAR(r.movedM, 0.08, 0.001);
}

TEST(SurfaceSnap, AnOffsetInsideTheBodyMovesOutToTheNearestSurface)
{
    const auto r = snapOffsetToSurface(Vec3{0.05, 0, 0.5}, Vec3{-1, 0, 0}, cylinder());
    ASSERT_TRUE(r.snapped);
    EXPECT_NEAR(r.offsetM.x, 0.1 + 0.02, 0.001); // the near side, not the far one
}

TEST(SurfaceSnap, OnlyTheFarSideOfTheBodyIsTooFarToSnapTo)
{
    // Only the back half of the cylinder (x < 0) is present: it lies 30 cm
    // along the line, beyond the 12 cm limit, so the offset stays.
    std::vector<Vec3> back;
    for (const Vec3& p : cylinder()) {
        if (p.x < 0.0) {
            back.push_back(p);
        }
    }
    const auto r = snapOffsetToSurface(Vec3{0.2, 0, 0.5}, Vec3{-1, 0, 0}, back);
    EXPECT_FALSE(r.snapped);
    EXPECT_EQ(r.offsetM.x, 0.2);
}

TEST(SurfaceSnap, NoSurfaceNearTheLineLeavesTheOffsetUnchanged)
{
    const Vec3 offset{0.2, 0, 2.0}; // far above the cylinder
    const auto r = snapOffsetToSurface(offset, Vec3{-1, 0, 0}, cylinder());
    EXPECT_FALSE(r.snapped);
    EXPECT_EQ(r.offsetM.x, offset.x);
    EXPECT_EQ(r.offsetM.z, offset.z);
    EXPECT_FALSE(snapOffsetToSurface(offset, Vec3{}, cylinder()).snapped);
}

// A torso-like region: axis along Z from 0 to 1 m at the origin, radius 0.1,
// surface points on that cylinder.
opas::markers::SurfaceRegion torso()
{
    opas::markers::SurfaceRegion r;
    r.region = "Torso";
    r.axisStartM = Vec3{0, 0, 0};
    r.axisEndM = Vec3{0, 0, 1};
    r.radiusM = 0.1;
    for (int h = 0; h <= 100; ++h) {
        for (int i = 0; i < 72; ++i) {
            const double angle = 2.0 * std::numbers::pi * i / 72;
            r.pointsM.push_back(Vec3{0.1 * std::cos(angle), 0.1 * std::sin(angle), 0.01 * h});
        }
    }
    return r;
}

TEST(ContactCorrection, AHandJustOffTheBodyIsFullyCorrectedToTheContactDepth)
{
    const std::vector<opas::markers::SurfaceRegion> regions{torso()};
    // 3 cm off the surface on +X: within 4 cm, so the hand ends 2 cm off.
    const auto c = opas::markers::contactCorrection(Vec3{0.13, 0, 0.5}, regions);
    ASSERT_TRUE(c);
    EXPECT_NEAR(c->gapM, 0.03, 0.001);
    EXPECT_EQ(c->weight, 1.0);
    EXPECT_NEAR(c->targetM.x, 0.12, 0.001);
    EXPECT_NEAR(c->targetM.y, 0.0, 1e-9); // slides along the body are kept
    EXPECT_NEAR(c->targetM.z, 0.5, 1e-9);
}

TEST(ContactCorrection, AHandInsideTheBodyIsPushedOut)
{
    const std::vector<opas::markers::SurfaceRegion> regions{torso()};
    const auto c = opas::markers::contactCorrection(Vec3{0.08, 0, 0.5}, regions);
    ASSERT_TRUE(c);
    EXPECT_LT(c->gapM, 0.0);
    EXPECT_EQ(c->weight, 1.0);
    EXPECT_NEAR(c->targetM.x, 0.12, 0.001);
}

TEST(ContactCorrection, TheCorrectionFadesBetweenFourAndTenCentimetres)
{
    const std::vector<opas::markers::SurfaceRegion> regions{torso()};
    const auto c = opas::markers::contactCorrection(Vec3{0.17, 0, 0.5}, regions); // 7 cm off
    ASSERT_TRUE(c);
    EXPECT_GT(c->weight, 0.0);
    EXPECT_LT(c->weight, 1.0);
    EXPECT_GT(c->targetM.x, 0.12); // moved part of the way only
    EXPECT_LT(c->targetM.x, 0.17);
    EXPECT_FALSE(opas::markers::contactCorrection(Vec3{0.25, 0, 0.5}, regions)); // 15 cm off
}

TEST(ContactCorrection, TheNearestRegionWins)
{
    auto arm = torso();
    arm.region = "Arm";
    for (auto& p : arm.pointsM) {
        p.x += 0.5; // a second cylinder 50 cm away along X
    }
    arm.axisStartM.x += 0.5;
    arm.axisEndM.x += 0.5;
    const std::vector<opas::markers::SurfaceRegion> regions{torso(), arm};
    const auto c = opas::markers::contactCorrection(Vec3{0.37, 0, 0.5}, regions); // 3 cm off Arm
    ASSERT_TRUE(c);
    EXPECT_EQ(c->region, 1U);
    EXPECT_FALSE(opas::markers::contactCorrection(Vec3{0.13, 0, 0.5}, {}));
}

} // namespace