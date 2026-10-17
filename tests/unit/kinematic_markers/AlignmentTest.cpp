#include "opas/markers/Alignment.hpp"
#include "AlignmentOracle.hpp"
#include "AlignmentScenes.hpp"
#include "SyntheticHumanoid.hpp"
#include "TestSupport.hpp"
#include "opas/markers/InteractionMarker.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace {

using opas::markers::AlignmentResult;
using opas::markers::AlignmentSettings;
using opas::markers::InteractionMarker;
using opas::markers::LimbCorrection;
using opas::markers::TimeWindow;
using opas::math::length;
using opas::math::Vec3;
using opas::test::alignScene;
using opas::test::bonePosition;
using opas::test::faceToFace;
using opas::test::findMarker;
using opas::test::makeMarkerAt;
using opas::test::markerTargetPoint;
using opas::test::Scene;
using opas::test::shiftedScene;
using opas::test::Vec3Near;

constexpr double kTol = 1e-12;

// ---- Helpers for root-only cases -------------------------------------------------------
//
// A marker whose source bone is the chest has no IK chain, so the solver can only close it by
// moving the roots. That makes the expected numbers pure arithmetic.

Scene rootScene()
{
    return faceToFace(1.0, 1.0, 1.0);
}

// A marker on `source`'s chest whose target is `error` away (horizontally and/or vertically)
// from where the chest is now. Active between startS and endS.
InteractionMarker chestMarker(const Scene& scene, const std::string& id, int source, Vec3 error,
                              double startS = 0.0, double endS = 10.0)
{
    const Vec3 start = bonePosition(scene.actors[static_cast<std::size_t>(source)], "Chest");
    InteractionMarker m =
        makeMarkerAt(scene, id, source, "Chest", 1 - source, "Torso", start + error);
    m.window = TimeWindow{startS, endS};
    return m;
}

// The distance between a marker's source chest and its target point after the roots have moved
// by the reported shifts, computed here from the scene (not taken from the result).
double chestDistanceAfterShift(const Scene& scene, const AlignmentResult& result,
                               const std::string& markerId)
{
    const Scene shifted = shiftedScene(scene, result);
    const InteractionMarker* m = findMarker(scene, markerId);
    if (m == nullptr) {
        return -1.0;
    }
    return length(bonePosition(shifted.actors[static_cast<std::size_t>(m->sourceActor)], "Chest") -
                  markerTargetPoint(shifted, *m));
}

void expectShifts(const AlignmentResult& r, Vec3 shift0, Vec3 shift1)
{
    EXPECT_PRED_FORMAT3(Vec3Near, r.rootShiftM[0], shift0, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, r.rootShiftM[1], shift1, kTol);
}

// ---- Root alignment: hand-computed -------------------------------------------------------

TEST(AlignmentRoot, EqualWeightsSplitAHorizontalErrorHalfAndHalf)
{
    Scene s = rootScene();
    const Vec3 error{0.12, 0.16, 0.0}; // 0.2 m long
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.error.empty());
    EXPECT_FALSE(r.rootShiftClamped);
    EXPECT_TRUE(r.limbs.empty());
    expectShifts(r, error * 0.5, -error * 0.5);

    // The error is fully closed (the whole relative shift is the error).
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_EQ(r.markers[0].id, "m");
    EXPECT_NEAR(r.markers[0].distanceM, 0.0, 1e-9);
    EXPECT_TRUE(r.markers[0].met);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 0.0, 1e-9);
}

TEST(AlignmentRoot, WeightsOneToZeroMoveOnlyActorZero)
{
    Scene s = rootScene();
    s.actors[0].weight = 1.0;
    s.actors[1].weight = 0.0;
    const Vec3 error{0.12, 0.16, 0.0};
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, error, Vec3{});
    EXPECT_EQ(r.rootShiftM[1].x, 0.0);
    EXPECT_EQ(r.rootShiftM[1].y, 0.0);
    EXPECT_EQ(r.rootShiftM[1].z, 0.0);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 0.0, 1e-9);
    EXPECT_TRUE(r.markers[0].met);
}

TEST(AlignmentRoot, WeightsZeroToOneMoveOnlyActorOne)
{
    Scene s = rootScene();
    s.actors[0].weight = 0.0;
    s.actors[1].weight = 1.0;
    const Vec3 error{0.12, 0.16, 0.0};
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    // Actor 1 moves by -error, so the relative shift (actor 0 minus actor 1) is +error.
    expectShifts(r, Vec3{}, -error);
    EXPECT_EQ(r.rootShiftM[0].x, 0.0);
    EXPECT_EQ(r.rootShiftM[0].y, 0.0);
    EXPECT_EQ(r.rootShiftM[0].z, 0.0);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 0.0, 1e-9);
}

TEST(AlignmentRoot, UnequalWeightsSplitInProportionToTheWeights)
{
    Scene s = rootScene();
    s.actors[0].weight = 3.0;
    s.actors[1].weight = 1.0;
    const Vec3 error{0.12, 0.16, 0.0};
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, error * 0.75, -error * 0.25);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 0.0, 1e-9);
}

TEST(AlignmentRoot, MarkerSourcedByActorOneMovesThatActorTowardsTheTarget)
{
    // Actor 1's chest has to move by `error` to meet the target: it takes +error/2, actor 0
    // takes -error/2 (equal weights).
    Scene s = rootScene();
    const Vec3 error{-0.06, 0.08, 0.0};
    s.markers.markers.push_back(chestMarker(s, "m", 1, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, -error * 0.5, error * 0.5);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 0.0, 1e-9);
    EXPECT_NEAR(r.markers[0].distanceM, 0.0, 1e-9);

    // With actor 0 pinned (weight 0), actor 1 takes all of it: +error.
    s.actors[0].weight = 0.0;
    const AlignmentResult pinned = alignScene(s);
    ASSERT_TRUE(pinned.valid) << pinned.error;
    expectShifts(pinned, Vec3{}, error);
}

TEST(AlignmentRoot, VerticalErrorIsIgnored)
{
    Scene s = rootScene();
    const Vec3 error{0.10, 0.0, 0.30};
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    // Only the horizontal part moves the roots, and Z of both shifts is exactly 0.
    expectShifts(r, Vec3{0.05, 0.0, 0.0}, Vec3{-0.05, 0.0, 0.0});
    EXPECT_EQ(r.rootShiftM[0].z, 0.0);
    EXPECT_EQ(r.rootShiftM[1].z, 0.0);
    // The vertical 0.30 m stays: the marker is not met, and a chest has no chain to bend.
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_NEAR(r.markers[0].distanceM, 0.30, 1e-9);
    EXPECT_FALSE(r.markers[0].met);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 0.30, 1e-9);
}

TEST(AlignmentRoot, ShiftBeyondTheLimitIsClampedAndReported)
{
    Scene s = rootScene();
    const Vec3 error{0.9, 1.2, 0.0}; // 1.5 m; the default limit is 0.5 m
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    AlignmentSettings settings;
    ASSERT_EQ(settings.maxRootShiftM, 0.5);
    const AlignmentResult r = alignScene(s, settings);
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.rootShiftClamped);
    // The relative shift keeps its direction (0.6, 0.8) and has length 0.5: (0.3, 0.4).
    expectShifts(r, Vec3{0.15, 0.2, 0.0}, Vec3{-0.15, -0.2, 0.0});
    // 1.5 m of error minus the 0.5 m shift leaves 1.0 m.
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_NEAR(r.markers[0].distanceM, 1.0, 1e-9);
    EXPECT_FALSE(r.markers[0].met);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 1.0, 1e-9);
}

TEST(AlignmentRoot, ClampUsesTheSettingsLimitAndThenSplitsByWeight)
{
    Scene s = rootScene();
    s.actors[1].weight = 0.0;
    const Vec3 error{0.12, 0.16, 0.0}; // 0.2 m
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    AlignmentSettings settings;
    settings.maxRootShiftM = 0.1;
    const AlignmentResult r = alignScene(s, settings);
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.rootShiftClamped);
    expectShifts(r, Vec3{0.06, 0.08, 0.0}, Vec3{}); // length 0.1, direction (0.6, 0.8)
    EXPECT_NEAR(r.markers[0].distanceM, 0.1, 1e-9);
}

TEST(AlignmentRoot, ShiftBelowTheLimitIsNotClamped)
{
    Scene s = rootScene();
    const Vec3 error{0.294, 0.392, 0.0}; // 0.49 m, just under the 0.5 m default limit
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_FALSE(r.rootShiftClamped);
    expectShifts(r, error * 0.5, -error * 0.5);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "m"), 0.0, 1e-9);
}

TEST(AlignmentRoot, SeveralMarkersGiveTheMeanOfTheirErrors)
{
    Scene s = rootScene();
    s.markers.markers.push_back(chestMarker(s, "x", 0, Vec3{0.2, 0.0, 0.0}));
    s.markers.markers.push_back(chestMarker(s, "y", 0, Vec3{0.0, 0.2, 0.0}));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    // Mean error (0.1, 0.1); each actor takes half.
    expectShifts(r, Vec3{0.05, 0.05, 0.0}, Vec3{-0.05, -0.05, 0.0});
    ASSERT_EQ(r.markers.size(), 2U);
    // The remaining distances are those of the leftovers (0.1, -0.1) and (-0.1, 0.1).
    EXPECT_NEAR(r.markers[0].distanceM, std::sqrt(0.02), 1e-9);
    EXPECT_NEAR(r.markers[1].distanceM, std::sqrt(0.02), 1e-9);
}

TEST(AlignmentRoot, MarkersFromBothActorsAgreeingOnTheSameRelativeShiftAdd)
{
    // Actor 0 wants to move +0.2 in X to meet its target. Actor 1's marker has its target 0.2
    // m in -X of actor 1's chest, i.e. actor 1 wants to move -0.2: the same relative shift.
    Scene s = rootScene();
    s.markers.markers.push_back(chestMarker(s, "a0", 0, Vec3{0.2, 0.0, 0.0}));
    s.markers.markers.push_back(chestMarker(s, "a1", 1, Vec3{-0.2, 0.0, 0.0}));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, Vec3{0.1, 0.0, 0.0}, Vec3{-0.1, 0.0, 0.0});
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "a0"), 0.0, 1e-9);
    EXPECT_NEAR(chestDistanceAfterShift(s, r, "a1"), 0.0, 1e-9);
}

TEST(AlignmentRoot, MarkersFromBothActorsThatDisagreeCancel)
{
    // Both targets are 0.2 m in +X of their own chests: actor 0 wants +0.2, actor 1 wants +0.2,
    // so as a relative shift they ask for +0.2 and -0.2, and the mean is zero.
    Scene s = rootScene();
    s.markers.markers.push_back(chestMarker(s, "a0", 0, Vec3{0.2, 0.0, 0.0}));
    s.markers.markers.push_back(chestMarker(s, "a1", 1, Vec3{0.2, 0.0, 0.0}));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, Vec3{}, Vec3{});
    ASSERT_EQ(r.markers.size(), 2U);
    EXPECT_NEAR(r.markers[0].distanceM, 0.2, 1e-9);
    EXPECT_NEAR(r.markers[1].distanceM, 0.2, 1e-9);
}

TEST(AlignmentRoot, NoActiveMarkersMeansNoShift)
{
    Scene s = rootScene();
    s.markers.markers.push_back(chestMarker(s, "later", 0, Vec3{0.3, 0.0, 0.0}, 5.0, 6.0));
    const AlignmentResult r = alignScene(s); // clip time 1.0
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, Vec3{}, Vec3{});
    EXPECT_FALSE(r.rootShiftClamped);
    EXPECT_TRUE(r.markers.empty());
    EXPECT_TRUE(r.limbs.empty());

    const Scene empty = rootScene();
    const AlignmentResult none = alignScene(empty);
    ASSERT_TRUE(none.valid) << none.error;
    expectShifts(none, Vec3{}, Vec3{});
    EXPECT_TRUE(none.markers.empty());
}

TEST(AlignmentRoot, ShiftsAreWorldSpaceEvenForAnActorTurnedAround)
{
    // Actor 1 faces -Y (turned half a turn). Its shift is still expressed in world axes.
    Scene s = rootScene();
    const Vec3 error{0.04, -0.08, 0.0};
    s.markers.markers.push_back(chestMarker(s, "m", 0, error));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, error * 0.5, -error * 0.5);
}

// ---- Windows --------------------------------------------------------------------------------

std::vector<std::string> activeIds(const Scene& scene, double clipTimeS)
{
    Scene copy = scene;
    copy.clipTimeS = clipTimeS;
    const AlignmentResult r = alignScene(copy);
    EXPECT_TRUE(r.valid) << r.error;
    std::vector<std::string> ids;
    ids.reserve(r.markers.size());
    for (const auto& m : r.markers) {
        ids.push_back(m.id);
    }
    return ids;
}

Scene windowScene()
{
    Scene s = rootScene();
    s.markers.markers.push_back(chestMarker(s, "early", 0, Vec3{0.1, 0.0, 0.0}, 0.0, 0.5));
    s.markers.markers.push_back(chestMarker(s, "first", 0, Vec3{0.1, 0.0, 0.0}, 1.0, 2.0));
    s.markers.markers.push_back(chestMarker(s, "second", 0, Vec3{0.1, 0.0, 0.0}, 2.0, 3.0));
    s.markers.markers.push_back(chestMarker(s, "instant", 0, Vec3{0.1, 0.0, 0.0}, 4.0, 4.0));
    return s;
}

TEST(AlignmentWindows, MarkersOutsideTheClipTimeAreIgnored)
{
    const Scene s = windowScene();
    EXPECT_EQ(activeIds(s, 1.5), std::vector<std::string>({"first"}));
    EXPECT_EQ(activeIds(s, 2.5), std::vector<std::string>({"second"}));
    EXPECT_EQ(activeIds(s, 0.25), std::vector<std::string>({"early"}));
    EXPECT_TRUE(activeIds(s, 0.75).empty());
    EXPECT_TRUE(activeIds(s, 3.5).empty());
    EXPECT_TRUE(activeIds(s, 100.0).empty());
}

TEST(AlignmentWindows, StartAndEndAreInclusive)
{
    const Scene s = windowScene();
    EXPECT_EQ(activeIds(s, 0.0), std::vector<std::string>({"early"}));   // exactly startS = 0
    EXPECT_EQ(activeIds(s, 0.5), std::vector<std::string>({"early"}));   // exactly endS
    EXPECT_EQ(activeIds(s, 1.0), std::vector<std::string>({"first"}));   // exactly startS
    EXPECT_EQ(activeIds(s, 3.0), std::vector<std::string>({"second"}));  // exactly endS
    EXPECT_EQ(activeIds(s, 4.0), std::vector<std::string>({"instant"})); // start == end
    // Where one window ends and the next starts, both are active, in file order.
    EXPECT_EQ(activeIds(s, 2.0), std::vector<std::string>({"first", "second"}));
}

TEST(AlignmentWindows, ActiveMarkersKeepTheMarkerSetOrder)
{
    Scene s = rootScene();
    s.markers.markers.push_back(chestMarker(s, "zeta", 0, Vec3{0.1, 0.0, 0.0}));
    s.markers.markers.push_back(chestMarker(s, "alpha", 0, Vec3{0.1, 0.0, 0.0}));
    s.markers.markers.push_back(chestMarker(s, "mid", 1, Vec3{0.1, 0.0, 0.0}));
    EXPECT_EQ(activeIds(s, 1.0), std::vector<std::string>({"zeta", "alpha", "mid"}));
}

TEST(AlignmentWindows, InactiveMarkersDoNotInfluenceTheShift)
{
    Scene s = rootScene();
    s.markers.markers.push_back(chestMarker(s, "near", 0, Vec3{0.1, 0.0, 0.0}, 0.0, 2.0));
    s.markers.markers.push_back(chestMarker(s, "far", 0, Vec3{0.0, 0.4, 0.0}, 5.0, 6.0));
    const AlignmentResult r = alignScene(s); // clip time 1.0: only "near"
    ASSERT_TRUE(r.valid) << r.error;
    expectShifts(r, Vec3{0.05, 0.0, 0.0}, Vec3{-0.05, 0.0, 0.0});
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_EQ(r.markers[0].id, "near");

    // At 5.5 s only "far" counts.
    s.clipTimeS = 5.5;
    const AlignmentResult later = alignScene(s);
    ASSERT_TRUE(later.valid) << later.error;
    expectShifts(later, Vec3{0.0, 0.2, 0.0}, Vec3{0.0, -0.2, 0.0});
}

// ---- Limb IK: which marker drives a chain ----------------------------------------------------

// Actor 0's right hand has two markers on the other actor's left shoulder, a few centimeters
// apart. `shoulderId` names the one at the original contact; `otherId` the second one.
Scene twoMarkerScene(const std::string& shoulderId, int shoulderPriority,
                     const std::string& otherId, int otherPriority, bool otherFirst)
{
    Scene s = opas::test::makeShoulderScene(1.0, 1.0);
    InteractionMarker shoulder = s.markers.markers.front();
    shoulder.id = shoulderId;
    shoulder.priority = shoulderPriority;

    // A second contact on the same shoulder capsule, nearer the neck, approached the same way.
    const opas::test::SceneActor& target = s.actors[1];
    const Vec3 point =
        opas::test::capsuleAxisPoint(target, "LeftShoulder", 0.35) +
        Vec3{0.0, -opas::test::kShoulderApproachFront, opas::test::kShoulderApproachUp} *
            opas::test::capsuleRadiusM(target, "LeftShoulder");
    InteractionMarker other =
        makeMarkerAt(s, otherId, 0, "RightHand", 1, "LeftShoulder", point, otherPriority);

    s.markers.markers.clear();
    if (otherFirst) {
        s.markers.markers = {other, shoulder};
    } else {
        s.markers.markers = {shoulder, other};
    }
    return s;
}

void expectDrivenBy(const Scene& s, const std::string& winnerId, const std::string& loserId)
{
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 1U);
    EXPECT_EQ(r.limbs[0].markerId, winnerId);
    EXPECT_EQ(r.limbs[0].actor, 0);
    EXPECT_EQ(r.limbs[0].chain, "RightArm");
    ASSERT_EQ(r.markers.size(), 2U);
    for (const auto& m : r.markers) {
        if (m.id == winnerId) {
            EXPECT_TRUE(m.met) << m.distanceM;
            EXPECT_LE(m.distanceM, 0.02);
        } else {
            EXPECT_EQ(m.id, loserId);
            EXPECT_FALSE(m.met) << m.distanceM; // a few centimeters from where the hand went
        }
    }
    // The hand is at the winner's target, checked from the reported rotations.
    const Scene shifted = shiftedScene(s, r);
    const auto* winner = findMarker(s, winnerId);
    ASSERT_NE(winner, nullptr);
    const Vec3 hand = opas::test::solvedLimb(shifted, r.limbs[0]).end;
    EXPECT_LE(length(hand - markerTargetPoint(shifted, *winner)), 0.02);
}

TEST(AlignmentPriorities, TheHigherPriorityMarkerDrivesTheChain)
{
    for (const bool otherFirst : {false, true}) {
        SCOPED_TRACE(otherFirst ? "other marker first in the file" : "shoulder marker first");
        expectDrivenBy(twoMarkerScene("idShoulder", 5, "idOther", 1, otherFirst), "idShoulder",
                       "idOther");
        expectDrivenBy(twoMarkerScene("idShoulder", 1, "idOther", 5, otherFirst), "idOther",
                       "idShoulder");
    }
}

TEST(AlignmentPriorities, NegativePrioritiesCompareLikeAnyOtherNumber)
{
    expectDrivenBy(twoMarkerScene("idShoulder", -5, "idOther", -1, false), "idOther", "idShoulder");
    expectDrivenBy(twoMarkerScene("idShoulder", 0, "idOther", -1, true), "idShoulder", "idOther");
}

TEST(AlignmentPriorities, OnEqualPriorityTheSmallerIdWins)
{
    for (const bool otherFirst : {false, true}) {
        SCOPED_TRACE(otherFirst ? "other marker first in the file" : "shoulder marker first");
        // Smaller id on the shoulder marker, then on the other marker; file order must not matter.
        expectDrivenBy(twoMarkerScene("idA", 3, "idB", 3, otherFirst), "idA", "idB");
        expectDrivenBy(twoMarkerScene("idB", 3, "idA", 3, otherFirst), "idA", "idB");
        expectDrivenBy(twoMarkerScene("idA", 0, "idB", 0, otherFirst), "idA", "idB");
    }
}

TEST(AlignmentPriorities, PriorityBeatsIdOrder)
{
    expectDrivenBy(twoMarkerScene("idA", 1, "idB", 2, false), "idB", "idA");
}

// ---- Output structure -------------------------------------------------------------------------

TEST(AlignmentOutput, LimbCorrectionNamesActorChainAndDrivingMarker)
{
    const Scene s = opas::test::makeShoulderScene(1.0, 1.0);
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 1U);
    const LimbCorrection& limb = r.limbs[0];
    EXPECT_EQ(limb.actor, 0);
    EXPECT_EQ(limb.chain, "RightArm");
    EXPECT_EQ(limb.markerId, "handOnShoulder");
    EXPECT_TRUE(limb.reached);
    EXPECT_NEAR(length(limb.rootBoneDelta), 1.0, 1e-9);
    EXPECT_NEAR(length(limb.midBoneDelta), 1.0, 1e-9);
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_EQ(r.markers[0].id, "handOnShoulder");
}

TEST(AlignmentOutput, ChainsAreListedActorZeroFirstThenInMappingOrder)
{
    Scene s = opas::test::makeHugScene(1.0, 1.0);
    // Add the left hands too: actor 1's left hand first in the file, then actor 0's.
    const opas::test::SceneActor& a0 = s.actors[0];
    const opas::test::SceneActor& a1 = s.actors[1];
    const Vec3 onA1 = opas::test::backSurfacePoint(a1, 1.0, 0.45, -1.0, opas::test::kHugSwingRad);
    const Vec3 onA0 = opas::test::backSurfacePoint(a0, -1.0, 0.45, 1.0, opas::test::kHugSwingRad);
    opas::test::poseArmToward(s.actors[1], -1.0, onA0 + Vec3{0.0, 0.0, 0.1});
    opas::test::poseArmToward(s.actors[0], -1.0, onA1 + Vec3{0.0, 0.0, 0.1});
    s.markers.markers.push_back(makeMarkerAt(s, "a1Left", 1, "LeftHand", 0, "UpperBack", onA0));
    s.markers.markers.push_back(makeMarkerAt(s, "a0Left", 0, "LeftHand", 1, "UpperBack", onA1));

    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 4U);
    // Actor 0 first; within an actor, in BodyMapping::ikChains order (right arm, then left arm).
    EXPECT_EQ(r.limbs[0].actor, 0);
    EXPECT_EQ(r.limbs[0].chain, "RightArm");
    EXPECT_EQ(r.limbs[1].actor, 0);
    EXPECT_EQ(r.limbs[1].chain, "LeftArm");
    EXPECT_EQ(r.limbs[1].markerId, "a0Left");
    EXPECT_EQ(r.limbs[2].actor, 1);
    EXPECT_EQ(r.limbs[2].chain, "RightArm");
    EXPECT_EQ(r.limbs[3].actor, 1);
    EXPECT_EQ(r.limbs[3].chain, "LeftArm");
    EXPECT_EQ(r.limbs[3].markerId, "a1Left");
    // Markers are reported in file order.
    ASSERT_EQ(r.markers.size(), 4U);
    EXPECT_EQ(r.markers[2].id, "a1Left");
    EXPECT_EQ(r.markers[3].id, "a0Left");
}

TEST(AlignmentOutput, AMarkerOnABoneWithoutAChainIsMeasuredAfterTheRootShiftOnly)
{
    // Chest has no IK chain. Add a chest marker to a scene whose hand marker has one.
    Scene s = opas::test::makeShoulderScene(1.0, 1.0);
    s.markers.markers.push_back(chestMarker(s, "chest", 0, Vec3{0.0, 0.0, 0.0}));
    const AlignmentResult r = alignScene(s);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 1U); // only the hand marker drives a chain
    EXPECT_EQ(r.limbs[0].markerId, "handOnShoulder");
    ASSERT_EQ(r.markers.size(), 2U);
    const Scene shifted = shiftedScene(s, r);
    const InteractionMarker* chest = findMarker(s, "chest");
    ASSERT_NE(chest, nullptr);
    const double expected =
        length(bonePosition(shifted.actors[0], "Chest") - markerTargetPoint(shifted, *chest));
    EXPECT_EQ(r.markers[1].id, "chest");
    EXPECT_NEAR(r.markers[1].distanceM, expected, 1e-9);
}

// ---- Determinism ------------------------------------------------------------------------------

bool bitsEqual(double a, double b)
{
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool bitsEqual(Vec3 a, Vec3 b)
{
    return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y) && bitsEqual(a.z, b.z);
}

bool bitsEqual(opas::math::Quat a, opas::math::Quat b)
{
    return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y) && bitsEqual(a.z, b.z) && bitsEqual(a.w, b.w);
}

void expectBitIdentical(const AlignmentResult& a, const AlignmentResult& b)
{
    EXPECT_EQ(a.valid, b.valid);
    EXPECT_EQ(a.error, b.error);
    EXPECT_EQ(a.rootShiftClamped, b.rootShiftClamped);
    EXPECT_TRUE(bitsEqual(a.rootShiftM[0], b.rootShiftM[0]));
    EXPECT_TRUE(bitsEqual(a.rootShiftM[1], b.rootShiftM[1]));
    ASSERT_EQ(a.limbs.size(), b.limbs.size());
    for (std::size_t i = 0; i < a.limbs.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(a.limbs[i].actor, b.limbs[i].actor);
        EXPECT_EQ(a.limbs[i].chain, b.limbs[i].chain);
        EXPECT_EQ(a.limbs[i].markerId, b.limbs[i].markerId);
        EXPECT_EQ(a.limbs[i].reached, b.limbs[i].reached);
        EXPECT_TRUE(bitsEqual(a.limbs[i].rootBoneDelta, b.limbs[i].rootBoneDelta));
        EXPECT_TRUE(bitsEqual(a.limbs[i].midBoneDelta, b.limbs[i].midBoneDelta));
        EXPECT_TRUE(bitsEqual(a.limbs[i].penetrationM, b.limbs[i].penetrationM));
    }
    ASSERT_EQ(a.markers.size(), b.markers.size());
    for (std::size_t i = 0; i < a.markers.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(a.markers[i].id, b.markers[i].id);
        EXPECT_EQ(a.markers[i].met, b.markers[i].met);
        EXPECT_TRUE(bitsEqual(a.markers[i].distanceM, b.markers[i].distanceM));
    }
}

TEST(AlignmentDeterminism, TheSameInputsGiveBitIdenticalResults)
{
    struct Case {
        const char* name;
        Scene scene;
    };
    const std::vector<Case> cases = {
        {"hug 0.8/1.25", opas::test::makeHugScene(0.8, 1.25)},
        {"hug 1.25/0.8", opas::test::makeHugScene(1.25, 0.8)},
        {"handshake 0.8/1.25", opas::test::makeHandshakeScene(0.8, 1.25)},
        {"shoulder 1.25/0.8", opas::test::makeShoulderScene(1.25, 0.8)},
        {"penetration", opas::test::makePenetrationScene()},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.name);
        const AlignmentResult first = alignScene(c.scene);
        const AlignmentResult second = alignScene(c.scene);
        ASSERT_TRUE(first.valid) << first.error;
        expectBitIdentical(first, second);
    }
}

TEST(AlignmentDeterminism, AnIdenticalSceneBuiltAgainGivesTheSameBits)
{
    // A fresh copy of the data at another address must not change a single bit.
    const AlignmentResult first = alignScene(opas::test::makeHugScene(0.8, 1.25));
    const AlignmentResult second = alignScene(opas::test::makeHugScene(0.8, 1.25));
    ASSERT_TRUE(first.valid) << first.error;
    expectBitIdentical(first, second);
}

TEST(AlignmentDeterminism, CallingForOtherInputsInBetweenDoesNotChangeTheResult)
{
    const Scene hug = opas::test::makeHugScene(0.8, 1.25);
    const AlignmentResult before = alignScene(hug);
    (void)alignScene(opas::test::makePenetrationScene());
    (void)alignScene(opas::test::makeShoulderScene(1.25, 0.8));
    const AlignmentResult after = alignScene(hug);
    expectBitIdentical(before, after);
}

// ---- Invalid input ------------------------------------------------------------------------------

void expectInvalid(const AlignmentResult& r)
{
    EXPECT_FALSE(r.valid);
    EXPECT_FALSE(r.error.empty());
}

Scene validScene()
{
    return opas::test::makeShoulderScene(1.0, 1.0);
}

TEST(AlignmentInvalid, AValidSceneIsAccepted)
{
    const AlignmentResult r = alignScene(validScene());
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.error.empty());
}

TEST(AlignmentInvalid, PoseLengthThatDoesNotMatchTheSkeleton)
{
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        Scene shorter = validScene();
        shorter.actors[actor].pose.pop_back();
        expectInvalid(alignScene(shorter));

        Scene longer = validScene();
        longer.actors[actor].pose.push_back(opas::math::Transform{});
        expectInvalid(alignScene(longer));

        Scene empty = validScene();
        empty.actors[actor].pose.clear();
        expectInvalid(alignScene(empty));
    }
}

TEST(AlignmentInvalid, InvalidSkeleton)
{
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        Scene parentAfterChild = validScene();
        parentAfterChild.actors[actor].body.skeleton.bones[3].parent = 3;
        expectInvalid(alignScene(parentAfterChild));

        Scene emptyName = validScene();
        emptyName.actors[actor].body.skeleton.bones[2].name = "";
        expectInvalid(alignScene(emptyName));

        Scene notUnit = validScene();
        notUnit.actors[actor].body.skeleton.bones[1].localRest.rotation =
            opas::math::Quat{0.0, 0.0, 0.0, 2.0};
        expectInvalid(alignScene(notUnit));
    }
}

// Each way a BodyMapping can be invalid is its own test, so a failure names the rule.
Scene sceneWithMappingEdit(int actor, void (*edit)(opas::markers::BodyMapping&))
{
    Scene s = validScene();
    edit(s.actors[static_cast<std::size_t>(actor)].body.mapping);
    return s;
}

TEST(AlignmentInvalid, MappingCapsuleNamingAMissingBone)
{
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.capsules[1].endBone = "NoSuchBone"; })));
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.capsules[1].bone = "NoSuchBone"; })));
    }
}

TEST(AlignmentInvalid, MappingCapsuleWithNonPositiveRadiusRatio)
{
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.capsules[1].radiusRatio = 0.0; })));
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.capsules[1].radiusRatio = -0.1; })));
    }
}

TEST(AlignmentInvalid, MappingWithAnEmptyLimb)
{
    // Test choice: a mapping that rig/markers would reject in measureBody is invalid here too.
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.limbs[0].bones.clear(); })));
    }
}

TEST(AlignmentInvalid, MappingWithTwoCapsulesForOneRegion)
{
    // Test choice: a region name that two capsules share would make a marker ambiguous.
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.capsules[2].region = "UpperBack"; })));
    }
}

TEST(AlignmentInvalid, ChainWithNonPositiveLimbRadius)
{
    // Test choice: IkChain::limbRadiusM is documented as "> 0".
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.ikChains[0].limbRadiusM = 0.0; })));
        expectInvalid(alignScene(sceneWithMappingEdit(
            actor, [](opas::markers::BodyMapping& m) { m.ikChains[1].limbRadiusM = -0.01; })));
    }
}

TEST(AlignmentInvalid, ChainNamingAMissingBone)
{
    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        Scene root = validScene();
        root.actors[actor].body.mapping.ikChains[0].rootBone = "NoSuchBone";
        expectInvalid(alignScene(root));

        Scene mid = validScene();
        mid.actors[actor].body.mapping.ikChains[1].midBone = "NoSuchBone";
        expectInvalid(alignScene(mid));

        Scene end = validScene();
        end.actors[actor].body.mapping.ikChains[0].endBone = "NoSuchBone";
        expectInvalid(alignScene(end));
    }
}

TEST(AlignmentInvalid, BothWeightsZero)
{
    Scene s = validScene();
    s.actors[0].weight = 0.0;
    s.actors[1].weight = 0.0;
    expectInvalid(alignScene(s));
}

TEST(AlignmentInvalid, NegativeWeight)
{
    Scene first = validScene();
    first.actors[0].weight = -1.0;
    expectInvalid(alignScene(first));

    Scene second = validScene();
    second.actors[1].weight = -0.5;
    expectInvalid(alignScene(second));

    // A negative weight is invalid even when the sum would be positive.
    Scene sum = validScene();
    sum.actors[0].weight = 3.0;
    sum.actors[1].weight = -1.0;
    expectInvalid(alignScene(sum));
}

TEST(AlignmentInvalid, MarkerNamingAMissingSourceBone)
{
    Scene s = validScene();
    s.markers.markers[0].sourceBone = "NoSuchBone";
    expectInvalid(alignScene(s));
}

TEST(AlignmentInvalid, MarkerNamingAMissingTargetRegion)
{
    Scene s = validScene();
    s.markers.markers[0].targetRegion = "NoSuchRegion";
    expectInvalid(alignScene(s));
}

TEST(AlignmentInvalid, MarkerNamingAnActorIndexTheSolverDoesNotHave)
{
    Scene source = validScene();
    source.markers.markers[0].sourceActor = 2;
    expectInvalid(alignScene(source));

    Scene target = validScene();
    target.markers.markers[0].targetActor = 5;
    expectInvalid(alignScene(target));

    Scene negative = validScene();
    negative.markers.markers[0].sourceActor = -1;
    expectInvalid(alignScene(negative));

    Scene negativeTarget = validScene();
    negativeTarget.markers.markers[0].targetActor = -1;
    expectInvalid(alignScene(negativeTarget));
}

TEST(AlignmentInvalid, NonFiniteInput)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    for (int actor = 0; actor < 2; ++actor) {
        SCOPED_TRACE(actor);
        Scene rootNan = validScene();
        rootNan.actors[actor].root.translation.x = nan;
        expectInvalid(alignScene(rootNan));

        Scene rootInf = validScene();
        rootInf.actors[actor].root.translation.y = inf;
        expectInvalid(alignScene(rootInf));

        Scene rootRotation = validScene();
        rootRotation.actors[actor].root.rotation = opas::math::Quat{nan, 0.0, 0.0, 1.0};
        expectInvalid(alignScene(rootRotation));

        Scene poseNan = validScene();
        poseNan.actors[actor].pose[5].translation.z = nan;
        expectInvalid(alignScene(poseNan));

        Scene poseRotation = validScene();
        poseRotation.actors[actor].pose[6].rotation = opas::math::Quat{0.0, inf, 0.0, 1.0};
        expectInvalid(alignScene(poseRotation));

        Scene weightNan = validScene();
        weightNan.actors[actor].weight = nan;
        expectInvalid(alignScene(weightNan));

        Scene weightInf = validScene();
        weightInf.actors[actor].weight = inf;
        expectInvalid(alignScene(weightInf));
    }

    Scene offset = validScene();
    offset.markers.markers[0].offsetM.y = nan;
    expectInvalid(alignScene(offset));

    Scene offsetInf = validScene();
    offsetInf.markers.markers[0].offsetM.x = inf;
    expectInvalid(alignScene(offsetInf));

    Scene time = validScene();
    time.clipTimeS = nan;
    expectInvalid(alignScene(time));
    time.clipTimeS = inf;
    expectInvalid(alignScene(time));

    AlignmentSettings maxShift;
    maxShift.maxRootShiftM = nan;
    expectInvalid(alignScene(validScene(), maxShift));
    maxShift.maxRootShiftM = inf;
    expectInvalid(alignScene(validScene(), maxShift));
}

TEST(AlignmentInvalid, PoleSamplesBelowOne)
{
    // Test choice: AlignmentSettings::poleSamples is documented as ">= 1".
    for (const int samples : {0, -1}) {
        AlignmentSettings settings;
        settings.poleSamples = samples;
        expectInvalid(alignScene(validScene(), settings));
    }
    AlignmentSettings one;
    one.poleSamples = 1;
    EXPECT_TRUE(alignScene(validScene(), one).valid);
}

TEST(AlignmentInvalid, ARejectedCallLeavesALaterValidCallUnaffected)
{
    const Scene good = validScene();
    const AlignmentResult before = alignScene(good);
    Scene bad = validScene();
    bad.actors[0].weight = -1.0;
    expectInvalid(alignScene(bad));
    const AlignmentResult after = alignScene(good);
    expectBitIdentical(before, after);
}

} // namespace
