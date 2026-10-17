// A2 body-size scenes: synthetic humanoids scaled 0.8 and 1.25 (a 1.56 times height ratio),
// in both pairings, solved and then checked with independent recomputation
// (tests/common/AlignmentOracle.hpp).

#include "AlignmentScenes.hpp"
#include "AlignmentOracle.hpp"
#include "SyntheticHumanoid.hpp"
#include "TestSupport.hpp"
#include "opas/markers/Alignment.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace {

using opas::markers::AlignmentResult;
using opas::markers::AlignmentSettings;
using opas::markers::InteractionMarker;
using opas::markers::LimbCorrection;
using opas::math::distance;
using opas::math::length;
using opas::math::Vec3;
using opas::test::alignScene;
using opas::test::bonePosition;
using opas::test::findMarker;
using opas::test::limbPenetrationM;
using opas::test::markerTargetPoint;
using opas::test::Scene;
using opas::test::shiftedScene;
using opas::test::solvedLimb;

constexpr double kMarkerToleranceM = 0.02; // the scenes' markers
constexpr double kMaxPenetrationM = 0.005; // acceptance criterion
constexpr double kBoneLengthTolM = 1e-9;
constexpr double kReportedVsRecomputedM = 1e-6;

// The two pairings of the body-size scenes, plus equal sizes.
constexpr std::array<std::pair<double, double>, 3> kPairings = {
    {{0.8, 1.25}, {1.25, 0.8}, {1.0, 1.0}}};

const LimbCorrection* limbDrivenBy(const AlignmentResult& r, const std::string& markerId)
{
    for (const LimbCorrection& l : r.limbs) {
        if (l.markerId == markerId) {
            return &l;
        }
    }
    return nullptr;
}

// Solves the scene and checks the A2 body-size criteria on it. Every marker of these scenes
// is driven by a limb chain.
void expectSceneSolved(const Scene& scene)
{
    const AlignmentResult r = alignScene(scene);
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_FALSE(r.rootShiftClamped);
    EXPECT_EQ(r.rootShiftM[0].z, 0.0);
    EXPECT_EQ(r.rootShiftM[1].z, 0.0);
    ASSERT_EQ(r.markers.size(), scene.markers.markers.size());
    ASSERT_EQ(r.limbs.size(), scene.markers.markers.size());

    const Scene shifted = shiftedScene(scene, r);
    for (std::size_t i = 0; i < scene.markers.markers.size(); ++i) {
        const InteractionMarker& m = scene.markers.markers[i];
        SCOPED_TRACE(m.id);

        // The solver says the marker is met within its tolerance...
        EXPECT_EQ(r.markers[i].id, m.id);
        EXPECT_TRUE(r.markers[i].met) << "residual " << r.markers[i].distanceM;
        EXPECT_LE(r.markers[i].distanceM, m.toleranceM);
        EXPECT_LE(m.toleranceM, kMarkerToleranceM + 1e-12);

        // ...and it is, when the hand is recomputed from the reported rotations.
        const LimbCorrection* limb = limbDrivenBy(r, m.id);
        ASSERT_NE(limb, nullptr);
        EXPECT_TRUE(limb->reached);
        const auto solved = solvedLimb(shifted, *limb);
        const Vec3 target = markerTargetPoint(shifted, m);
        const double handError = distance(solved.end, target);
        EXPECT_LE(handError, m.toleranceM);
        EXPECT_NEAR(handError, r.markers[i].distanceM, kReportedVsRecomputedM);

        // Bone lengths are kept (the rotations only turn the bones).
        const auto& actor = shifted.actors[static_cast<std::size_t>(limb->actor)];
        const auto* chain = opas::test::findChain(actor, limb->chain);
        ASSERT_NE(chain, nullptr);
        EXPECT_NEAR(
            distance(solved.root, solved.mid),
            distance(bonePosition(actor, chain->rootBone), bonePosition(actor, chain->midBone)),
            kBoneLengthTolM);
        EXPECT_NEAR(
            distance(solved.mid, solved.end),
            distance(bonePosition(actor, chain->midBone), bonePosition(actor, chain->endBone)),
            kBoneLengthTolM);

        // The rotations are unit quaternions.
        EXPECT_NEAR(length(limb->rootBoneDelta), 1.0, 1e-9);
        EXPECT_NEAR(length(limb->midBoneDelta), 1.0, 1e-9);

        // No limb penetrates the other actor's capsules by more than 5 mm: as reported, and
        // as recomputed from the solved bones.
        EXPECT_LE(limb->penetrationM, kMaxPenetrationM);
        EXPECT_LE(limbPenetrationM(shifted, *limb, m.targetRegion), kMaxPenetrationM);

        // The scene needed the limb IK: after the roots have moved, the clip's hand is still
        // farther from the contact than the marker tolerance.
        const Vec3 clipHand =
            bonePosition(shifted.actors[static_cast<std::size_t>(m.sourceActor)], m.sourceBone);
        EXPECT_GT(distance(clipHand, target), m.toleranceM);
    }
}

// ---- (a) Hug ------------------------------------------------------------------------------

TEST(AlignmentScenes, HugBetweenDifferentBodySizes)
{
    for (const auto& [scale0, scale1] : kPairings) {
        SCOPED_TRACE("scales " + std::to_string(scale0) + " / " + std::to_string(scale1));
        expectSceneSolved(opas::test::makeHugScene(scale0, scale1));
    }
}

// ---- (b) Handshake --------------------------------------------------------------------------

TEST(AlignmentScenes, HandshakeBetweenDifferentBodySizes)
{
    for (const auto& [scale0, scale1] : kPairings) {
        for (int source = 0; source < 2; ++source) {
            SCOPED_TRACE("scales " + std::to_string(scale0) + " / " + std::to_string(scale1) +
                         ", hand moved by actor " + std::to_string(source));
            expectSceneSolved(opas::test::makeHandshakeScene(scale0, scale1, source));
        }
    }
}

// ---- (c) A hand on the other's shoulder -------------------------------------------------------

TEST(AlignmentScenes, HandOnTheOthersShoulderBetweenDifferentBodySizes)
{
    for (const auto& [scale0, scale1] : kPairings) {
        SCOPED_TRACE("scales " + std::to_string(scale0) + " / " + std::to_string(scale1));
        expectSceneSolved(opas::test::makeShoulderScene(scale0, scale1));
    }
}

TEST(AlignmentScenes, ShoulderSceneHasTheTallerActorDoTheTouching)
{
    // Documents a scene-design choice (see makeShoulderScene): the toucher is the taller actor.
    const Scene tallFirst = opas::test::makeShoulderScene(1.25, 0.8);
    EXPECT_EQ(tallFirst.markers.markers.at(0).sourceActor, 0);
    const Scene tallSecond = opas::test::makeShoulderScene(0.8, 1.25);
    EXPECT_EQ(tallSecond.markers.markers.at(0).sourceActor, 1);
}

// ---- Weights in a scene -----------------------------------------------------------------------

TEST(AlignmentScenes, PinnedActorStaysPutAndTheOtherOneTakesTheWholeShift)
{
    for (const auto& [scale0, scale1] : kPairings) {
        SCOPED_TRACE("scales " + std::to_string(scale0) + " / " + std::to_string(scale1));
        Scene scene = opas::test::makeHugScene(scale0, scale1);
        scene.actors[1].weight = 0.0; // e.g. an actor lying in a bed
        expectSceneSolved(scene);
        const AlignmentResult r = alignScene(scene);
        ASSERT_TRUE(r.valid) << r.error;
        EXPECT_EQ(r.rootShiftM[1].x, 0.0);
        EXPECT_EQ(r.rootShiftM[1].y, 0.0);
        EXPECT_EQ(r.rootShiftM[1].z, 0.0);
        EXPECT_GT(length(r.rootShiftM[0]), 0.05);
    }
}

TEST(AlignmentScenes, UnequalWeightsStillMeetTheMarkers)
{
    Scene scene = opas::test::makeShoulderScene(1.0, 1.25);
    scene.actors[0].weight = 3.0;
    scene.actors[1].weight = 1.0;
    expectSceneSolved(scene);
}

// ---- Penetration avoidance
// -----------------------------------------------------------------------

TEST(AlignmentPenetration, TheOnlyPoleDrivesTheArmThroughTheOtherActor)
{
    // Sets up the problem: with poleSamples = 1 (only the chain's own pole is tried) the elbow
    // and forearm run through the partner's body. The marker is still met; the penetration is
    // large and reported.
    const Scene scene = opas::test::makePenetrationScene();
    AlignmentSettings settings;
    settings.poleSamples = 1;
    const AlignmentResult r = alignScene(scene, settings);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 1U);
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_TRUE(r.markers[0].met);
    EXPECT_GT(r.limbs[0].penetrationM, 0.05);

    // The same number, recomputed from the reported rotations.
    const Scene shifted = shiftedScene(scene, r);
    const InteractionMarker* marker = findMarker(scene, "handAcrossToShoulder");
    ASSERT_NE(marker, nullptr);
    EXPECT_NEAR(limbPenetrationM(shifted, r.limbs[0], marker->targetRegion),
                r.limbs[0].penetrationM, 1e-6);
}

TEST(AlignmentPenetration, PoleSearchFindsAWayAroundAndTheMarkerIsStillMet)
{
    const Scene scene = opas::test::makePenetrationScene();
    const AlignmentSettings defaults;
    ASSERT_GE(defaults.poleSamples, 2);
    const AlignmentResult r = alignScene(scene, defaults);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 1U);
    ASSERT_EQ(r.markers.size(), 1U);

    EXPECT_LE(r.limbs[0].penetrationM, kMaxPenetrationM);
    EXPECT_TRUE(r.limbs[0].reached);
    EXPECT_TRUE(r.markers[0].met) << "residual " << r.markers[0].distanceM;
    EXPECT_LE(r.markers[0].distanceM, kMarkerToleranceM);

    const Scene shifted = shiftedScene(scene, r);
    const InteractionMarker* marker = findMarker(scene, "handAcrossToShoulder");
    ASSERT_NE(marker, nullptr);
    const auto solved = solvedLimb(shifted, r.limbs[0]);
    EXPECT_LE(distance(solved.end, markerTargetPoint(shifted, *marker)), kMarkerToleranceM);
    EXPECT_LE(limbPenetrationM(shifted, r.limbs[0], marker->targetRegion), kMaxPenetrationM);
}

TEST(AlignmentPenetration, ASecondPoleSampleIsEnoughHere)
{
    // With two samples the pole is tried as given and turned half way round the axis. In this
    // scene the half-turned pole is clear of the other actor, and the search keeps it.
    const Scene scene = opas::test::makePenetrationScene();
    AlignmentSettings settings;
    settings.poleSamples = 2;
    const AlignmentResult r = alignScene(scene, settings);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.limbs.size(), 1U);
    EXPECT_LE(r.limbs[0].penetrationM, kMaxPenetrationM);
    EXPECT_TRUE(r.markers[0].met);
}

TEST(AlignmentPenetration, AClearPoseKeepsTheChainsOwnPole)
{
    // When the chain's own pole already gives no penetration, the pole is not changed: the
    // result with 1 sample and with 16 samples is bit-identical (ties go to the original pole).
    const Scene scene = opas::test::makeHandshakeScene(1.0, 1.0);
    AlignmentSettings one;
    one.poleSamples = 1;
    const AlignmentResult a = alignScene(scene, one);
    const AlignmentResult b = alignScene(scene, AlignmentSettings{});
    ASSERT_TRUE(a.valid) << a.error;
    ASSERT_TRUE(b.valid) << b.error;
    ASSERT_EQ(a.limbs.size(), 1U);
    ASSERT_EQ(b.limbs.size(), 1U);
    EXPECT_EQ(a.limbs[0].penetrationM, 0.0);
    EXPECT_EQ(b.limbs[0].penetrationM, 0.0);
    EXPECT_EQ(a.limbs[0].rootBoneDelta, b.limbs[0].rootBoneDelta);
    EXPECT_EQ(a.limbs[0].midBoneDelta, b.limbs[0].midBoneDelta);
}

} // namespace
