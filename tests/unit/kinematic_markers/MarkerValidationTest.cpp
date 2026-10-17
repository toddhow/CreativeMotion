// A4b marker validation (docs/tasks/0007-a4b-marker-validation.md): scaleActorPose and
// validateMarkers on a synthetic scene.
//
// The scene: two synthetic humanoids (SyntheticHumanoid.hpp, scale 1, rest pose) facing each other
// 0.3 m apart, recorded in a clip of 8 frames at 10 Hz. Both body maps get an extra capsule
// "RightFootBox" whose bone is the right foot, so that a marker can target a point that scaling
// does not move vertically (see below). Validation keeps only root alignment, so a marker's
// residual is its horizontal leftover (after the root shift) plus its vertical gap.
//
// How the numbers are derived (rule 1): an actor's ground point g is (x, y of its first bone,
// lowest z of all its bones) = (pelvis x, y; foot z = 0.05). Scaling by s moves a bone at height z
// to 0.05 + s * (z - 0.05). The feet are at 0.05, so feet never move vertically. The hand is at
// z = 0.78 (shoulder 1.48 minus arm 0.70), so its height above the ground point is dz = 0.73, and
// at scale s0 the hand is (s0 - 1) * dz away from where it was. A marker offset is in meters in the
// region frame and is not scaled, so a marker that targets the other actor's right foot, with the
// offset that leads to the hand's unscaled position, wants z = 0.78 whatever s1 is: its residual
// at (s0, s1) is |s0 - 1| * dz = 0.146 (s0 = 0.8), 0 (1.0), 0.1825 (1.25), once the horizontal gap
// has been closed by the root shift.

#include "AlignmentScenes.hpp"
#include "ClipBuilder.hpp"
#include "SyntheticHumanoid.hpp"
#include "TestSupport.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/markers/InteractionMarker.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/markers/Validation.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using opas::markers::BodyMapping;
using opas::markers::FailedPairing;
using opas::markers::InteractionMarker;
using opas::markers::MarkerSet;
using opas::markers::MarkerValidation;
using opas::markers::PoseClip;
using opas::markers::scaleActorPose;
using opas::markers::validateMarkers;
using opas::markers::ValidationReport;
using opas::markers::ValidationSettings;
using opas::math::fromAxisAngle;
using opas::math::Quat;
using opas::math::Transform;
using opas::math::Vec3;
using opas::test::appendFrame;
using opas::test::bonePosition;
using opas::test::faceToFace;
using opas::test::makeHumanoid;
using opas::test::makeMarkerAt;
using opas::test::Scene;
using opas::test::Vec3Near;
using opas::test::worldOf;

constexpr double kTol = 1e-9;
constexpr double kHandDz = 0.73; // hand height (0.78) above the lowest bone (0.05)
constexpr double kRateHz = 10.0;

// ---- scaleActorPose (rule 1)
// ---------------------------------------------------------------------

TEST(ScaleActorPose, ScalesPositionsAboutTheGroundPoint)
{
    // Three bones. The ground point is (x, y of the first bone, lowest z) = (1, 2, 0.1).
    const Quat qa = fromAxisAngle(Vec3{0, 0, 1}, 0.5);
    const Quat qb = fromAxisAngle(Vec3{1, 0, 0}, 1.0);
    const Quat qc = fromAxisAngle(Vec3{0, 1, 0}, -0.7);
    const std::vector<Transform> frame = {
        Transform{Vec3{1.0, 2.0, 0.5}, qa},
        Transform{Vec3{3.0, 2.0, 2.0}, qb},
        Transform{Vec3{0.0, 0.0, 0.1}, qc},
    };
    // Scale 2: g + 2 * (p - g).
    const std::vector<Transform> doubled = scaleActorPose(frame, 2.0);
    ASSERT_EQ(doubled.size(), 3U);
    EXPECT_PRED_FORMAT3(Vec3Near, doubled[0].translation, (Vec3{1.0, 2.0, 0.9}),
                        kTol); // (0, 0, 0.4) -> (0, 0, 0.8)
    EXPECT_PRED_FORMAT3(Vec3Near, doubled[1].translation, (Vec3{5.0, 2.0, 3.9}),
                        kTol); // (2, 0, 1.9) -> (4, 0, 3.8)
    EXPECT_PRED_FORMAT3(Vec3Near, doubled[2].translation, (Vec3{-1.0, -2.0, 0.1}),
                        kTol); // (-1,-2,0) -> (-2,-4,0)

    // Scale 0.5.
    const std::vector<Transform> halved = scaleActorPose(frame, 0.5);
    EXPECT_PRED_FORMAT3(Vec3Near, halved[0].translation, (Vec3{1.0, 2.0, 0.3}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, halved[1].translation, (Vec3{2.0, 2.0, 1.05}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, halved[2].translation, (Vec3{0.5, 1.0, 0.1}), kTol);
}

TEST(ScaleActorPose, RotationsAreUnchanged)
{
    const Quat qa = fromAxisAngle(Vec3{0, 0, 1}, 0.5);
    const Quat qb = fromAxisAngle(Vec3{1, 0, 0}, 1.0);
    const std::vector<Transform> frame = {Transform{Vec3{1.0, 2.0, 0.5}, qa},
                                          Transform{Vec3{3.0, 2.0, 2.0}, qb}};
    for (const double scale : {0.8, 1.0, 1.25, 3.0}) {
        SCOPED_TRACE(scale);
        const std::vector<Transform> scaled = scaleActorPose(frame, scale);
        ASSERT_EQ(scaled.size(), 2U);
        EXPECT_EQ(scaled[0].rotation, qa); // exactly the same quaternion
        EXPECT_EQ(scaled[1].rotation, qb);
    }
}

TEST(ScaleActorPose, ScaleOneIsTheIdentity)
{
    const std::vector<Transform> frame = {
        Transform{Vec3{1.0, 2.0, 0.5}, fromAxisAngle(Vec3{0, 0, 1}, 0.5)},
        Transform{Vec3{-3.0, 2.5, 2.0}, fromAxisAngle(Vec3{1, 0, 0}, 1.0)},
        Transform{Vec3{0.25, -7.0, -0.3}, Quat{}},
    };
    const std::vector<Transform> same = scaleActorPose(frame, 1.0);
    ASSERT_EQ(same.size(), frame.size());
    for (std::size_t i = 0; i < frame.size(); ++i) {
        EXPECT_PRED_FORMAT3(Vec3Near, same[i].translation, frame[i].translation, 1e-12);
        EXPECT_EQ(same[i].rotation, frame[i].rotation);
    }
}

TEST(ScaleActorPose, TheGroundPointUsesTheLowestZOfAnyBoneAndNegativeZIsAllowed)
{
    // The lowest bone is the last one, at z = -1: g = (0, 0, -1). Scale 3: z = -1 + 3 * (z + 1).
    const std::vector<Transform> frame = {Transform{Vec3{0.0, 0.0, 1.0}, Quat{}},
                                          Transform{Vec3{1.0, 1.0, 0.0}, Quat{}},
                                          Transform{Vec3{2.0, -1.0, -1.0}, Quat{}}};
    const std::vector<Transform> tripled = scaleActorPose(frame, 3.0);
    EXPECT_PRED_FORMAT3(Vec3Near, tripled[0].translation, (Vec3{0.0, 0.0, 5.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, tripled[1].translation, (Vec3{3.0, 3.0, 2.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, tripled[2].translation, (Vec3{6.0, -3.0, -1.0}), kTol);
}

TEST(ScaleActorPose, TheFirstBoneKeepsItsXYAndAnyBoneAtTheLowestZKeepsItsZ)
{
    const std::vector<Transform> frame = {Transform{Vec3{4.0, 5.0, 2.0}, Quat{}},
                                          Transform{Vec3{6.0, 5.0, 0.25}, Quat{}}};
    for (const double scale : {0.8, 1.25, 2.0}) {
        SCOPED_TRACE(scale);
        const std::vector<Transform> scaled = scaleActorPose(frame, scale);
        EXPECT_NEAR(scaled[0].translation.x, 4.0, kTol);
        EXPECT_NEAR(scaled[0].translation.y, 5.0, kTol);
        EXPECT_NEAR(scaled[1].translation.z, 0.25, kTol);
    }
}

TEST(ScaleActorPose, OneBoneStaysWhereItIs)
{
    const std::vector<Transform> frame = {Transform{Vec3{1.0, 2.0, 3.0}, Quat{}}};
    const std::vector<Transform> scaled = scaleActorPose(frame, 5.0);
    ASSERT_EQ(scaled.size(), 1U);
    EXPECT_PRED_FORMAT3(Vec3Near, scaled[0].translation, (Vec3{1.0, 2.0, 3.0}), kTol);
}

TEST(ScaleActorPose, AnEmptyFrameGivesAnEmptyFrame)
{
    EXPECT_TRUE(scaleActorPose(std::vector<Transform>{}, 2.0).empty());
}

// ---- Scene for validateMarkers
// ------------------------------------------------------------------------

BodyMapping footBoxMapping()
{
    BodyMapping m = makeHumanoid(1.0).mapping;
    m.capsules.push_back({"RightFootBox", "RightFoot", "RightShin", 0.5});
    return m;
}

Scene footScene(double separationM)
{
    Scene s = faceToFace(1.0, 1.0, separationM);
    for (opas::test::SceneActor& actor : s.actors) {
        actor.body.mapping = footBoxMapping();
    }
    return s;
}

PoseClip repeatedClip(const Scene& s, std::size_t frames)
{
    PoseClip clip;
    clip.sampleRateHz = kRateHz;
    for (std::size_t f = 0; f < frames; ++f) {
        appendFrame(clip, s.actors[0], s.actors[1]);
    }
    return clip;
}

std::array<BodyMapping, 2> mapsOf(const Scene& s)
{
    return {s.actors[0].body.mapping, s.actors[1].body.mapping};
}

// Actor 0's right foot to actor 1's right foot. Scaling does not move either vertically.
InteractionMarker footMarker(const Scene& s, double startS, double endS)
{
    InteractionMarker m = makeMarkerAt(s, "footOnFoot", 0, "RightFoot", 1, "RightFootBox",
                                       bonePosition(s.actors[1], "RightFoot"));
    m.window = {startS, endS};
    return m;
}

// Actor 0's right hand to the point where it hangs at scale 1, described in actor 1's foot frame.
// Tolerance 0.165 lies between the residuals at s0 = 0.8 (0.146) and s0 = 1.25 (0.1825).
InteractionMarker handMarker(const Scene& s, double startS, double endS)
{
    InteractionMarker m = makeMarkerAt(s, "handOnFoot", 0, "RightHand", 1, "RightFootBox",
                                       bonePosition(s.actors[0], "RightHand"));
    m.window = {startS, endS};
    m.toleranceM = 0.165;
    return m;
}

const FailedPairing* failureAt(const MarkerValidation& v, double s0, double s1)
{
    for (const FailedPairing& f : v.failures) {
        if (f.scale0 == s0 && f.scale1 == s1) {
            return &f;
        }
    }
    return nullptr;
}

TEST(ValidationScene, TheHandsHeightAboveTheLowestBoneIsAsDerived)
{
    const Scene s = footScene(0.3);
    EXPECT_NEAR(bonePosition(s.actors[0], "RightHand").z, 0.78, 1e-12);
    EXPECT_NEAR(bonePosition(s.actors[0], "RightFoot").z, 0.05, 1e-12);
    const std::vector<Transform> world = worldOf(s.actors[0]);
    double lowest = world[0].translation.z;
    for (const Transform& t : world) {
        lowest = std::min(lowest, t.translation.z);
    }
    EXPECT_NEAR(lowest, 0.05, 1e-12);
    EXPECT_NEAR(bonePosition(s.actors[0], "RightHand").z - lowest, kHandDz, 1e-12);
}

// ---- Rules 2-4
// ------------------------------------------------------------------------------------------

TEST(ValidateMarkers, AReachableMarkerPassesAtEveryPairingOfTheDefaultSweep)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(footMarker(s, 0.0, 0.1));
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s));
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_EQ(r.markers[0].id, "footOnFoot");
    EXPECT_TRUE(r.markers[0].failures.empty());
    EXPECT_TRUE(r.markers[0].passed());
    EXPECT_NEAR(r.markers[0].worstResidualM, 0.0, 1e-6);
    EXPECT_TRUE(r.allPassed());
    // (The humanoid body map has IK chains whose bones are parent and child. The flat skeleton of
    // rule 3 would make them invalid, so passing here also shows the chains were removed.)
}

TEST(ValidateMarkers, AnUnreachableMarkerFailsAtExactlyTheHandDerivedPairings)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(handMarker(s, 0.5, 0.6));
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s));
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 1U);
    const MarkerValidation& v = r.markers[0];
    EXPECT_FALSE(v.passed());
    EXPECT_FALSE(r.allPassed());

    // Residual (s0 - 1) * dz: 0.146 at s0 = 0.8 (within 0.165), 0 at 1, 0.1825 at 1.25 (beyond).
    // It does not depend on s1. So it fails exactly at the three pairings with s0 = 1.25.
    ASSERT_EQ(v.failures.size(), 3U);
    const double expectedResidual = 0.25 * kHandDz; // 0.1825
    EXPECT_EQ(v.failures[0].scale0, 1.25);
    EXPECT_EQ(v.failures[0].scale1, 0.8);
    EXPECT_EQ(v.failures[1].scale0, 1.25);
    EXPECT_EQ(v.failures[1].scale1, 1.0);
    EXPECT_EQ(v.failures[2].scale0, 1.25);
    EXPECT_EQ(v.failures[2].scale1, 1.25);
    for (const FailedPairing& f : v.failures) {
        EXPECT_NEAR(f.residualM, expectedResidual, 1e-6);
        EXPECT_FALSE(f.clamped);
        EXPECT_TRUE(f.error.empty());
    }
    EXPECT_NEAR(v.worstResidualM, expectedResidual, 1e-6);
    // The pairings that are not listed pass, including the ones with the 0.146 residual.
    EXPECT_EQ(failureAt(v, 0.8, 0.8), nullptr);
    EXPECT_EQ(failureAt(v, 0.8, 1.25), nullptr);
    EXPECT_EQ(failureAt(v, 1.0, 1.0), nullptr);
}

TEST(ValidateMarkers, TheReportListsMarkersInMarkerSetOrderEachWithItsOwnResult)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(handMarker(s, 0.5, 0.6));
    markers.markers.push_back(footMarker(s, 0.0, 0.1));
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s));
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 2U);
    EXPECT_EQ(r.markers[0].id, "handOnFoot");
    EXPECT_EQ(r.markers[1].id, "footOnFoot");
    EXPECT_FALSE(r.markers[0].passed());
    EXPECT_TRUE(r.markers[1].passed());
    EXPECT_FALSE(r.allPassed());

    // The other order gives the other order.
    MarkerSet reversed;
    reversed.markers = {markers.markers[1], markers.markers[0]};
    const ValidationReport q = validateMarkers(reversed, repeatedClip(s, 8), mapsOf(s));
    ASSERT_EQ(q.markers.size(), 2U);
    EXPECT_EQ(q.markers[0].id, "footOnFoot");
    EXPECT_EQ(q.markers[1].id, "handOnFoot");
}

TEST(ValidateMarkers, FailingPairingsAreListedInSweepOrder)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(handMarker(s, 0.5, 0.6));
    ValidationSettings settings;
    settings.scales = {1.25, 0.8}; // pairings: (1.25, 1.25), (1.25, 0.8), (0.8, 1.25), (0.8, 0.8)
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s), settings);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 1U);
    const std::vector<FailedPairing>& f = r.markers[0].failures;
    ASSERT_EQ(f.size(), 2U); // the two with s0 = 1.25, in the order given
    EXPECT_EQ(f[0].scale0, 1.25);
    EXPECT_EQ(f[0].scale1, 1.25);
    EXPECT_EQ(f[1].scale0, 1.25);
    EXPECT_EQ(f[1].scale1, 0.8);
}

TEST(ValidateMarkers, AShorterSweepTriesFewerPairings)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(handMarker(s, 0.5, 0.6));

    ValidationSettings one;
    one.scales = {1.0};
    const ValidationReport same = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s), one);
    ASSERT_TRUE(same.valid) << same.error;
    EXPECT_TRUE(same.allPassed()); // the animation as recorded: the hand is where it was

    ValidationSettings tall;
    tall.scales = {1.25};
    const ValidationReport big = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s), tall);
    ASSERT_TRUE(big.valid) << big.error;
    ASSERT_EQ(big.markers[0].failures.size(), 1U); // only (1.25, 1.25)
    EXPECT_EQ(big.markers[0].failures[0].scale0, 1.25);
    EXPECT_EQ(big.markers[0].failures[0].scale1, 1.25);

    ValidationSettings two;
    two.scales = {0.8, 1.25};
    const ValidationReport mixed = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s), two);
    ASSERT_EQ(mixed.markers[0].failures.size(), 2U); // (1.25, 0.8), (1.25, 1.25)
}

TEST(ValidateMarkers, OtherMarkersActiveAtTheSameTimeCountTowardsTheResidual)
{
    // Both markers are active from 0 to 1 s, so the root shift is the mean of their two horizontal
    // errors and each marker is left with half the difference between them. At (s0, s1), with
    // actor 1 facing actor 0 from 0.3 m away (right foot of actor 0 at (0.09 s0, 0), right hand at
    // (0.18 s0, 0), right foot of actor 1 at (-0.09 s1, 0.3), hand offset fixed in meters):
    //   foot marker error  e1 = (-0.09 (s0 + s1), 0.3)
    //   hand marker error  e2 = (0.27 - 0.09 s1 - 0.18 s0, 0)
    //   leftover (e1 - e2) / 2 = ((0.09 s0 - 0.27) / 2, 0.15), opposite signs for the two markers.
    // The foot marker (tolerance 0.02) has residual hypot(that) and fails everywhere; the hand
    // marker (tolerance 0.165) has residual hypot(that, (s0 - 1) * 0.73) and also fails everywhere
    // here.
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(footMarker(s, 0.0, 1.0));
    markers.markers.push_back(handMarker(s, 0.0, 1.0));
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s));
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 2U);

    const auto leftover = [](double s0) { return std::hypot((0.09 * s0 - 0.27) / 2.0, 0.15); };
    const MarkerValidation& foot = r.markers[0];
    const MarkerValidation& hand = r.markers[1];
    ASSERT_EQ(foot.failures.size(), 9U);
    ASSERT_EQ(hand.failures.size(), 9U);
    double worstFoot = 0.0;
    double worstHand = 0.0;
    for (const FailedPairing& f : foot.failures) {
        EXPECT_NEAR(f.residualM, leftover(f.scale0), 1e-6) << f.scale0 << "/" << f.scale1;
        EXPECT_FALSE(f.clamped);
        worstFoot = std::max(worstFoot, f.residualM);
    }
    for (const FailedPairing& f : hand.failures) {
        EXPECT_NEAR(f.residualM, std::hypot(leftover(f.scale0), (f.scale0 - 1.0) * kHandDz), 1e-6)
            << f.scale0 << "/" << f.scale1;
        EXPECT_FALSE(f.clamped);
        worstHand = std::max(worstHand, f.residualM);
    }
    EXPECT_NEAR(foot.worstResidualM, worstFoot, 1e-9);
    EXPECT_NEAR(hand.worstResidualM, worstHand, 1e-9);
    EXPECT_NEAR(hand.worstResidualM, std::hypot(leftover(1.25), 0.25 * kHandDz), 1e-6);
    // Alone, each of the two is met where its own vertical gap allows (see the tests above).
}

TEST(ValidateMarkers, TheWorstResidualCoversPassingPairingsToo)
{
    // The hand marker's residual at s0 = 0.8 is 0.146 (a pass) and at 1.25 is 0.1825 (a fail); the
    // worst is the larger. With only s0 = 0.8 in the sweep, the marker passes and the worst
    // residual is the 0.146 that was measured.
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(handMarker(s, 0.5, 0.6));
    ValidationSettings settings;
    settings.scales = {0.8};
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s), settings);
    ASSERT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.markers[0].passed());
    EXPECT_NEAR(r.markers[0].worstResidualM, 0.2 * kHandDz, 1e-6);
}

TEST(ValidateMarkers, AMarkerWhoseAlignPairCallIsInvalidFailsWithTheErrorKept)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(footMarker(s, 0.0, 0.1));
    // Actor 1's body map has lost the capsule the marker targets.
    std::array<BodyMapping, 2> maps = mapsOf(s);
    maps[1].capsules.pop_back();
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), maps);
    ASSERT_TRUE(r.valid) << r.error; // the report itself is valid
    ASSERT_EQ(r.markers.size(), 1U);
    const MarkerValidation& v = r.markers[0];
    EXPECT_FALSE(v.passed());
    ASSERT_EQ(v.failures.size(), 9U); // every pairing
    for (const FailedPairing& f : v.failures) {
        EXPECT_FALSE(f.error.empty());
        EXPECT_EQ(f.residualM, 0.0);
        EXPECT_FALSE(f.clamped);
    }
    EXPECT_EQ(v.worstResidualM, 0.0);
    EXPECT_FALSE(r.allPassed());
}

TEST(ValidateMarkers, AMarkerNamingAMissingBoneFailsWithTheErrorKept)
{
    const Scene s = footScene(0.3);
    InteractionMarker m = footMarker(s, 0.0, 0.1);
    m.sourceBone = "NoSuchBone";
    MarkerSet markers;
    markers.markers.push_back(m);
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s));
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 1U);
    ASSERT_EQ(r.markers[0].failures.size(), 9U);
    EXPECT_FALSE(r.markers[0].failures[0].error.empty());
}

TEST(ValidateMarkers, AClampedRootShiftIsAFailureEvenWhenTheMarkerIsMet)
{
    // Actors 1.5 m apart: the horizontal gap between the feet is over 1.5 m, far beyond the 0.5 m
    // root shift limit. A tolerance of 10 m makes the marker "met" even so.
    const Scene s = footScene(1.5);
    InteractionMarker m = footMarker(s, 0.0, 0.1);
    m.toleranceM = 10.0;
    MarkerSet markers;
    markers.markers.push_back(m);
    const ValidationReport r = validateMarkers(markers, repeatedClip(s, 8), mapsOf(s));
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 1U);
    const MarkerValidation& v = r.markers[0];
    EXPECT_FALSE(v.passed());
    ASSERT_EQ(v.failures.size(), 9U);
    for (const FailedPairing& f : v.failures) {
        EXPECT_TRUE(f.clamped);
        EXPECT_TRUE(f.error.empty());
        // Left over: the gap (at least 1.5 m) minus the 0.5 m that was allowed.
        EXPECT_GE(f.residualM, 1.0);
        EXPECT_LT(f.residualM, 10.0); // met by the marker's own tolerance
    }
    EXPECT_GE(v.worstResidualM, 1.0);
}

TEST(ValidateMarkers, TheFrameIsTheRoundedStartTimeClampedToTheClip)
{
    // Frames 0-7 at 10 Hz. Only the listed frame has the actors close; in the others they are
    // 1.5 m apart (root shift clamped, so a failure).
    const Scene good = footScene(0.3);
    const Scene bad = footScene(1.5);
    const std::array<BodyMapping, 2> maps = mapsOf(good);
    const auto clipWithGoodFrame = [&](std::size_t goodFrame) {
        PoseClip clip;
        clip.sampleRateHz = kRateHz;
        for (std::size_t f = 0; f < 8; ++f) {
            const Scene& s = f == goodFrame ? good : bad;
            appendFrame(clip, s.actors[0], s.actors[1]);
        }
        return clip;
    };
    const auto passesAt = [&](double startS, const PoseClip& clip) {
        MarkerSet markers;
        markers.markers.push_back(footMarker(good, startS, startS + 0.1));
        const ValidationReport r = validateMarkers(markers, clip, maps);
        EXPECT_TRUE(r.valid) << r.error;
        return r.valid && r.allPassed();
    };

    const PoseClip frame4 = clipWithGoodFrame(4);
    EXPECT_TRUE(passesAt(0.4, frame4));   // 0.4 * 10 = 4
    EXPECT_TRUE(passesAt(0.36, frame4));  // 3.6 rounds to 4
    EXPECT_TRUE(passesAt(0.44, frame4));  // 4.4 rounds to 4
    EXPECT_FALSE(passesAt(0.34, frame4)); // 3.4 rounds to 3, a bad frame
    EXPECT_FALSE(passesAt(0.46, frame4)); // 4.6 rounds to 5, a bad frame
    EXPECT_FALSE(passesAt(0.0, frame4));

    // Past the end of the clip: the last frame.
    const PoseClip last = clipWithGoodFrame(7);
    EXPECT_TRUE(passesAt(100.0, last));
    EXPECT_TRUE(passesAt(0.7, last));
    EXPECT_FALSE(passesAt(0.5, last));
}

TEST(ValidateMarkers, NoMarkersIsAValidReportThatPasses)
{
    const Scene s = footScene(0.3);
    const ValidationReport r = validateMarkers(MarkerSet{}, repeatedClip(s, 3), mapsOf(s));
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.markers.empty());
    EXPECT_TRUE(r.allPassed());
}

// ---- Invalid settings
// --------------------------------------------------------------------------------------

TEST(ValidateMarkersInvalid, EmptyOrBadScalesGiveAnInvalidReportWithAMessage)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(footMarker(s, 0.0, 0.1));
    const PoseClip clip = repeatedClip(s, 8);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<std::vector<double>> bad = {
        {}, {0.0}, {-1.0}, {nan}, {inf}, {-inf}, {1.0, 0.0}, {1.0, nan}, {0.8, 1.0, -0.5},
    };
    for (const std::vector<double>& scales : bad) {
        SCOPED_TRACE(scales.size());
        ValidationSettings settings;
        settings.scales = scales;
        ValidationReport r;
        EXPECT_NO_THROW(r = validateMarkers(markers, clip, mapsOf(s), settings));
        EXPECT_FALSE(r.valid);
        EXPECT_FALSE(r.error.empty());
        EXPECT_TRUE(r.markers.empty());
        EXPECT_FALSE(r.allPassed() && r.valid);
    }
}

// ---- Determinism
// --------------------------------------------------------------------------------------------

bool sameBits(double a, double b)
{
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

void expectBitIdentical(const ValidationReport& a, const ValidationReport& b)
{
    EXPECT_EQ(a.valid, b.valid);
    EXPECT_EQ(a.error, b.error);
    ASSERT_EQ(a.markers.size(), b.markers.size());
    for (std::size_t i = 0; i < a.markers.size(); ++i) {
        SCOPED_TRACE(a.markers[i].id);
        EXPECT_EQ(a.markers[i].id, b.markers[i].id);
        EXPECT_TRUE(sameBits(a.markers[i].worstResidualM, b.markers[i].worstResidualM));
        ASSERT_EQ(a.markers[i].failures.size(), b.markers[i].failures.size());
        for (std::size_t k = 0; k < a.markers[i].failures.size(); ++k) {
            const FailedPairing& x = a.markers[i].failures[k];
            const FailedPairing& y = b.markers[i].failures[k];
            EXPECT_TRUE(sameBits(x.scale0, y.scale0));
            EXPECT_TRUE(sameBits(x.scale1, y.scale1));
            EXPECT_TRUE(sameBits(x.residualM, y.residualM));
            EXPECT_EQ(x.clamped, y.clamped);
            EXPECT_EQ(x.error, y.error);
        }
    }
}

TEST(ValidateMarkersDeterminism, TheSameInputsGiveBitIdenticalReports)
{
    const Scene s = footScene(0.3);
    MarkerSet markers;
    markers.markers.push_back(footMarker(s, 0.0, 1.0));
    markers.markers.push_back(handMarker(s, 0.0, 1.0));
    const PoseClip clip = repeatedClip(s, 8);
    const ValidationReport first = validateMarkers(markers, clip, mapsOf(s));
    const ValidationReport second = validateMarkers(markers, clip, mapsOf(s));
    ASSERT_TRUE(first.valid) << first.error;
    expectBitIdentical(first, second);

    // A call with other inputs in between changes nothing.
    (void)validateMarkers(MarkerSet{}, clip, mapsOf(s));
    expectBitIdentical(first, validateMarkers(markers, clip, mapsOf(s)));
}

TEST(ValidateMarkersDeterminism, InvalidAndClampedReportsAreAlsoStable)
{
    const Scene s = footScene(1.5);
    MarkerSet markers;
    markers.markers.push_back(footMarker(s, 0.0, 0.1));
    std::array<BodyMapping, 2> broken = mapsOf(s);
    broken[0].capsules.clear();
    broken[1].capsules.clear();
    expectBitIdentical(validateMarkers(markers, repeatedClip(s, 8), mapsOf(s)),
                       validateMarkers(markers, repeatedClip(s, 8), mapsOf(s)));
    expectBitIdentical(validateMarkers(markers, repeatedClip(s, 8), broken),
                       validateMarkers(markers, repeatedClip(s, 8), broken));
}

} // namespace
