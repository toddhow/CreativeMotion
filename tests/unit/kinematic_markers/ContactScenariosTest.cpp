// Contact detection on clips of the synthetic humanoid (A4a): a held hand, a brush, a fast slide,
// no contact, and the feedback criterion (detected markers, fed back to alignPair on the frame at
// window.startS, are met without a root shift).
//
// Setup: two humanoids (scale 1, SyntheticHumanoid.hpp) both facing +Y, actor 1 standing 0.3 m in
// front of actor 0. Actor 0's right arm is posed frame by frame so that its hand follows a list of
// world points (poseArmToward, the A1 two-bone IK); everything else is static. Clip rate 30 Hz,
// default detection settings (threshold 0.03 m, minimum duration 0.25 s, maximum speed 0.2 m/s).

#include "AlignmentScenes.hpp"
#include "ClipBuilder.hpp"
#include "SyntheticHumanoid.hpp"
#include "TestSupport.hpp"
#include "opas/markers/Alignment.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/markers/ContactDetection.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

using opas::markers::AlignmentResult;
using opas::markers::BodyMapping;
using opas::markers::Confidence;
using opas::markers::detectContacts;
using opas::markers::DetectedContact;
using opas::markers::DetectionResult;
using opas::markers::DetectionSettings;
using opas::markers::PoseClip;
using opas::math::conjugate;
using opas::math::distance;
using opas::math::dot;
using opas::math::length;
using opas::math::normalized;
using opas::math::rotate;
using opas::math::Vec3;
using opas::test::appendFrame;
using opas::test::capsuleAxisPoint;
using opas::test::capsuleRadiusM;
using opas::test::localPoseFromWorld;
using opas::test::makeSceneActor;
using opas::test::poseArmToward;
using opas::test::regionFrame;
using opas::test::SceneActor;
using opas::test::standingAt;

constexpr double kRate = 30.0;
constexpr double kSeparationM = 0.3;
constexpr double kTouchDistanceM = 0.008; // how far the hand is from the surface when touching

struct Scenario {
    PoseClip clip;
    SceneActor actor0;
    SceneActor actor1;

    [[nodiscard]] std::array<BodyMapping, 2> maps() const
    {
        return {actor0.body.mapping, actor1.body.mapping};
    }
};

Scenario makeScenario(const std::vector<Vec3>& rightHandTargets, double separationM = kSeparationM)
{
    Scenario s;
    s.clip.sampleRateHz = kRate;
    s.actor0 = makeSceneActor(1.0, standingAt(0.0, 0.0, false));
    s.actor1 = makeSceneActor(1.0, standingAt(0.0, separationM, false));
    for (const Vec3& target : rightHandTargets) {
        SceneActor posed = s.actor0;
        poseArmToward(posed, 1.0, target);
        appendFrame(s.clip, posed, s.actor1);
    }
    return s;
}

// The touch point on the back of actor 1's left shoulder, chosen (by a search over the capsule
// surface) to be at least 0.04 m from every other capsule of the body, so that only one region is
// ever within the 0.03 m threshold: 0.008 m off the shoulder capsule's surface, at 65 % along its
// axis, in the direction (0, -cos 0.07, sin 0.07) (mostly backward, i.e. towards actor 0).
Vec3 shoulderTouchPoint(const SceneActor& target)
{
    const double elevation = 0.07;
    const Vec3 direction{0.0, -std::cos(elevation), std::sin(elevation)};
    return capsuleAxisPoint(target, "LeftShoulder", 0.65) +
           direction * (capsuleRadiusM(target, "LeftShoulder") + kTouchDistanceM);
}

// Hand targets: `approach` frames moving in from behind in 0.03 m steps, `hold` frames on the
// touch point, then `release` frames moving back out.
std::vector<Vec3> touchSequence(Vec3 touch, std::size_t approach, std::size_t hold,
                                std::size_t release)
{
    std::vector<Vec3> targets;
    targets.reserve(approach + hold + release);
    for (std::size_t k = 0; k < approach; ++k) {
        targets.push_back(touch + Vec3{0.0, -0.03 * static_cast<double>(approach - k), 0.0});
    }
    for (std::size_t k = 0; k < hold; ++k) {
        targets.push_back(touch);
    }
    for (std::size_t k = 1; k <= release; ++k) {
        targets.push_back(touch + Vec3{0.0, -0.03 * static_cast<double>(k), 0.0});
    }
    return targets;
}

DetectionResult detect(const Scenario& s, const DetectionSettings& settings = {})
{
    return detectContacts(s.clip, s.maps(), settings);
}

// Solves the alignment for the clip's frame at `contact`'s window start, from only what the clip
// records: world transforms. The clip's bones are the skeleton's bones in order, so each actor's
// local pose is rebuilt from the world transforms with an identity root (localPoseFromWorld).
AlignmentResult alignAtStart(const Scenario& s, const DetectedContact& contact)
{
    const double startS = contact.marker.window.startS;
    const auto frame = static_cast<std::size_t>(std::llround(startS * s.clip.sampleRateHz));
    const std::vector<opas::math::Transform> local0 =
        localPoseFromWorld(s.actor0.body.skeleton, s.clip.actors[0].frames[frame]);
    const std::vector<opas::math::Transform> local1 =
        localPoseFromWorld(s.actor1.body.skeleton, s.clip.actors[1].frames[frame]);
    const opas::markers::ActorInput in0{s.actor0.body.skeleton, s.actor0.body.mapping, local0,
                                        opas::math::Transform{}, 1.0};
    const opas::markers::ActorInput in1{s.actor1.body.skeleton, s.actor1.body.mapping, local1,
                                        opas::math::Transform{}, 1.0};
    opas::markers::MarkerSet markers;
    markers.markers.push_back(contact.marker);
    return opas::markers::alignPair(in0, in1, markers, startS);
}

void expectFeedbackMet(const Scenario& s, const DetectedContact& contact,
                       const DetectionSettings& settings)
{
    const AlignmentResult r = alignAtStart(s, contact);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.markers.size(), 1U);
    EXPECT_EQ(r.markers[0].id, contact.marker.id);
    EXPECT_TRUE(r.markers[0].met) << "residual " << r.markers[0].distanceM;
    EXPECT_LE(r.markers[0].distanceM, settings.markerToleranceM);
    // No root shift larger than 1 mm.
    EXPECT_LE(length(r.rootShiftM[0]), 1e-3);
    EXPECT_LE(length(r.rootShiftM[1]), 1e-3);
    EXPECT_FALSE(r.rootShiftClamped);
}

// ---- Held hand ---------------------------------------------------------------------------------

// 5 approach frames, 31 hold frames (frames 5..35, one second plus one sample), 5 release frames.
Scenario heldScenario()
{
    const SceneActor probe = makeSceneActor(1.0, standingAt(0.0, kSeparationM, false));
    return makeScenario(touchSequence(shoulderTouchPoint(probe), 5, 31, 5));
}

TEST(ContactScenarioHeld, ASecondOnAShoulderIsOneHighContactOnTheRightBoneAndRegion)
{
    const Scenario s = heldScenario();
    ASSERT_EQ(s.clip.frameCount(), 41U);
    const DetectionResult r = detect(s);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 1U);
    const DetectedContact& c = r.contacts[0];
    EXPECT_EQ(c.marker.sourceActor, 0);
    EXPECT_EQ(c.marker.targetActor, 1);
    EXPECT_EQ(c.marker.sourceBone, "RightHand");
    EXPECT_EQ(c.marker.targetRegion, "LeftShoulder");
    EXPECT_EQ(c.marker.id, "a0-RightHand-on-a1-LeftShoulder-0");
    EXPECT_EQ(c.confidence, Confidence::High);
    EXPECT_EQ(c.marker.priority, 2);
    EXPECT_EQ(c.marker.toleranceM, 0.02);
}

TEST(ContactScenarioHeld, TheWindowIsWithinOneSampleOfTheHold)
{
    const DetectionResult r = detect(heldScenario());
    ASSERT_EQ(r.contacts.size(), 1U);
    const DetectedContact& c = r.contacts[0];
    EXPECT_NEAR(c.marker.window.startS, 5.0 / kRate, 1.0 / kRate);
    EXPECT_NEAR(c.marker.window.endS, 35.0 / kRate, 1.0 / kRate);
    EXPECT_NEAR(c.durationS, 31.0 / kRate, 1.0 / kRate);
    EXPECT_GE(c.durationS, 1.0); // held for a second
    EXPECT_NEAR(c.meanDistanceM, kTouchDistanceM, 1e-3);
    EXPECT_LT(c.meanSpeedMps, 0.2);
}

TEST(ContactScenarioHeld,
     TheOffsetIsWithinOneCentimeterOfTheTruthAndTheDirectionPointsIntoTheSurface)
{
    const Scenario s = heldScenario();
    const DetectionResult r = detect(s);
    ASSERT_EQ(r.contacts.size(), 1U);
    const DetectedContact& c = r.contacts[0];

    const Vec3 touch = shoulderTouchPoint(s.actor1);
    const opas::math::Transform frame = regionFrame(s.actor1, "LeftShoulder");
    const Vec3 truth = opas::math::transformPoint(opas::math::inverse(frame), touch);
    EXPECT_LE(distance(c.marker.offsetM, truth), 0.01);

    // Into the surface: from the hand towards the closest point of the shoulder's axis, in the
    // region frame.
    const Vec3 axisStart = opas::test::bonePosition(s.actor1, "Chest");
    const Vec3 axisEnd = opas::test::bonePosition(s.actor1, "LeftUpperArm");
    const Vec3 axis = axisEnd - axisStart;
    const double t = std::clamp(dot(touch - axisStart, axis) / dot(axis, axis), 0.0, 1.0);
    const Vec3 closest = axisStart + axis * t;
    const Vec3 inward = rotate(conjugate(frame.rotation), normalized(closest - touch));
    EXPECT_GE(dot(c.marker.contactDirection, inward), 0.99);
    EXPECT_NEAR(length(c.marker.contactDirection), 1.0, 1e-9);
}

TEST(ContactScenarioHeld, ALowConfidenceProfileChangesNothingForAHighContact)
{
    DetectionSettings low;
    low.includeLowConfidence = true;
    const DetectionResult r = detect(heldScenario(), low);
    ASSERT_EQ(r.contacts.size(), 1U);
    EXPECT_EQ(r.contacts[0].confidence, Confidence::High);
}

TEST(ContactScenarioHeld, TheDetectedMarkerIsMetWhenFedBackToTheSolver)
{
    const Scenario s = heldScenario();
    const DetectionResult r = detect(s);
    ASSERT_EQ(r.contacts.size(), 1U);
    expectFeedbackMet(s, r.contacts[0], DetectionSettings{});
}

// ---- A brush
// --------------------------------------------------------------------------------------

// The same touch for 7 frames (7 / 30 = 0.233 s, shorter than the 0.25 s minimum).
Scenario brushScenario()
{
    const SceneActor probe = makeSceneActor(1.0, standingAt(0.0, kSeparationM, false));
    return makeScenario(touchSequence(shoulderTouchPoint(probe), 5, 7, 5));
}

TEST(ContactScenarioBrush, ATouchShorterThanTheMinimumDurationIsNotHigh)
{
    const Scenario s = brushScenario();
    DetectionSettings low;
    low.includeLowConfidence = true;
    const DetectionResult r = detect(s, low);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 1U);
    const DetectedContact& c = r.contacts[0];
    EXPECT_LT(c.durationS, 0.25);
    EXPECT_NEAR(c.durationS, 7.0 / kRate, 1e-9);
    EXPECT_NE(c.confidence, Confidence::High);
    // Close (0.008 <= 0.015) and still (the mean over the 7 frames is (0.45 + 0.45) / 7 = 0.129
    // m/s, from the two frames beside the approach and release, where the hand moved 0.03 m per two
    // frames), but not long: two tests, so Medium.
    EXPECT_EQ(c.confidence, Confidence::Medium);
    EXPECT_EQ(c.marker.priority, 1);
    EXPECT_NEAR(c.meanSpeedMps, 0.9 / 7.0, 1e-3);
}

TEST(ContactScenarioBrush, AMediumBrushIsKeptWithTheDefaultProfile)
{
    const DetectionResult r = detect(brushScenario());
    ASSERT_EQ(r.contacts.size(), 1U);
    EXPECT_EQ(r.contacts[0].confidence, Confidence::Medium);
}

TEST(ContactScenarioBrush, TheBrushMarkerIsMetWhenFedBackToTheSolver)
{
    const Scenario s = brushScenario();
    const DetectionResult r = detect(s);
    ASSERT_EQ(r.contacts.size(), 1U);
    expectFeedbackMet(s, r.contacts[0], DetectionSettings{});
}

// ---- A fast slide
// ----------------------------------------------------------------------------------

// The right hand slides up the back of actor 1 (the upper-back capsule) at 0.6 m/s for 15 frames:
// 0.02 m per frame at 30 Hz, from z = 1.26 to 1.54, 0.008 m off the surface all the way.
Scenario slideScenario()
{
    const SceneActor probe = makeSceneActor(1.0, standingAt(0.0, kSeparationM, false));
    const double radius = capsuleRadiusM(probe, "UpperBack");
    std::vector<Vec3> targets;
    targets.reserve(15);
    for (int k = 0; k < 15; ++k) {
        targets.push_back(Vec3{0.0, kSeparationM - (radius + kTouchDistanceM), 1.26 + 0.02 * k});
    }
    return makeScenario(targets);
}

TEST(ContactScenarioSlide, AHandSlidingFasterThanTheMaximumSpeedIsNotHigh)
{
    const Scenario s = slideScenario();
    DetectionSettings low;
    low.includeLowConfidence = true;
    const DetectionResult r = detect(s, low);
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 1U);
    const DetectedContact& c = r.contacts[0];
    EXPECT_EQ(c.marker.targetRegion, "UpperBack");
    EXPECT_NEAR(c.meanSpeedMps, 0.6, 1e-3);
    EXPECT_GT(c.meanSpeedMps, 0.2);
    EXPECT_NE(c.confidence, Confidence::High);
    // Close and long (15 / 30 = 0.5 s), not still: Medium.
    EXPECT_EQ(c.confidence, Confidence::Medium);
    EXPECT_NEAR(c.durationS, 0.5, 1e-9);
}

TEST(ContactScenarioSlide, TheSlideMarkerIsMetWhenFedBackToTheSolver)
{
    const Scenario s = slideScenario();
    const DetectionResult r = detect(s);
    ASSERT_EQ(r.contacts.size(), 1U);
    expectFeedbackMet(s, r.contacts[0], DetectionSettings{});
}

// ---- No contact
// -------------------------------------------------------------------------------------

TEST(ContactScenarioNone, TwoActorsStandingApartHaveNoContacts)
{
    Scenario s;
    s.clip.sampleRateHz = kRate;
    s.actor0 = makeSceneActor(1.0, standingAt(0.0, 0.0, false));
    s.actor1 = makeSceneActor(1.0, standingAt(0.0, 0.8, false));
    for (int f = 0; f < 20; ++f) {
        appendFrame(s.clip, s.actor0, s.actor1);
    }
    DetectionSettings low;
    low.includeLowConfidence = true;
    const DetectionResult r = detect(s, low);
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.contacts.empty());
    EXPECT_TRUE(r.markerSet().markers.empty());
}

TEST(ContactScenarioNone, ARaisedHandThatNeverComesWithinTheThresholdHasNoContact)
{
    // The hand passes 0.05 m behind the shoulder: 0.008 + 0.05 from the surface.
    const SceneActor probe = makeSceneActor(1.0, standingAt(0.0, kSeparationM, false));
    const Vec3 near = shoulderTouchPoint(probe) + Vec3{0.0, -0.05, 0.0};
    const Scenario s = makeScenario(std::vector<Vec3>(20, near));
    DetectionSettings low;
    low.includeLowConfidence = true;
    const DetectionResult r = detect(s, low);
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.contacts.empty());
}

} // namespace
