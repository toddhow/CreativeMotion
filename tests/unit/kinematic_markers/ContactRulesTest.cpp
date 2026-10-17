// Hand-computed cases for every numbered rule of contact detection
// (docs/tasks/0006-a4a-contact-detection.md). Most cases use the mini rig of ClipBuilder.hpp: a
// post (the segment (0,0,0)-(0,0,1), radius 0.1 m) on actor 1 and a hand of actor 0 that the test
// places frame by frame.

#include "ClipBuilder.hpp"
#include "TestSupport.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/markers/ContactDetection.hpp"
#include "opas/markers/InteractionMarker.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace {

using opas::markers::BodyMapping;
using opas::markers::Confidence;
using opas::markers::detectContacts;
using opas::markers::DetectedContact;
using opas::markers::DetectionResult;
using opas::markers::DetectionSettings;
using opas::math::fromAxisAngle;
using opas::math::length;
using opas::math::Quat;
using opas::math::Transform;
using opas::math::Vec3;
using opas::test::contains;
using opas::test::kPi;
using opas::test::miniBodyMaps;
using opas::test::MiniClip;
using opas::test::MiniNames;
using opas::test::Vec3Near;

constexpr double kTol = 1e-9;

DetectionResult detect(const MiniClip& c, const DetectionSettings& s = {},
                       const std::array<BodyMapping, 2>& maps = miniBodyMaps())
{
    return detectContacts(c.clip(), maps, s);
}

DetectionSettings withLow()
{
    DetectionSettings s;
    s.includeLowConfidence = true;
    return s;
}

// The mini maps without the Rod capsule, for cases whose numbers must not touch a second region.
std::array<BodyMapping, 2> postOnlyMaps(double ratio = 0.1)
{
    std::array<BodyMapping, 2> maps = miniBodyMaps(ratio);
    maps[0].capsules.pop_back();
    maps[1].capsules.pop_back();
    return maps;
}

// The only contact of a result, or nullptr (after recording a failure).
std::shared_ptr<const DetectedContact> onlyContact(const DetectionResult& r)
{
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_EQ(r.contacts.size(), 1U);
    if (r.contacts.size() != 1) {
        return nullptr;
    }
    return std::make_shared<const DetectedContact>(
        r.contacts[0]); // a copy: callers pass temporaries
}

std::vector<Vec3> repeated(Vec3 p, std::size_t n)
{
    return std::vector<Vec3>(n, p);
}

// ---- Rule 1: candidates
// -----------------------------------------------------------------------------

TEST(ContactCandidates, OnlyTheLastBoneOfEachLimbIsASource)
{
    // The elbow touches the post for the whole clip; the hand (the limb's last bone) is far away.
    MiniClip c(10, 4);
    c.hand0(repeated({0.12, 0.0, 0.5}, 4), "Elbow");
    const DetectionResult r = detect(c, withLow());
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.contacts.empty());
}

TEST(ContactCandidates, EveryLimbEndIsASource)
{
    MiniClip c(10, 4);
    c.hand0(repeated({0.12, 0.0, 0.5}, 4), "Hand2");
    const DetectionResult r = detect(c, withLow());
    const auto contact = onlyContact(r);
    ASSERT_NE(contact, nullptr);
    EXPECT_EQ(contact->marker.sourceBone, "Hand2");

    // Both limb ends touching give two contacts, one per source bone.
    MiniClip both(10, 4);
    both.hand0(repeated({0.12, 0.0, 0.5}, 4), "Hand");
    both.hand0(repeated({-0.12, 0.0, 0.5}, 4), "Hand2");
    const DetectionResult two = detect(both, withLow());
    ASSERT_TRUE(two.valid) << two.error;
    ASSERT_EQ(two.contacts.size(), 2U);
    EXPECT_EQ(two.contacts[0].marker.sourceBone, "Hand");
    EXPECT_EQ(two.contacts[1].marker.sourceBone, "Hand2");
}

TEST(ContactCandidates, BothActorsAreSourcesAndTargets)
{
    MiniClip c(10, 4);
    c.hand0(repeated({0.12, 0.0, 0.5}, 4));      // actor 0's hand on actor 1's post
    c.set(1, "Hand", 0, Vec3{100.12, 0.0, 0.5}); // actor 1's hand on actor 0's post
    c.set(1, "Hand", 1, Vec3{100.12, 0.0, 0.5});
    c.set(1, "Hand", 2, Vec3{100.12, 0.0, 0.5});
    c.set(1, "Hand", 3, Vec3{100.12, 0.0, 0.5});
    const DetectionResult r = detect(c, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 2U);
    // Same window: ordered by id, "a0-..." before "a1-...".
    EXPECT_EQ(r.contacts[0].marker.sourceActor, 0);
    EXPECT_EQ(r.contacts[0].marker.targetActor, 1);
    EXPECT_EQ(r.contacts[1].marker.sourceActor, 1);
    EXPECT_EQ(r.contacts[1].marker.targetActor, 0);
    EXPECT_EQ(r.contacts[0].marker.targetRegion, "Post");
    EXPECT_EQ(r.contacts[1].marker.targetRegion, "Post");
}

TEST(ContactCandidates, EveryCapsuleOfTheTargetIsARegion)
{
    // Near the base of the post, 0.05 above the Rod (the x axis): the hand is inside the Rod and
    // 0.02 from the surface of the post. Two regions, two contacts.
    MiniClip c(10, 4);
    c.hand0(repeated({0.12, 0.0, 0.05}, 4));
    const DetectionResult r = detect(c, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 2U);
    EXPECT_EQ(r.contacts[0].marker.targetRegion, "Post");
    EXPECT_EQ(r.contacts[1].marker.targetRegion, "Rod");
    EXPECT_NEAR(r.contacts[0].meanDistanceM, 0.02, kTol);  // |p - (0,0,0.05)| - 0.1
    EXPECT_NEAR(r.contacts[1].meanDistanceM, -0.05, kTol); // |p - (0.12,0,0)| - 0.1
}

TEST(ContactCandidates, AHandTouchingOnlyOneRegionGivesOneContact)
{
    MiniClip c(10, 4);
    c.hand0(repeated({0.5, 0.12, 0.0}, 4)); // 0.02 from the Rod, 0.51 from the post
    const DetectionResult r = detect(c, withLow());
    const auto contact = onlyContact(r);
    ASSERT_NE(contact, nullptr);
    EXPECT_EQ(contact->marker.targetRegion, "Rod");
}

TEST(ContactCandidates, AnActorsOwnCapsulesAreNotTargets)
{
    // Actor 0's hand placed on actor 0's own post (100 m away in X): not a candidate.
    MiniClip c(10, 4);
    c.hand0(repeated({100.12, 0.0, 0.5}, 4));
    const DetectionResult r = detect(c, withLow());
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.contacts.empty());
}

TEST(ContactCandidates, ABodyMapWithoutLimbsHasNoSources)
{
    std::array<BodyMapping, 2> maps = miniBodyMaps();
    maps[0].limbs.clear();
    MiniClip c(10, 4);
    c.hand0(repeated({0.12, 0.0, 0.5}, 4));
    const DetectionResult r = detect(c, withLow(), maps);
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.contacts.empty());
}

TEST(ContactCandidates, ABodyMapWithoutCapsulesHasNoTargets)
{
    std::array<BodyMapping, 2> maps = miniBodyMaps();
    maps[1].capsules.clear();
    MiniClip c(10, 4);
    c.hand0(repeated({0.12, 0.0, 0.5}, 4));
    const DetectionResult r = detect(c, withLow(), maps);
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.contacts.empty());
}

TEST(ContactCandidates, ABoneThatACandidateNeedsMustBeInTheClip)
{
    const struct {
        std::size_t actor;
        const char* bone;
    } missing[] = {
        {0, "Hand"},  // a source bone of actor 0
        {1, "Hand"},  // a source bone of actor 1
        {0, "Hand2"}, // the end of a second limb
        {1, "Tip"},   // a capsule end bone of the target of actor 0's hand
        {0, "Tip"},   // a capsule end bone of the target of actor 1's hand
        {1, "Base"},  // a capsule start bone
        {0, "Side"},  // the second capsule's end bone
    };
    for (const auto& m : missing) {
        SCOPED_TRACE(std::string("actor ") + std::to_string(m.actor) + " lacks " + m.bone);
        MiniClip c(10, 3);
        c.removeBone(m.actor, m.bone);
        DetectionResult r;
        EXPECT_NO_THROW(r = detect(c, withLow()));
        EXPECT_FALSE(r.valid);
        EXPECT_TRUE(r.contacts.empty());
        EXPECT_TRUE(contains(r.error, m.bone)) << r.error;
        EXPECT_TRUE(contains(r.error, std::to_string(m.actor))) << r.error;
    }
}

TEST(ContactCandidates, BonesNoCandidateNeedsMayBeMissingOrExtra)
{
    // "Elbow" is a middle bone of a limb: no candidate uses it.
    MiniClip c(10, 4);
    c.removeBone(0, "Elbow");
    c.removeBone(1, "Elbow");
    EXPECT_TRUE(detect(c, withLow()).valid);

    MiniClip extra(10, 2);
    extra.clip().actors[0].bones.push_back("Unused");
    for (auto& frame : extra.clip().actors[0].frames) {
        frame.push_back(Transform{});
    }
    EXPECT_TRUE(detect(extra, withLow()).valid);
}

// ---- Rule 2: surface distance and contact frames
// -------------------------------------------------------

TEST(ContactSurface, OutsideTheCapsuleSideDistanceIsCentreDistanceMinusRadius)
{
    MiniClip c(10, 3);
    c.hand0(repeated({0.12, 0.0, 0.5}, 3)); // |p - c| = 0.12, r = 0.1
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->meanDistanceM, 0.02, kTol);
}

TEST(ContactSurface, InsideTheCapsuleTheDistanceIsNegative)
{
    MiniClip c(10, 3);
    c.hand0(repeated({0.05, 0.0, 0.5}, 3)); // 0.05 from the axis: d = -0.05
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->meanDistanceM, -0.05, kTol);

    MiniClip onAxis(10, 3);
    onAxis.hand0(repeated({0.0, 0.0, 0.5}, 3)); // d = -0.1
    const auto axis = onlyContact(detect(onAxis, withLow()));
    ASSERT_NE(axis, nullptr);
    EXPECT_NEAR(axis->meanDistanceM, -0.1, kTol);
}

TEST(ContactSurface, BeyondTheThresholdIsNoContact)
{
    MiniClip c(10, 3);
    c.hand0(repeated({0.14, 0.0, 0.5}, 3)); // d = 0.04 > 0.03
    const DetectionResult r = detect(c, withLow());
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.contacts.empty());
}

TEST(ContactSurface, PastTheEndsTheClosestPointIsTheEndPoint)
{
    // Above the tip: closest point (0,0,1), distance 0.12, d = 0.02.
    MiniClip above(10, 3);
    above.hand0(repeated({0.0, 0.0, 1.12}, 3));
    const auto a = onlyContact(detect(above, withLow()));
    ASSERT_NE(a, nullptr);
    EXPECT_NEAR(a->meanDistanceM, 0.02, kTol);

    // Below the base: closest point (0,0,0); 0.2 away, d = 0.1: no contact (and the Rod, an x-axis
    // segment, is 0.2 away too).
    MiniClip below(10, 3);
    below.hand0(repeated({0.0, 0.0, -0.2}, 3));
    EXPECT_TRUE(detect(below, withLow()).contacts.empty());

    // Diagonal past the tip: (0.06, 0, 1.08) is exactly 0.1 from (0,0,1): d = 0.
    MiniClip diagonal(10, 3);
    diagonal.hand0(repeated({0.06, 0.0, 1.08}, 3));
    const auto d = onlyContact(detect(diagonal, withLow()));
    ASSERT_NE(d, nullptr);
    EXPECT_NEAR(d->meanDistanceM, 0.0, kTol);
}

TEST(ContactSurface, TheThresholdIsInclusive)
{
    // Binary-exact numbers: ratio 0.5 gives radius 0.5 on the unit post; a hand at x = 1 is
    // exactly 0.5 from the surface.
    DetectionSettings s = withLow();
    s.distanceThresholdM = 0.5;
    const auto maps = postOnlyMaps(0.5);
    MiniClip on(8, 2);
    on.hand0(repeated({1.0, 0.0, 0.5}, 2));
    const DetectionResult hit = detect(on, s, maps);
    ASSERT_TRUE(hit.valid) << hit.error;
    EXPECT_EQ(hit.contacts.size(), 1U); // d == threshold counts

    MiniClip off(8, 2);
    off.hand0(repeated({1.000001, 0.0, 0.5}, 2));
    EXPECT_TRUE(detect(off, s, maps).contacts.empty()); // just beyond does not
}

TEST(ContactSurface, TheRadiusIsRatioTimesTheAxisLengthInThatFrame)
{
    // The tip rises from z = 1 (radius 0.1) to z = 2 (radius 0.2) in frame 1. A hand at x = 0.215
    // is 0.115 from the surface in frame 0 (no contact) and 0.015 in frame 1 (contact).
    MiniClip c(10, 3);
    c.hand0(repeated({0.215, 0.0, 0.5}, 3));
    c.set(1, "Tip", 1, Vec3{0.0, 0.0, 2.0});
    c.set(1, "Tip", 2, Vec3{0.0, 0.0, 2.0});
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->durationS, 0.2, kTol); // frames 1 and 2
    EXPECT_NEAR(contact->marker.window.startS, 0.1, kTol);
    EXPECT_NEAR(contact->meanDistanceM, 0.015, kTol); // 0.215 - 0.2
}

TEST(ContactSurface, MeanDistanceIsTheMeanOverTheRun)
{
    MiniClip c(10, 3);
    c.hand0({{0.11, 0.0, 0.5}, {0.12, 0.0, 0.5}, {0.125, 0.0, 0.5}}); // d = 0.01, 0.02, 0.025
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->meanDistanceM, (0.01 + 0.02 + 0.025) / 3.0, 1e-9);
}

// ---- Rule 3: region frame, offset
// ------------------------------------------------------------------------

TEST(ContactRegionFrame, OffsetIsThePointInTheRegionsBoneFrame)
{
    // Post base at world (1,2,0) turned 90 degrees about Z (tip at (1,2,1)). A hand at world
    // (1, 2.12, 0.5): minus the base is (0, 0.12, 0.5); Rz(-90) turns (x,y,z) into (y,-x,z), so
    // the point in the region frame is (0.12, 0, 0.5).
    const Quat rz = fromAxisAngle(Vec3{0, 0, 1}, kPi / 2.0);
    MiniClip c(10, 3);
    c.set(1, "Base", 0, Transform{Vec3{1, 2, 0}, rz});
    c.set(1, "Base", 1, Transform{Vec3{1, 2, 0}, rz});
    c.set(1, "Base", 2, Transform{Vec3{1, 2, 0}, rz});
    c.setAll(1, "Tip", Transform{Vec3{1, 2, 1}, Quat{}});
    c.hand0(repeated({1.0, 2.12, 0.5}, 3));
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_PRED_FORMAT3(Vec3Near, contact->marker.offsetM, (Vec3{0.12, 0.0, 0.5}), kTol);
    // n (from the axis to the hand) is world +Y, which is +X in the region frame: the direction
    // into the surface is -X.
    EXPECT_PRED_FORMAT3(Vec3Near, contact->marker.contactDirection, (Vec3{-1.0, 0.0, 0.0}), kTol);
    EXPECT_NEAR(contact->meanDistanceM, 0.02, kTol);
}

TEST(ContactRegionFrame, OffsetIsTheMeanOfThePointsOverTheRun)
{
    MiniClip c(10, 4);
    c.hand0({{0.11, 0.0, 0.4}, {0.11, 0.0, 0.5}, {0.11, 0.0, 0.6}, {0.11, 0.0, 0.7}});
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_PRED_FORMAT3(Vec3Near, contact->marker.offsetM, (Vec3{0.11, 0.0, 0.55}), kTol);
}

// ---- Rule 4: relative speed
// ----------------------------------------------------------------------------------

TEST(ContactSpeed, CentralDifferenceInTheMiddleOneSidedAtTheEnds)
{
    // Rate 10. z = 0.50, 0.51, 0.53, 0.56 at x = 0.12:
    //   f0 one-sided  |0.51 - 0.50| * 10         = 0.10
    //   f1 central    |0.53 - 0.50| / (2 / 10)    = 0.15
    //   f2 central    |0.56 - 0.51| / (2 / 10)    = 0.25
    //   f3 one-sided  |0.56 - 0.53| * 10         = 0.30       mean 0.20
    MiniClip c(10, 4);
    c.hand0({{0.12, 0.0, 0.50}, {0.12, 0.0, 0.51}, {0.12, 0.0, 0.53}, {0.12, 0.0, 0.56}});
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->meanSpeedMps, 0.20, 1e-9);
}

TEST(ContactSpeed, TheSampleRateScalesTheSpeed)
{
    // The same positions at 20 Hz are twice as fast.
    MiniClip c(20, 4);
    c.hand0({{0.12, 0.0, 0.50}, {0.12, 0.0, 0.51}, {0.12, 0.0, 0.53}, {0.12, 0.0, 0.56}});
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->meanSpeedMps, 0.40, 1e-9);
}

TEST(ContactSpeed, AClipOfOneFrameHasSpeedZero)
{
    MiniClip c(10, 1);
    c.hand0({{0.12, 0.0, 0.5}});
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_EQ(contact->meanSpeedMps, 0.0);
    EXPECT_NEAR(contact->durationS, 0.1, kTol); // one frame lasts 1 / rate
    EXPECT_NEAR(contact->marker.window.startS, 0.0, kTol);
    EXPECT_NEAR(contact->marker.window.endS, 0.0, kTol);
}

TEST(ContactSpeed, TwoFramesUseOneSidedDifferencesAtBothEnds)
{
    MiniClip c(10, 2);
    c.hand0({{0.12, 0.0, 0.40}, {0.12, 0.0, 0.45}}); // 0.05 per 0.1 s = 0.5 m/s at both frames
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->meanSpeedMps, 0.5, 1e-9);
}

TEST(ContactSpeed, SpeedUsesNeighbouringFramesEvenOutsideTheRun)
{
    // x = 1.0 (far), 0.12 (contact), 1.0 (far): the one contact frame's central difference is
    // |l2 - l0| / 0.2 = 0, although the hand came from and went back to far away.
    MiniClip still(10, 3);
    still.hand0({{1.0, 0.0, 0.5}, {0.12, 0.0, 0.5}, {1.0, 0.0, 0.5}});
    const auto s = onlyContact(detect(still, withLow()));
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->meanSpeedMps, 0.0);

    // x = 0.12, 0.12, 1.0: frame 0 is one-sided (0), frame 1 central |1.0 - 0.12| / 0.2 = 4.4;
    // mean over the two contact frames 2.2.
    MiniClip leaving(10, 3);
    leaving.hand0({{0.12, 0.0, 0.5}, {0.12, 0.0, 0.5}, {1.0, 0.0, 0.5}});
    const auto l = onlyContact(detect(leaving, withLow()));
    ASSERT_NE(l, nullptr);
    EXPECT_NEAR(l->meanSpeedMps, 2.2, 1e-9);
}

TEST(ContactSpeed, SpeedIsRelativeToTheRegionFrameNotTheWorld)
{
    // The whole post and the hand move together at 1 m/s along X: no relative motion.
    MiniClip c(10, 4);
    for (std::size_t f = 0; f < 4; ++f) {
        const double shift = 0.1 * static_cast<double>(f);
        c.set(1, "Base", f, Vec3{shift, 0.0, 0.0});
        c.set(1, "Tip", f, Vec3{shift, 0.0, 1.0});
        c.set(1, "Side", f, Vec3{1.0 + shift, 0.0, 0.0});
        c.set(0, "Hand", f, Vec3{0.12 + shift, 0.0, 0.5});
    }
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_NEAR(contact->meanSpeedMps, 0.0, 1e-9);
}

// ---- Rule 5: runs
// ------------------------------------------------------------------------------------------------

TEST(ContactRuns, ContactFramesSeparatedByAGapSplitIntoRuns)
{
    // Contact (C) at x = 0.12, none (N) at x = 0.5: C C N C C C. Rate 10.
    const Vec3 c0{0.12, 0.0, 0.5};
    const Vec3 n0{0.5, 0.0, 0.5};
    MiniClip c(10, 6);
    c.hand0({c0, c0, n0, c0, c0, c0});
    const DetectionResult r = detect(c, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 2U);
    // Run 1: frames 0-1, start 0, end 1/10, duration 2/10.
    EXPECT_NEAR(r.contacts[0].marker.window.startS, 0.0, kTol);
    EXPECT_NEAR(r.contacts[0].marker.window.endS, 0.1, kTol);
    EXPECT_NEAR(r.contacts[0].durationS, 0.2, kTol);
    EXPECT_EQ(r.contacts[0].marker.id, "a0-Hand-on-a1-Post-0");
    // Run 2: frames 3-5 (up to the last frame of the clip), start 0.3, end 0.5, duration 0.3.
    EXPECT_NEAR(r.contacts[1].marker.window.startS, 0.3, kTol);
    EXPECT_NEAR(r.contacts[1].marker.window.endS, 0.5, kTol);
    EXPECT_NEAR(r.contacts[1].durationS, 0.3, kTol);
    EXPECT_EQ(r.contacts[1].marker.id, "a0-Hand-on-a1-Post-1");
    EXPECT_NEAR(r.contacts[0].meanDistanceM, 0.02, kTol);
    EXPECT_NEAR(r.contacts[1].meanDistanceM, 0.02, kTol);
}

TEST(ContactRuns, AGapOfOneFrameSeparatesAndARunMayStartOrEndAnywhere)
{
    const Vec3 c0{0.12, 0.0, 0.5};
    const Vec3 n0{0.5, 0.0, 0.5};
    MiniClip c(10, 7);
    c.hand0({n0, c0, n0, c0, c0, n0, n0}); // runs: frame 1; frames 3-4
    const DetectionResult r = detect(c, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 2U);
    EXPECT_NEAR(r.contacts[0].marker.window.startS, 0.1, kTol);
    EXPECT_NEAR(r.contacts[0].marker.window.endS, 0.1, kTol);
    EXPECT_NEAR(r.contacts[0].durationS, 0.1, kTol);
    EXPECT_NEAR(r.contacts[1].marker.window.startS, 0.3, kTol);
    EXPECT_NEAR(r.contacts[1].marker.window.endS, 0.4, kTol);
    EXPECT_NEAR(r.contacts[1].durationS, 0.2, kTol);
}

TEST(ContactRuns, ARunKeepsGoingWhileTheHandMovesAlongTheSurface)
{
    MiniClip c(10, 5);
    c.hand0(
        {{0.12, 0.0, 0.3}, {0.12, 0.0, 0.4}, {0.12, 0.0, 0.5}, {0.12, 0.0, 0.6}, {0.12, 0.0, 0.7}});
    const DetectionResult r = detect(c, withLow());
    ASSERT_EQ(r.contacts.size(), 1U);
    EXPECT_NEAR(r.contacts[0].durationS, 0.5, kTol);
}

// ---- Rule 6: tiers
// ------------------------------------------------------------------------------------------------

// A run of two or three contact frames (rate 10, default settings: threshold 0.03, minimum duration
// 0.25 s, maximum speed 0.2 m/s) with chosen close / long / still properties:
//   close: surface distance 0.01 (x = 0.11) against 0.02 (x = 0.12); the limit is 0.03 / 2 = 0.015
//   long:  3 frames = 0.3 s against 2 frames = 0.2 s
//   still: no motion (speed 0) against 0.1 m per frame (speed 1 m/s)
void expectTier(bool close, bool isLong, bool still, Confidence expected, int priority)
{
    SCOPED_TRACE(std::string("close=") + (close ? "1" : "0") + " long=" + (isLong ? "1" : "0") +
                 " still=" + (still ? "1" : "0"));
    const double x = close ? 0.11 : 0.12;
    const std::size_t frames = isLong ? 3 : 2;
    std::vector<Vec3> hand;
    for (std::size_t i = 0; i < frames; ++i) {
        const double z = still ? 0.5 : (isLong ? 0.4 : 0.45) + 0.1 * static_cast<double>(i);
        hand.push_back(Vec3{x, 0.0, z});
    }
    MiniClip c(10, frames);
    c.hand0(hand);
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_EQ(contact->confidence, expected);
    EXPECT_EQ(contact->marker.priority, priority);
    EXPECT_NEAR(contact->durationS, static_cast<double>(frames) / 10.0, kTol);
    EXPECT_NEAR(contact->meanDistanceM, x - 0.1, kTol);
    EXPECT_NEAR(contact->meanSpeedMps, still ? 0.0 : 1.0, 1e-9);
}

TEST(ContactTiers, AllThreeTestsPassGivesHigh)
{
    expectTier(true, true, true, Confidence::High, 2);
}

TEST(ContactTiers, ExactlyTwoTestsPassGivesMedium)
{
    expectTier(true, true, false, Confidence::Medium, 1); // close + long
    expectTier(true, false, true, Confidence::Medium, 1); // close + still
    expectTier(false, true, true, Confidence::Medium, 1); // long + still
}

TEST(ContactTiers, OneOrNoTestsGivesLow)
{
    expectTier(true, false, false, Confidence::Low, 0);  // close only
    expectTier(false, true, false, Confidence::Low, 0);  // long only
    expectTier(false, false, true, Confidence::Low, 0);  // still only
    expectTier(false, false, false, Confidence::Low, 0); // none
}

TEST(ContactTiers, TheTestsAreInclusiveAtTheirLimits)
{
    // All limits hit exactly, with binary-exact numbers: radius 0.5, threshold 0.5 (half 0.25),
    // rate 8, minimum duration 0.25 s (2 frames), maximum speed 1 m/s (0.125 m per frame * 8).
    DetectionSettings s;
    s.distanceThresholdM = 0.5;
    s.minDurationS = 0.25;
    s.maxRelativeSpeedMps = 1.0;
    const auto maps = postOnlyMaps(0.5);
    MiniClip c(8, 2);
    c.hand0({{0.75, 0.0, 0.5}, {0.75, 0.0, 0.625}}); // d = 0.25 = threshold / 2
    const DetectionResult r = detect(c, s, maps);
    const auto contact = onlyContact(r);
    ASSERT_NE(contact, nullptr);
    EXPECT_EQ(contact->confidence, Confidence::High);
    EXPECT_NEAR(contact->meanDistanceM, 0.25, 1e-12);
    EXPECT_NEAR(contact->durationS, 0.25, 1e-12);
    EXPECT_NEAR(contact->meanSpeedMps, 1.0, 1e-12);

    // Each limit just missed costs one test: Medium.
    DetectionSettings notClose = s;
    notClose.distanceThresholdM = 0.4999; // half 0.24995 < 0.25
    // (the hand is still within the threshold: d = 0.25 <= 0.4999)
    EXPECT_EQ(onlyContact(detect(c, notClose, maps))->confidence, Confidence::Medium);
    DetectionSettings notLong = s;
    notLong.minDurationS = 0.2501;
    EXPECT_EQ(onlyContact(detect(c, notLong, maps))->confidence, Confidence::Medium);
    DetectionSettings notStill = s;
    notStill.maxRelativeSpeedMps = 0.9999;
    EXPECT_EQ(onlyContact(detect(c, notStill, maps))->confidence, Confidence::Medium);
}

TEST(ContactTiers, SettingsDecideTheTiers)
{
    // The same clip is High with loose limits and Low with tight ones.
    MiniClip c(10, 3);
    c.hand0({{0.12, 0.0, 0.4}, {0.12, 0.0, 0.5}, {0.12, 0.0, 0.6}}); // d 0.02, 0.3 s, 1 m/s
    DetectionSettings loose = withLow();
    loose.distanceThresholdM = 0.1; // half 0.05 >= 0.02
    loose.minDurationS = 0.1;
    loose.maxRelativeSpeedMps = 2.0;
    EXPECT_EQ(onlyContact(detect(c, loose))->confidence, Confidence::High);
    DetectionSettings tight = withLow();
    tight.distanceThresholdM = 0.0201; // contact still, half 0.01005 < 0.02
    tight.minDurationS = 1.0;
    tight.maxRelativeSpeedMps = 0.5;
    EXPECT_EQ(onlyContact(detect(c, tight))->confidence, Confidence::Low);
}

// ---- includeLowConfidence
// ------------------------------------------------------------------------------------------

TEST(ContactLowConfidence, LowContactsAreDroppedUnlessRequested)
{
    MiniClip c(10, 2);
    c.hand0({{0.12, 0.0, 0.45}, {0.12, 0.0, 0.55}}); // not close, short, fast: Low
    const DetectionResult dropped = detect(c);       // includeLowConfidence is false
    EXPECT_TRUE(dropped.valid) << dropped.error;
    EXPECT_TRUE(dropped.contacts.empty());
    const DetectionResult kept = detect(c, withLow());
    const auto contact = onlyContact(kept);
    ASSERT_NE(contact, nullptr);
    EXPECT_EQ(contact->confidence, Confidence::Low);
    EXPECT_EQ(contact->marker.priority, 0);
}

TEST(ContactLowConfidence, HighAndMediumContactsAreKeptEitherWay)
{
    MiniClip high(10, 3);
    high.hand0(repeated({0.11, 0.0, 0.5}, 3));
    EXPECT_EQ(onlyContact(detect(high))->confidence, Confidence::High);
    EXPECT_EQ(onlyContact(detect(high, withLow()))->confidence, Confidence::High);

    MiniClip medium(10, 2);
    medium.hand0(repeated({0.11, 0.0, 0.5}, 2)); // close + still, short
    EXPECT_EQ(onlyContact(detect(medium))->confidence, Confidence::Medium);
    EXPECT_EQ(onlyContact(detect(medium, withLow()))->confidence, Confidence::Medium);
}

// ---- Rule 7: markers
// ------------------------------------------------------------------------------------------------

TEST(ContactMarker, FieldsOfAKeptContact)
{
    DetectionSettings s = withLow();
    s.markerToleranceM = 0.05;
    MiniClip c(10, 3);
    c.hand0(repeated({0.11, 0.0, 0.5}, 3));
    const DetectionResult r = detect(c, s);
    const auto contact = onlyContact(r);
    ASSERT_NE(contact, nullptr);
    const opas::markers::InteractionMarker& m = contact->marker;
    EXPECT_EQ(m.id, "a0-Hand-on-a1-Post-0");
    EXPECT_EQ(m.sourceActor, 0);
    EXPECT_EQ(m.targetActor, 1);
    EXPECT_EQ(m.sourceBone, "Hand");
    EXPECT_EQ(m.targetRegion, "Post");
    EXPECT_PRED_FORMAT3(Vec3Near, m.offsetM, (Vec3{0.11, 0.0, 0.5}), kTol);
    EXPECT_NEAR(m.window.startS, 0.0, kTol);
    EXPECT_NEAR(m.window.endS, 0.2, kTol);
    EXPECT_EQ(m.priority, 2);
    EXPECT_EQ(m.toleranceM, 0.05);
    EXPECT_NEAR(length(m.contactDirection), 1.0, kTol);
}

TEST(ContactMarker, TheIdKeepsNamesWithLettersDigitsDashUnderscoreAndDotAsTheyAre)
{
    const MiniNames names{"Ab-1_2.x", "Region_9.z-q"};
    MiniClip c(10, 3, names);
    c.hand0(repeated({0.11, 0.0, 0.5}, 3), "Ab-1_2.x");
    const auto contact = onlyContact(detect(c, withLow(), miniBodyMaps(0.1, names)));
    ASSERT_NE(contact, nullptr);
    EXPECT_EQ(contact->marker.id, "a0-Ab-1_2.x-on-a1-Region_9.z-q-0");
}

TEST(ContactMarker, OtherCharactersInBoneAndRegionNamesBecomeUnderscoresInTheIdOnly)
{
    const MiniNames names{"Left Hand [LHnd]", "Chest.1/x y"};
    MiniClip c(10, 3, names);
    c.hand0(repeated({0.11, 0.0, 0.5}, 3), "Left Hand [LHnd]");
    const auto contact = onlyContact(detect(c, withLow(), miniBodyMaps(0.1, names)));
    ASSERT_NE(contact, nullptr);
    // space, "[" and "]" each become one underscore, "/" and the space too.
    EXPECT_EQ(contact->marker.id, "a0-Left_Hand__LHnd_-on-a1-Chest.1_x_y-0");
    // The marker's own fields keep the real names.
    EXPECT_EQ(contact->marker.sourceBone, "Left Hand [LHnd]");
    EXPECT_EQ(contact->marker.targetRegion, "Chest.1/x y");
}

TEST(ContactMarker, ARunIndexCountsTheRunsOfAPairInTimeOrderIncludingDroppedLowOnes)
{
    // Frame 0: a one-frame, not-close contact (x = 0.12) which is Low; frame 1: far; frames 2-4:
    // a three-frame close contact at x = 0.11, Medium (long and close, but its first frame's
    // central difference reaches the far frame, so it is not still).
    const Vec3 far{1.0, 0.0, 0.5};
    MiniClip c(10, 5);
    c.hand0({{0.12, 0.0, 0.5}, far, {0.11, 0.0, 0.5}, {0.11, 0.0, 0.5}, {0.11, 0.0, 0.5}});

    const DetectionResult all = detect(c, withLow());
    ASSERT_TRUE(all.valid) << all.error;
    ASSERT_EQ(all.contacts.size(), 2U);
    EXPECT_EQ(all.contacts[0].marker.id, "a0-Hand-on-a1-Post-0");
    EXPECT_EQ(all.contacts[0].confidence, Confidence::Low);
    EXPECT_EQ(all.contacts[1].marker.id, "a0-Hand-on-a1-Post-1");
    EXPECT_EQ(all.contacts[1].confidence, Confidence::Medium);

    // Without the Low contact the other keeps its id: -1, not -0 (ids do not depend on the
    // profile).
    const DetectionResult kept = detect(c);
    ASSERT_TRUE(kept.valid) << kept.error;
    ASSERT_EQ(kept.contacts.size(), 1U);
    EXPECT_EQ(kept.contacts[0].marker.id, "a0-Hand-on-a1-Post-1");
    EXPECT_EQ(kept.contacts[0].confidence, Confidence::Medium);
}

TEST(ContactMarker, TheRunIndexIsCountedPerSourceBoneTargetRegionPair)
{
    // Hand touches the post in two runs; Hand2 touches the post in one run; actor 1's hand touches
    // actor 0's post in one run. Every pair starts at 0.
    const Vec3 c0{0.11, 0.0, 0.5};
    const Vec3 n0{0.5, 0.0, 0.5};
    MiniClip c(10, 7);
    c.hand0({c0, c0, c0, n0, c0, c0, c0});
    c.hand0({n0, n0, n0, n0, {-0.11, 0.0, 0.5}, {-0.11, 0.0, 0.5}, {-0.11, 0.0, 0.5}}, "Hand2");
    for (std::size_t f = 0; f < 3; ++f) {
        c.set(1, "Hand", f, Vec3{100.11, 0.0, 0.5});
    }
    const DetectionResult r = detect(c, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    std::vector<std::string> ids;
    ids.reserve(r.contacts.size());
    for (const DetectedContact& d : r.contacts) {
        ids.push_back(d.marker.id);
    }
    // Sorted by start time, then id.
    const std::vector<std::string> expected = {
        "a0-Hand-on-a1-Post-0",  // frames 0-2, start 0.0
        "a1-Hand-on-a0-Post-0",  // frames 0-2, start 0.0
        "a0-Hand-on-a1-Post-1",  // frames 4-6, start 0.4
        "a0-Hand2-on-a1-Post-0", // frames 4-6, start 0.4
    };
    EXPECT_EQ(ids, expected);
}

TEST(ContactMarker, AMarkerSetHoldsTheMarkersInTheSameOrder)
{
    const Vec3 c0{0.11, 0.0, 0.5};
    MiniClip c(10, 4);
    c.hand0({c0, c0, c0, c0});
    c.hand0({{-0.11, 0.0, 0.5}, {-0.11, 0.0, 0.5}, {-0.11, 0.0, 0.5}, {-0.11, 0.0, 0.5}}, "Hand2");
    const DetectionResult r = detect(c, withLow());
    ASSERT_EQ(r.contacts.size(), 2U);
    const opas::markers::MarkerSet set = r.markerSet();
    ASSERT_EQ(set.markers.size(), 2U);
    EXPECT_TRUE(set.markers[0] == r.contacts[0].marker);
    EXPECT_TRUE(set.markers[1] == r.contacts[1].marker);
    EXPECT_TRUE(DetectionResult{}.markerSet().markers.empty());
}

// ---- Rule 7: contact direction
// ------------------------------------------------------------------------------------------

TEST(ContactDirection, PointsFromTheHandIntoTheSurface)
{
    // The hand at +X of the post: n (axis to hand) = +X, direction = -X.
    MiniClip c(10, 3);
    c.hand0(repeated({0.12, 0.0, 0.5}, 3));
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    EXPECT_PRED_FORMAT3(Vec3Near, contact->marker.contactDirection, (Vec3{-1.0, 0.0, 0.0}), kTol);
}

TEST(ContactDirection, BelowTheBaseTheDirectionIsUp)
{
    // (0,0,-0.12): closest axis point is the base, n = (0,0,-1), direction (0,0,1). (The Rod is
    // left out: it starts at the same base point and would be touched too.)
    MiniClip c(10, 3);
    c.hand0(repeated({0.0, 0.0, -0.12}, 3));
    const auto contact = onlyContact(detect(c, withLow(), postOnlyMaps()));
    ASSERT_NE(contact, nullptr);
    EXPECT_PRED_FORMAT3(Vec3Near, contact->marker.contactDirection, (Vec3{0.0, 0.0, 1.0}), kTol);
}

TEST(ContactDirection, IsTheUnitVectorOfTheMeanOfTheInwardDirections)
{
    // Frame 0 hand at +X (n = (1,0,0)), frame 1 at +Y (n = (0,1,0)): mean of -n is (-0.5, -0.5, 0),
    // whose unit vector is (-sqrt(1/2), -sqrt(1/2), 0). The offset is the mean point (0.06, 0.06,
    // 0.5).
    MiniClip c(10, 2);
    c.hand0({{0.12, 0.0, 0.5}, {0.0, 0.12, 0.5}});
    const auto contact = onlyContact(detect(c, withLow()));
    ASSERT_NE(contact, nullptr);
    const double s = std::sqrt(0.5);
    EXPECT_PRED_FORMAT3(Vec3Near, contact->marker.contactDirection, (Vec3{-s, -s, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, contact->marker.offsetM, (Vec3{0.06, 0.06, 0.5}), kTol);
}

TEST(ContactDirection, ZeroMeanFallsBackToMinusZ)
{
    // Opposite sides of the post: the inward directions cancel.
    MiniClip opposite(10, 2);
    opposite.hand0({{0.12, 0.0, 0.5}, {-0.12, 0.0, 0.5}});
    const auto o = onlyContact(detect(opposite, withLow()));
    ASSERT_NE(o, nullptr);
    EXPECT_PRED_FORMAT3(Vec3Near, o->marker.contactDirection, (Vec3{0.0, 0.0, -1.0}), kTol);

    // On the axis every frame: there is no direction from the axis to the hand at all.
    MiniClip onAxis(10, 3);
    onAxis.hand0(repeated({0.0, 0.0, 0.5}, 3));
    const auto a = onlyContact(detect(onAxis, withLow()));
    ASSERT_NE(a, nullptr);
    EXPECT_PRED_FORMAT3(Vec3Near, a->marker.contactDirection, (Vec3{0.0, 0.0, -1.0}), kTol);
}

// ---- Rule 8: output order
// ---------------------------------------------------------------------------------------------------

TEST(ContactOrder, SortedByStartTimeThenId)
{
    const Vec3 c0{0.11, 0.0, 0.5};
    const Vec3 n0{0.5, 0.0, 0.5};
    // Actor 0's hand touches in frames 3-5; actor 1's hand touches actor 0's post in frames 0-2.
    // By id "a0-..." would come first; by time "a1-..." does.
    MiniClip c(10, 6);
    c.hand0({n0, n0, n0, c0, c0, c0});
    for (std::size_t f = 0; f < 3; ++f) {
        c.set(1, "Hand", f, Vec3{100.11, 0.0, 0.5});
    }
    const DetectionResult r = detect(c, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 2U);
    EXPECT_EQ(r.contacts[0].marker.sourceActor, 1);
    EXPECT_NEAR(r.contacts[0].marker.window.startS, 0.0, kTol);
    EXPECT_EQ(r.contacts[1].marker.sourceActor, 0);
    EXPECT_NEAR(r.contacts[1].marker.window.startS, 0.3, kTol);
}

TEST(ContactOrder, EqualStartTimesAreOrderedById)
{
    // Post and Rod touched in the same frames: "...-Post-0" before "...-Rod-0".
    MiniClip c(10, 4);
    c.hand0(repeated({0.12, 0.0, 0.05}, 4));
    const DetectionResult r = detect(c, withLow());
    ASSERT_EQ(r.contacts.size(), 2U);
    EXPECT_EQ(r.contacts[0].marker.id, "a0-Hand-on-a1-Post-0");
    EXPECT_EQ(r.contacts[1].marker.id, "a0-Hand-on-a1-Rod-0");

    // Different sources in the same frames: "a0-Hand-..." < "a0-Hand2-..." (a prefix sorts first).
    MiniClip two(10, 4);
    two.hand0(repeated({0.11, 0.0, 0.5}, 4), "Hand");
    two.hand0(repeated({-0.11, 0.0, 0.5}, 4), "Hand2");
    const DetectionResult t = detect(two, withLow());
    ASSERT_EQ(t.contacts.size(), 2U);
    EXPECT_EQ(t.contacts[0].marker.id, "a0-Hand-on-a1-Post-0");
    EXPECT_EQ(t.contacts[1].marker.id, "a0-Hand2-on-a1-Post-0");
}

TEST(ContactOrder, ResultsAreDeterministic)
{
    const Vec3 c0{0.11, 0.0, 0.5};
    MiniClip c(10, 5);
    c.hand0({c0, c0, {0.5, 0.0, 0.5}, c0, c0});
    c.hand0(repeated({-0.12, 0.0, 0.3}, 5), "Hand2");
    const DetectionResult first = detect(c, withLow());
    const DetectionResult second = detect(c, withLow());
    ASSERT_EQ(first.contacts.size(), second.contacts.size());
    for (std::size_t i = 0; i < first.contacts.size(); ++i) {
        EXPECT_TRUE(first.contacts[i].marker == second.contacts[i].marker);
        EXPECT_EQ(first.contacts[i].confidence, second.contacts[i].confidence);
        EXPECT_EQ(first.contacts[i].durationS, second.contacts[i].durationS);
        EXPECT_EQ(first.contacts[i].meanDistanceM, second.contacts[i].meanDistanceM);
        EXPECT_EQ(first.contacts[i].meanSpeedMps, second.contacts[i].meanSpeedMps);
    }
}

// ---- Invalid input
// --------------------------------------------------------------------------------------------------------------

TEST(ContactInvalid, EveryThresholdMustBePositiveAndFinite)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double bad[] = {0.0, -0.01, nan, inf, -inf};
    MiniClip c(10, 3);
    c.hand0(repeated({0.11, 0.0, 0.5}, 3));
    for (const double value : bad) {
        SCOPED_TRACE(value);
        DetectionSettings a;
        a.distanceThresholdM = value;
        DetectionSettings b;
        b.minDurationS = value;
        DetectionSettings d;
        d.maxRelativeSpeedMps = value;
        DetectionSettings e;
        e.markerToleranceM = value;
        for (const DetectionSettings& s : {a, b, d, e}) {
            DetectionResult r;
            EXPECT_NO_THROW(r = detect(c, s));
            EXPECT_FALSE(r.valid);
            EXPECT_FALSE(r.error.empty());
            EXPECT_TRUE(r.contacts.empty());
        }
    }
}

TEST(ContactInvalid, TheSampleRateMustBePositiveAndFinite)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double rate : {0.0, -30.0, nan, inf}) {
        SCOPED_TRACE(rate);
        MiniClip c(10, 3);
        c.hand0(repeated({0.11, 0.0, 0.5}, 3));
        c.clip().sampleRateHz = rate;
        DetectionResult r;
        EXPECT_NO_THROW(r = detect(c, withLow()));
        EXPECT_FALSE(r.valid);
        EXPECT_FALSE(r.error.empty());
        EXPECT_TRUE(r.contacts.empty());
    }
}

TEST(ContactInvalid, ValidInputGivesAnEmptyError)
{
    MiniClip c(10, 3);
    c.hand0(repeated({0.11, 0.0, 0.5}, 3));
    const DetectionResult r = detect(c, withLow());
    EXPECT_TRUE(r.valid);
    EXPECT_TRUE(r.error.empty());
}

TEST(ContactInvalid, NoContactAtAllIsValidAndEmpty)
{
    MiniClip c(10, 5); // every bone far from every capsule
    const DetectionResult r = detect(c, withLow());
    EXPECT_TRUE(r.valid) << r.error;
    EXPECT_TRUE(r.error.empty());
    EXPECT_TRUE(r.contacts.empty());
}

} // namespace
