// Review amendments for contact detection (docs/tasks/0006-a4a-contact-detection.md): limbs sharing
// a last bone, clips whose in-memory sizes disagree, non-finite transforms, colliding sanitized
// ids. Uses the mini rig of ClipBuilder.hpp (a post of radius 0.1 on actor 1; actor 0's hands are
// placed by the tests).

#include "ClipBuilder.hpp"
#include "TestSupport.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/markers/ContactDetection.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace {

using opas::markers::BodyMapping;
using opas::markers::detectContacts;
using opas::markers::DetectionResult;
using opas::markers::DetectionSettings;
using opas::math::Transform;
using opas::math::Vec3;
using opas::test::miniBodyMaps;
using opas::test::MiniClip;

DetectionSettings withLow()
{
    DetectionSettings s;
    s.includeLowConfidence = true;
    return s;
}

std::vector<Vec3> repeated(Vec3 p, std::size_t n)
{
    return std::vector<Vec3>(n, p);
}

TEST(ContactReview, TwoLimbsEndingInTheSameBoneGiveOneCandidate)
{
    std::array<BodyMapping, 2> maps = miniBodyMaps();
    maps[0].limbs = {{"Arm", {"Base", "Elbow", "Hand"}},
                     {"Arm2", {"Base", "Hand"}}}; // both end in Hand
    MiniClip c(10, 4);
    c.hand0(repeated({0.11, 0.0, 0.5}, 4));
    const DetectionResult r = detectContacts(c.clip(), maps, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 1U); // not two contacts with the same id
    EXPECT_EQ(r.contacts[0].marker.id, "a0-Hand-on-a1-Post-0");
}

TEST(ContactReview, ABoneEndingSeveralLimbsStillCountsOnceWhenOtherLimbsComeBetween)
{
    std::array<BodyMapping, 2> maps = miniBodyMaps();
    maps[0].limbs = {
        {"A", {"Base", "Hand"}}, {"B", {"Base", "Hand2"}}, {"C", {"Base", "Elbow", "Hand"}}};
    MiniClip c(10, 4);
    c.hand0(repeated({0.11, 0.0, 0.5}, 4));
    c.hand0(repeated({-0.11, 0.0, 0.5}, 4), "Hand2");
    const DetectionResult r = detectContacts(c.clip(), maps, withLow());
    ASSERT_TRUE(r.valid) << r.error;
    ASSERT_EQ(r.contacts.size(), 2U);
    EXPECT_EQ(r.contacts[0].marker.id, "a0-Hand-on-a1-Post-0");
    EXPECT_EQ(r.contacts[1].marker.id, "a0-Hand2-on-a1-Post-0");
}

TEST(ContactReview, AClipWhoseActorsHaveDifferentFrameCountsIsInvalid)
{
    MiniClip c(10, 4);
    c.hand0(repeated({0.11, 0.0, 0.5}, 4));
    c.clip().actors[1].frames.pop_back(); // 4 frames against 3
    DetectionResult r;
    EXPECT_NO_THROW(r = detectContacts(c.clip(), miniBodyMaps(), withLow()));
    EXPECT_FALSE(r.valid);
    EXPECT_FALSE(r.error.empty());
    EXPECT_TRUE(r.contacts.empty());
}

TEST(ContactReview, AFrameWhoseSizeDiffersFromTheBoneListIsInvalid)
{
    for (const std::size_t actor : {std::size_t{0}, std::size_t{1}}) {
        SCOPED_TRACE(actor);
        MiniClip shortFrame(10, 3);
        shortFrame.clip().actors[actor].frames[1].pop_back();
        DetectionResult r;
        EXPECT_NO_THROW(r = detectContacts(shortFrame.clip(), miniBodyMaps(), withLow()));
        EXPECT_FALSE(r.valid);
        EXPECT_FALSE(r.error.empty());
        EXPECT_TRUE(r.contacts.empty());

        MiniClip longFrame(10, 3);
        longFrame.clip().actors[actor].frames[2].push_back(Transform{});
        EXPECT_NO_THROW(r = detectContacts(longFrame.clip(), miniBodyMaps(), withLow()));
        EXPECT_FALSE(r.valid);
        EXPECT_FALSE(r.error.empty());
    }
}

TEST(ContactReview, NonFiniteTransformsGiveNoContact)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Vec3 bad[] = {{nan, 0.0, 0.5}, {0.11, inf, 0.5}, {0.11, 0.0, -inf}, {nan, nan, nan}};
    for (const Vec3& p : bad) {
        SCOPED_TRACE(opas::test::str(p));
        // A hand that is never finite.
        MiniClip hand(10, 4);
        hand.hand0(repeated(p, 4));
        DetectionResult r;
        EXPECT_NO_THROW(r = detectContacts(hand.clip(), miniBodyMaps(), withLow()));
        EXPECT_TRUE(r.contacts.empty());

        // A target capsule whose tip is never finite, with the hand where the post would be.
        MiniClip tip(10, 4);
        tip.hand0(repeated({0.11, 0.0, 0.5}, 4));
        tip.setAll(1, "Tip", Transform{p, opas::math::Quat{}});
        EXPECT_NO_THROW(r = detectContacts(tip.clip(), miniBodyMaps(), withLow()));
        EXPECT_TRUE(r.contacts.empty());
    }
}

TEST(ContactReview, ARotationThatIsNotFiniteGivesNoContact)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    MiniClip c(10, 4);
    c.hand0(repeated({0.11, 0.0, 0.5}, 4));
    c.setAll(1, "Base", Transform{Vec3{0.0, 0.0, 0.0}, opas::math::Quat{nan, 0.0, 0.0, 1.0}});
    DetectionResult r;
    EXPECT_NO_THROW(r = detectContacts(c.clip(), miniBodyMaps(), withLow()));
    EXPECT_TRUE(r.contacts.empty());
}

TEST(ContactReview, CollidingSanitizedIdsKeepAFixedOrderAndAreAllReported)
{
    // "Hand A" and "Hand_A" both sanitize to "Hand_A": two contacts with the same id. The sort is
    // stable, so they come out in candidate order, which follows the limb order.
    const auto build = [](bool spacedFirst) {
        std::array<BodyMapping, 2> maps = miniBodyMaps();
        maps[0].limbs.clear();
        const opas::markers::LimbDefinition spaced{"ArmA", {"Base", "Hand A"}};
        const opas::markers::LimbDefinition underscored{"ArmB", {"Base", "Hand_A"}};
        if (spacedFirst) {
            maps[0].limbs = {spaced, underscored};
        } else {
            maps[0].limbs = {underscored, spaced};
        }
        return maps;
    };
    MiniClip c(10, 4);
    for (const char* name : {"Hand A", "Hand_A"}) {
        c.clip().actors[0].bones.push_back(name);
        for (auto& frame : c.clip().actors[0].frames) {
            frame.push_back(
                Transform{Vec3{name[4] == ' ' ? 0.11 : -0.11, 0.0, 0.5}, opas::math::Quat{}});
        }
    }
    const DetectionResult first = detectContacts(c.clip(), build(true), withLow());
    ASSERT_TRUE(first.valid) << first.error;
    ASSERT_EQ(first.contacts.size(), 2U);
    EXPECT_EQ(first.contacts[0].marker.id, "a0-Hand_A-on-a1-Post-0");
    EXPECT_EQ(first.contacts[1].marker.id, "a0-Hand_A-on-a1-Post-0");
    EXPECT_EQ(first.contacts[0].marker.sourceBone, "Hand A");
    EXPECT_EQ(first.contacts[1].marker.sourceBone, "Hand_A");

    const DetectionResult second = detectContacts(c.clip(), build(false), withLow());
    ASSERT_EQ(second.contacts.size(), 2U);
    EXPECT_EQ(second.contacts[0].marker.sourceBone, "Hand_A");
    EXPECT_EQ(second.contacts[1].marker.sourceBone, "Hand A");

    // Repeating the call gives the same order.
    const DetectionResult again = detectContacts(c.clip(), build(true), withLow());
    ASSERT_EQ(again.contacts.size(), 2U);
    EXPECT_EQ(again.contacts[0].marker.sourceBone, "Hand A");
    EXPECT_EQ(again.contacts[1].marker.sourceBone, "Hand_A");
}

} // namespace
