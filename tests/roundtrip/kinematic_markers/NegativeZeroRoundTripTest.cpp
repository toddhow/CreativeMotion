// Review amendment (docs/tasks/0006-a4a-contact-detection.md): negative zero is written as 0 by
// every writer, so that text written from a value holding -0.0 loads and writes back byte for byte.

#include "TestSupport.hpp"
#include "opas/ik/TwoBoneIk.hpp"
#include "opas/markers/BodyMapFile.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/markers/InteractionMarker.hpp"
#include "opas/markers/MarkerFile.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

using opas::markers::BodyMapping;
using opas::markers::InteractionMarker;
using opas::markers::loadBodyMap;
using opas::markers::loadMarkerSet;
using opas::markers::loadPoseClip;
using opas::markers::MarkerSet;
using opas::markers::PoseClip;
using opas::markers::writeBodyMap;
using opas::markers::writeMarkerSet;
using opas::markers::writePoseClip;
using opas::math::Quat;
using opas::math::Transform;
using opas::math::Vec3;
using opas::test::contains;

constexpr double kNegZero = -0.0;

TEST(NegativeZeroPoseClip, IsWrittenAsZeroAndRoundTripsByteForByte)
{
    PoseClip clip;
    clip.sampleRateHz = 30.0;
    for (opas::markers::ClipActor& actor : clip.actors) {
        actor.bodyMap = "humanoid";
        actor.bones = {"Pelvis"};
        actor.frames = {
            {Transform{Vec3{kNegZero, kNegZero, 1.0}, Quat{kNegZero, kNegZero, kNegZero, 1.0}}}};
    }
    const std::string text = writePoseClip(clip);
    EXPECT_TRUE(contains(text, "[[0, 0, 1, 0, 0, 0, 1]]")) << text;
    EXPECT_FALSE(contains(text, "-0,")) << text;
    EXPECT_FALSE(contains(text, "-0]")) << text;

    const auto loaded = loadPoseClip(text, "negzero.poseclip.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_EQ(writePoseClip(loaded.value.value_or(PoseClip{})), text);
    EXPECT_TRUE(loaded.value.value_or(PoseClip{}) == clip); // -0.0 == 0.0
}

TEST(NegativeZeroMarkerSet, IsWrittenAsZeroAndRoundTripsByteForByte)
{
    InteractionMarker m;
    m.id = "m";
    m.sourceActor = 0;
    m.targetActor = 1;
    m.sourceBone = "RightHand";
    m.targetRegion = "Chest";
    m.offsetM = Vec3{kNegZero, 0.0, 0.1};
    m.contactDirection = Vec3{kNegZero, kNegZero, -1.0};
    m.window = {kNegZero, 2.0};
    m.priority = 1;
    m.toleranceM = 0.02;
    MarkerSet set;
    set.markers.push_back(m);

    const std::string text = writeMarkerSet(set);
    EXPECT_TRUE(contains(text, "\"offsetM\": [0, 0, 0.1],")) << text;
    EXPECT_TRUE(contains(text, "\"contactDirection\": [0, 0, -1],")) << text;
    EXPECT_TRUE(contains(text, "\"window\": {\"startS\": 0, \"endS\": 2},")) << text;

    const auto loaded = loadMarkerSet(text, "negzero.markers.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_EQ(writeMarkerSet(loaded.value.value_or(MarkerSet{})), text);
}

TEST(NegativeZeroBodyMap, IsWrittenAsZeroAndRoundTripsByteForByte)
{
    BodyMapping m;
    m.limbs = {{"Arm", {"Upper", "Lower", "Hand"}}};
    m.capsules = {{"Forearm", "Lower", "Hand", 0.15}};
    opas::markers::IkChain chain;
    chain.name = "Arm";
    chain.rootBone = "Upper";
    chain.midBone = "Lower";
    chain.endBone = "Hand";
    chain.limits = opas::ik::HingeLimits{kNegZero, 2.6};
    chain.poleDirection = Vec3{kNegZero, -1.0, kNegZero};
    chain.limbRadiusM = 0.04;
    m.ikChains = {chain};

    const std::string text = writeBodyMap(m);
    EXPECT_TRUE(contains(text, "\"minBendRad\": 0,")) << text;
    EXPECT_TRUE(contains(text, "\"poleDirection\": [0, -1, 0],")) << text;

    const auto loaded = loadBodyMap(text, "negzero.bodymap.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_EQ(writeBodyMap(loaded.value.value_or(BodyMapping{})), text);
}

} // namespace
