#include "ClipTestData.hpp"
#include "TestSupport.hpp"
#include "opas/markers/ContactDetection.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace {

using opas::markers::DetectionProfileLoadResult;
using opas::markers::DetectionSettings;
using opas::markers::loadDetectionProfile;
using opas::markers::loadPoseClip;
using opas::markers::PoseClip;
using opas::markers::PoseClipLoadResult;
using opas::markers::writeDetectionProfile;
using opas::markers::writePoseClip;
using opas::math::Quat;
using opas::math::Transform;
using opas::math::Vec3;
using opas::test::kDetectionText;
using opas::test::kOtherDetectionText;
using opas::test::kPoseClipText;
using opas::test::poseClipOfText;
using opas::test::SeededRandom;

constexpr const char* kFile = "roundtrip.json";

PoseClip clipOf(const PoseClipLoadResult& r)
{
    return r.value.value_or(PoseClip{});
}

DetectionSettings settingsOf(const DetectionProfileLoadResult& r)
{
    return r.value.value_or(DetectionSettings{});
}

// ---- Pose clips -------------------------------------------------------------------------------

TEST(PoseClipRoundTrip, LoadThenWriteOfTheCanonicalFileIsByteForByteIdentical)
{
    const PoseClipLoadResult loaded = loadPoseClip(kPoseClipText, kFile);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_EQ(writePoseClip(clipOf(loaded)), kPoseClipText);
}

TEST(PoseClipRoundTrip, WriteProducesTheDocumentedCanonicalText)
{
    EXPECT_EQ(writePoseClip(poseClipOfText()), kPoseClipText);
}

TEST(PoseClipRoundTrip, WriteThenLoadGivesAnEqualClip)
{
    const std::string text = writePoseClip(poseClipOfText());
    const PoseClipLoadResult loaded = loadPoseClip(text, kFile);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_TRUE(clipOf(loaded) == poseClipOfText());
}

TEST(PoseClipRoundTrip, WriteUsesLfLineEndingsOneFramePerLineAndEndsWithOneNewline)
{
    const std::string text = writePoseClip(poseClipOfText());
    EXPECT_EQ(text.find('\r'), std::string::npos);
    ASSERT_GE(text.size(), 2U);
    EXPECT_EQ(text.back(), '\n');
    EXPECT_NE(text[text.size() - 2], '\n');
    EXPECT_TRUE(opas::test::contains(
        text, "        [[0, 0, 1, 0, 0, 0, 1], [0.3, -0.2, 1.1, 0, 0, 0, 1]],\n"));
    EXPECT_TRUE(opas::test::contains(text, "      \"bones\": [\"Pelvis\", \"RightHand\"],\n"));
}

TEST(PoseClipRoundTrip, NumbersAreWrittenInShortestRoundTripForm)
{
    PoseClip clip = poseClipOfText();
    clip.sampleRateHz = 29.97;
    clip.actors[0].frames[0][1].translation = Vec3{2.0, 0.1 + 0.2, -1e-7};
    const std::string text = writePoseClip(clip);
    EXPECT_TRUE(opas::test::contains(text, "\"sampleRateHz\": 29.97,\n")) << text;
    EXPECT_TRUE(opas::test::contains(text, "[2, 0.30000000000000004, ")) << text;
    const PoseClipLoadResult loaded = loadPoseClip(text, kFile);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_TRUE(clipOf(loaded) == clip);
}

TEST(PoseClipRoundTrip, BoneNamesWithSpecialCharactersSurvive)
{
    PoseClip clip = poseClipOfText();
    clip.actors[0].bones[1] = "NPC R Hand [RHnd] \"x\" \\ caf\xC3\xA9";
    const std::string text = writePoseClip(clip);
    const PoseClipLoadResult loaded = loadPoseClip(text, kFile);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString() << "\n" << text;
    EXPECT_TRUE(clipOf(loaded) == clip);
    EXPECT_EQ(writePoseClip(clipOf(loaded)), text);
}

// A random but valid clip; every number comes from the seeded generator.
PoseClip randomClip(SeededRandom& rng)
{
    PoseClip clip;
    clip.sampleRateHz = rng.uniform(1.0, 240.0);
    const int frames = rng.uniformInt(1, 6);
    for (opas::markers::ClipActor& actor : clip.actors) {
        actor.bodyMap = rng.uniformInt(0, 1) == 0 ? "humanoid" : "creature";
        const int bones = rng.uniformInt(1, 5);
        for (int b = 0; b < bones; ++b) {
            actor.bones.push_back("Bone" + std::to_string(b));
        }
    }
    for (opas::markers::ClipActor& actor : clip.actors) {
        for (int f = 0; f < frames; ++f) {
            std::vector<Transform> frame;
            for (std::size_t b = 0; b < actor.bones.size(); ++b) {
                Transform t;
                t.translation =
                    Vec3{rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), rng.uniform(0.0, 2.0)};
                const Vec3 axis = rng.unitVector();
                const double half = rng.uniform(-3.0, 3.0);
                t.rotation = Quat{axis.x * std::sin(half), axis.y * std::sin(half),
                                  axis.z * std::sin(half), std::cos(half)};
                frame.push_back(t);
            }
            actor.frames.push_back(frame);
        }
    }
    return clip;
}

TEST(PoseClipRoundTripProperty, RandomClipsSurviveWriteLoadWrite)
{
    constexpr std::uint32_t kSeed = 5150U;
    SeededRandom rng(kSeed);
    for (int i = 0; i < 500; ++i) {
        const PoseClip clip = randomClip(rng);
        const std::string text = writePoseClip(clip);
        const PoseClipLoadResult loaded = loadPoseClip(text, kFile);
        ASSERT_TRUE(loaded.ok()) << "seed=" << kSeed << " case=" << i << "\n"
                                 << loaded.error.toString() << "\n"
                                 << text;
        // Test choice: the loader keeps the numbers as written (it does not renormalize).
        ASSERT_TRUE(clipOf(loaded) == clip) << "seed=" << kSeed << " case=" << i << "\n" << text;
        ASSERT_EQ(writePoseClip(clipOf(loaded)), text) << "seed=" << kSeed << " case=" << i;
    }
}

// ---- Detection profiles
// --------------------------------------------------------------------------

TEST(DetectionProfileRoundTrip, CanonicalTextsAreByteForByteIdentical)
{
    for (const std::string_view text : {kDetectionText, kOtherDetectionText}) {
        SCOPED_TRACE(std::string(text));
        const DetectionProfileLoadResult loaded = loadDetectionProfile(text, kFile);
        ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
        EXPECT_EQ(writeDetectionProfile(settingsOf(loaded)), text);
    }
}

TEST(DetectionProfileRoundTrip, WriteProducesTheDocumentedCanonicalText)
{
    EXPECT_EQ(writeDetectionProfile(DetectionSettings{}), kDetectionText);
    EXPECT_EQ(writeDetectionProfile(opas::test::otherDetectionSettings()), kOtherDetectionText);
}

TEST(DetectionProfileRoundTripProperty, RandomProfilesSurviveWriteLoadWrite)
{
    constexpr std::uint32_t kSeed = 6160U;
    SeededRandom rng(kSeed);
    for (int i = 0; i < 1000; ++i) {
        DetectionSettings s;
        s.distanceThresholdM = rng.uniform(1e-4, 1.0);
        s.minDurationS = rng.uniform(1e-3, 10.0);
        s.maxRelativeSpeedMps = rng.uniform(1e-3, 5.0);
        s.markerToleranceM = rng.uniform(1e-4, 0.5);
        s.includeLowConfidence = rng.uniformInt(0, 1) == 1;
        const std::string text = writeDetectionProfile(s);
        const DetectionProfileLoadResult loaded = loadDetectionProfile(text, kFile);
        ASSERT_TRUE(loaded.ok()) << "seed=" << kSeed << " case=" << i << "\n"
                                 << loaded.error.toString() << "\n"
                                 << text;
        ASSERT_TRUE(settingsOf(loaded) == s) << "seed=" << kSeed << " case=" << i;
        ASSERT_EQ(writeDetectionProfile(settingsOf(loaded)), text)
            << "seed=" << kSeed << " case=" << i;
    }
}

} // namespace
