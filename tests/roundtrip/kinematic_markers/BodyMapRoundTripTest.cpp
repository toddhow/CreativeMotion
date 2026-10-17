#include "BodyMapTestData.hpp"
#include "TestSupport.hpp"
#include "opas/ik/TwoBoneIk.hpp"
#include "opas/markers/BodyMapFile.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>

namespace {

using opas::markers::BodyMapLoadResult;
using opas::markers::BodyMapping;
using opas::markers::CapsuleRule;
using opas::markers::IkChain;
using opas::markers::LimbDefinition;
using opas::markers::loadBodyMap;
using opas::markers::writeBodyMap;
using opas::math::Vec3;
using opas::test::bodyMapOfText;
using opas::test::kBodyMapText;
using opas::test::kEmptyBodyMapText;
using opas::test::kPi;
using opas::test::sameMapping;
using opas::test::SeededRandom;

constexpr const char* kFile = "roundtrip.bodymap.json";

// Only call after checking ok(); value_or keeps static analysis happy about the optional.
BodyMapping valueOf(const BodyMapLoadResult& result)
{
    return result.value.value_or(BodyMapping{});
}

TEST(BodyMapRoundTrip, LoadThenWriteOfACanonicalFileIsByteForByteIdentical)
{
    for (const std::string_view text : {kBodyMapText, kEmptyBodyMapText}) {
        SCOPED_TRACE(std::string(text));
        const BodyMapLoadResult loaded = loadBodyMap(text, kFile);
        ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
        EXPECT_EQ(writeBodyMap(valueOf(loaded)), text);
    }
}

TEST(BodyMapRoundTrip, WriteProducesTheDocumentedCanonicalText)
{
    EXPECT_EQ(writeBodyMap(bodyMapOfText()), kBodyMapText);
    // An empty array is written [].
    EXPECT_EQ(writeBodyMap(BodyMapping{}), kEmptyBodyMapText);
}

TEST(BodyMapRoundTrip, WriteThenLoadGivesAnEqualMapping)
{
    for (const BodyMapping& m : {bodyMapOfText(), BodyMapping{}}) {
        const std::string text = writeBodyMap(m);
        SCOPED_TRACE(text);
        const BodyMapLoadResult loaded = loadBodyMap(text, kFile);
        ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
        EXPECT_TRUE(sameMapping(valueOf(loaded), m));
    }
}

TEST(BodyMapRoundTrip, WriteUsesLfLineEndingsAndEndsWithOneNewline)
{
    const std::string text = writeBodyMap(bodyMapOfText());
    EXPECT_EQ(text.find('\r'), std::string::npos);
    ASSERT_GE(text.size(), 2U);
    EXPECT_EQ(text.back(), '\n');
    EXPECT_NE(text[text.size() - 2], '\n');
}

TEST(BodyMapRoundTrip, LimbsAndCapsulesAreOneLineEachAndChainsOneKeyPerLine)
{
    const std::string text = writeBodyMap(bodyMapOfText());
    EXPECT_TRUE(
        opas::test::contains(text, "    {\"name\": \"RightArm\", \"bones\": [\"RightUpperArm\", "
                                   "\"RightForearm\", \"RightHand\"]},\n"));
    EXPECT_TRUE(opas::test::contains(text, "    {\"region\": \"UpperBack\", \"bone\": \"Spine\", "
                                           "\"endBone\": \"Neck\", \"radiusRatio\": 0.1667}\n"));
    EXPECT_TRUE(opas::test::contains(text, "      \"poleDirection\": [0, -1, 0],\n"));
    EXPECT_TRUE(opas::test::contains(text, "    },\n    {\n      \"name\": \"LeftArm\",\n"));
}

TEST(BodyMapRoundTrip, NumbersAreWrittenInShortestRoundTripForm)
{
    BodyMapping m = bodyMapOfText();
    m.capsules[0].radiusRatio = 2.0;       // written 2
    m.capsules[1].radiusRatio = 0.1 + 0.2; // needs 17 digits
    m.ikChains[0].limits = opas::ik::HingeLimits{0.0, 3.0};
    m.ikChains[0].poleDirection = Vec3{0.5, -0.25, 1.0};
    m.ikChains[0].limbRadiusM = 0.125;
    const std::string text = writeBodyMap(m);
    EXPECT_TRUE(opas::test::contains(text, "\"radiusRatio\": 2}")) << text;
    EXPECT_TRUE(opas::test::contains(text, "\"radiusRatio\": 0.30000000000000004}")) << text;
    EXPECT_TRUE(opas::test::contains(text, "      \"minBendRad\": 0,\n      \"maxBendRad\": 3,\n"))
        << text;
    EXPECT_TRUE(opas::test::contains(text, "[0.5, -0.25, 1],\n      \"limbRadiusM\": 0.125\n"))
        << text;
}

TEST(BodyMapRoundTrip, TextWithQuotesBackslashesAndNonAsciiSurvives)
{
    BodyMapping m = bodyMapOfText();
    m.limbs[0].name = "arm \"one\" \\ back";
    m.limbs[0].bones[1] = "Bone\tTab\nNewline";
    m.capsules[0].region = "caf\xC3\xA9"; // UTF-8 for "cafe" with an acute accent
    const std::string text = writeBodyMap(m);
    const BodyMapLoadResult loaded = loadBodyMap(text, kFile);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString() << "\n" << text;
    EXPECT_TRUE(sameMapping(valueOf(loaded), m));
    EXPECT_EQ(writeBodyMap(valueOf(loaded)), text);
}

// Random but valid body map; every field comes from the seeded generator.
BodyMapping randomBodyMap(SeededRandom& rng)
{
    BodyMapping m;
    const int limbs = rng.uniformInt(0, 4);
    for (int i = 0; i < limbs; ++i) {
        LimbDefinition limb;
        limb.name = "limb" + std::to_string(i);
        const int bones = rng.uniformInt(2, 5);
        for (int b = 0; b < bones; ++b) {
            limb.bones.push_back("Bone" + std::to_string(rng.uniformInt(0, 60)));
        }
        m.limbs.push_back(limb);
    }
    const int capsules = rng.uniformInt(0, 5);
    for (int i = 0; i < capsules; ++i) {
        CapsuleRule c;
        c.region = "Region" + std::to_string(i);
        c.bone = "Bone" + std::to_string(rng.uniformInt(0, 60));
        c.endBone = "Bone" + std::to_string(rng.uniformInt(0, 60));
        c.radiusRatio = rng.chance(0.2) ? 1.0 : rng.uniform(0.01, 2.0);
        m.capsules.push_back(c);
    }
    const int chains = rng.uniformInt(0, 4);
    for (int i = 0; i < chains; ++i) {
        IkChain c;
        c.name = "chain" + std::to_string(i);
        c.rootBone = "Root" + std::to_string(rng.uniformInt(0, 60));
        c.midBone = "Mid" + std::to_string(rng.uniformInt(0, 60));
        c.endBone = "End" + std::to_string(i); // unique, as the format requires
        const double lo = rng.chance(0.2) ? 0.0 : rng.uniform(0.0, kPi);
        const double hi = rng.chance(0.2) ? kPi : std::min(kPi, lo + rng.uniform01() * (kPi - lo));
        c.limits = opas::ik::HingeLimits{lo, std::max(lo, hi)};
        c.poleDirection = rng.unitVector() * rng.uniform(0.1, 3.0);
        c.limbRadiusM = rng.uniform(0.001, 0.2);
        m.ikChains.push_back(c);
    }
    return m;
}

TEST(BodyMapRoundTripProperty, RandomMapsSurviveWriteLoadWrite)
{
    constexpr std::uint32_t kSeed = 31337U;
    SeededRandom rng(kSeed);
    for (int i = 0; i < 1000; ++i) {
        const BodyMapping m = randomBodyMap(rng);
        const std::string text = writeBodyMap(m);
        const BodyMapLoadResult loaded = loadBodyMap(text, kFile);
        ASSERT_TRUE(loaded.ok()) << "seed=" << kSeed << " case=" << i << "\n"
                                 << loaded.error.toString() << "\n"
                                 << text;
        ASSERT_TRUE(sameMapping(valueOf(loaded), m)) << "seed=" << kSeed << " case=" << i << "\n"
                                                     << text;
        ASSERT_EQ(writeBodyMap(valueOf(loaded)), text) << "seed=" << kSeed << " case=" << i;
    }
}

} // namespace
