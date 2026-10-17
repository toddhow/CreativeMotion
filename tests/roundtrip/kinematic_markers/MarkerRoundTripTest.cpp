#include "MarkerTestData.hpp"
#include "TestSupport.hpp"
#include "opas/markers/InteractionMarker.hpp"
#include "opas/markers/MarkerFile.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>

namespace {

using opas::markers::InteractionMarker;
using opas::markers::loadMarkerSet;
using opas::markers::LoadResult;
using opas::markers::MarkerSet;
using opas::markers::writeMarkerSet;
using opas::math::Vec3;
using opas::test::kEmptyMarkersText;
using opas::test::kOneMarkerText;
using opas::test::kTwoMarkerText;
using opas::test::SeededRandom;

constexpr const char* kFile = "roundtrip.markers.json";

// Only call after checking ok(); value_or keeps static analysis happy about the optional.
MarkerSet loadedSet(const LoadResult& result)
{
    return result.value.value_or(MarkerSet{});
}

TEST(MarkersRoundTrip, LoadThenWriteOfACanonicalFileIsByteForByteIdentical)
{
    for (const std::string_view text : {kOneMarkerText, kTwoMarkerText, kEmptyMarkersText}) {
        SCOPED_TRACE(std::string(text));
        const LoadResult loaded = loadMarkerSet(text, kFile);
        ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
        EXPECT_EQ(writeMarkerSet(loadedSet(loaded)), text);
    }
}

TEST(MarkersRoundTrip, WriteThenLoadGivesAnEqualSet)
{
    for (const MarkerSet& set :
         {opas::test::oneMarkerSet(), opas::test::twoMarkerSet(), MarkerSet{}}) {
        const std::string text = writeMarkerSet(set);
        SCOPED_TRACE(text);
        const LoadResult loaded = loadMarkerSet(text, kFile);
        ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
        EXPECT_TRUE(loadedSet(loaded) == set);
    }
}

TEST(MarkersRoundTrip, WriteProducesTheDocumentedCanonicalText)
{
    EXPECT_EQ(writeMarkerSet(opas::test::oneMarkerSet()), kOneMarkerText);
    EXPECT_EQ(writeMarkerSet(opas::test::twoMarkerSet()), kTwoMarkerText);
    // An empty marker array is written inline.
    EXPECT_EQ(writeMarkerSet(MarkerSet{}), kEmptyMarkersText);
}

TEST(MarkersRoundTrip, WriteUsesLfLineEndingsAndEndsWithOneNewline)
{
    const std::string text = writeMarkerSet(opas::test::twoMarkerSet());
    EXPECT_EQ(text.find('\r'), std::string::npos);
    ASSERT_GE(text.size(), 2U);
    EXPECT_EQ(text.back(), '\n');
    EXPECT_NE(text[text.size() - 2], '\n');
}

TEST(MarkersRoundTrip, NumbersAreWrittenInShortestRoundTripForm)
{
    InteractionMarker m = opas::test::rightHandOnShoulder();
    m.offsetM = Vec3{0.1, 2.0, 0.1 + 0.2}; // 2.0 is written 2; 0.1 + 0.2 needs 17 digits
    m.window = {0.0, 123456.789};
    m.toleranceM = 0.5;
    MarkerSet set;
    set.markers.push_back(m);
    const std::string text = writeMarkerSet(set);
    EXPECT_TRUE(opas::test::contains(text, "\"offsetM\": [0.1, 2, 0.30000000000000004],")) << text;
    EXPECT_TRUE(opas::test::contains(text, "\"window\": {\"startS\": 0, \"endS\": 123456.789},"))
        << text;
    EXPECT_TRUE(opas::test::contains(text, "\"toleranceM\": 0.5\n")) << text;
}

TEST(MarkersRoundTrip, TextWithQuotesBackslashesAndNonAsciiSurvives)
{
    InteractionMarker m = opas::test::rightHandOnShoulder();
    m.id = "id with \"quotes\" and \\ backslash";
    m.sourceBone = "Bone\tTab\nNewline";
    m.targetRegion = "caf\xC3\xA9"; // UTF-8 for "cafe" with an acute accent
    MarkerSet set;
    set.markers.push_back(m);

    const std::string text = writeMarkerSet(set);
    const LoadResult loaded = loadMarkerSet(text, kFile);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString() << "\n" << text;
    EXPECT_TRUE(loadedSet(loaded) == set);
    EXPECT_EQ(writeMarkerSet(loadedSet(loaded)), text);
}

// Random but valid marker set; every field comes from the seeded generator.
MarkerSet randomMarkerSet(SeededRandom& rng)
{
    MarkerSet set;
    const int count = rng.uniformInt(0, 5);
    for (int i = 0; i < count; ++i) {
        InteractionMarker m;
        m.id = "marker" + std::to_string(i);
        m.sourceBone = "Bone" + std::to_string(rng.uniformInt(0, 40));
        m.targetRegion = "Region" + std::to_string(rng.uniformInt(0, 12));
        m.offsetM =
            Vec3{rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)};
        m.contactDirection = opas::math::normalized(rng.unitVector());
        const double start = rng.chance(0.2) ? 0.0 : rng.uniform(0.0, 5.0);
        m.window = {start, start + (rng.chance(0.2) ? 0.0 : rng.uniform(0.0, 5.0))};
        m.priority = rng.uniformInt(-100, 100);
        m.toleranceM = rng.uniform(0.001, 0.1);
        set.markers.push_back(m);
    }
    return set;
}

TEST(MarkersRoundTripProperty, RandomSetsSurviveWriteLoadWrite)
{
    constexpr std::uint32_t kSeed = 4242U;
    SeededRandom rng(kSeed);
    for (int i = 0; i < 1000; ++i) {
        const MarkerSet set = randomMarkerSet(rng);
        const std::string text = writeMarkerSet(set);
        const LoadResult loaded = loadMarkerSet(text, kFile);
        ASSERT_TRUE(loaded.ok()) << "seed=" << kSeed << " case=" << i << "\n"
                                 << loaded.error.toString() << "\n"
                                 << text;
        ASSERT_TRUE(loadedSet(loaded) == set) << "seed=" << kSeed << " case=" << i << "\n" << text;
        ASSERT_EQ(writeMarkerSet(loadedSet(loaded)), text) << "seed=" << kSeed << " case=" << i;
    }
}

} // namespace
