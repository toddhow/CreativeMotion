// Body map format version 2: the "fingers" section (A5c, docs/formats/bodymap.md).
// Written by the implementer under the lighter A4-A6 workflow (CLAUDE.md).

#include "opas/markers/BodyMapFile.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

using opas::markers::BodyMapping;
using opas::markers::loadBodyMap;
using opas::markers::writeBodyMap;

constexpr const char* kWithFingers = R"({
  "format": "opas.bodymap",
  "version": 2,
  "limbs": [],
  "capsules": [],
  "ikChains": [
    {
      "name": "RightArm",
      "rootBone": "UpperArm",
      "midBone": "Forearm",
      "endBone": "Hand",
      "minBendRad": 0,
      "maxBendRad": 2.6,
      "poleDirection": [0, -1, 0],
      "limbRadiusM": 0.04
    }
  ],
  "fingers": [
    {
      "name": "RightIndex",
      "chain": "RightArm",
      "bones": ["Index1", "Index2", "Index3"],
      "hingeAxis": [0, 0, 1],
      "maxCurlRad": [1.2, 1.5, 1]
    }
  ]
}
)";

std::string replaced(std::string text, const std::string& from, const std::string& to)
{
    const auto at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    return text.replace(at, from.size(), to);
}

TEST(BodyMapFingers, AVersionTwoFileRoundTripsByteForByte)
{
    const auto loaded = loadBodyMap(kWithFingers, "fingers.bodymap.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    const BodyMapping m = loaded.value.value_or(BodyMapping{});
    ASSERT_EQ(m.fingers.size(), 1U);
    EXPECT_EQ(m.fingers[0].bones.size(), 3U);
    EXPECT_EQ(m.fingers[0].maxCurlRad[1], 1.5);
    EXPECT_EQ(writeBodyMap(m), kWithFingers);
}

TEST(BodyMapFingers, AMapWithoutFingersIsStillWrittenAsVersionOne)
{
    const auto loaded = loadBodyMap(kWithFingers, "f.json");
    ASSERT_TRUE(loaded.ok());
    BodyMapping m = loaded.value.value_or(BodyMapping{});
    m.fingers.clear();
    const std::string text = writeBodyMap(m);
    EXPECT_NE(text.find("\"version\": 1,"), std::string::npos);
    EXPECT_EQ(text.find("fingers"), std::string::npos);
    EXPECT_TRUE(loadBodyMap(text, "f.json").ok());
}

TEST(BodyMapFingers, FingersNeedVersionTwo)
{
    const auto r =
        loadBodyMap(replaced(kWithFingers, "\"version\": 2", "\"version\": 1"), "f.json");
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error.field, "/fingers");
}

TEST(BodyMapFingers, VersionTwoRequiresTheFingersArray)
{
    const std::string text =
        R"({"format": "opas.bodymap", "version": 2, "limbs": [], "capsules": [], "ikChains": []})";
    const auto r = loadBodyMap(text, "f.json");
    ASSERT_FALSE(r.ok());
    EXPECT_EQ(r.error.field, "/fingers");
}

TEST(BodyMapFingers, BadFingersAreReportedAtTheField)
{
    struct Case {
        std::string from;
        std::string to;
        std::string field;
    };
    for (const Case& c :
         {Case{"\"chain\": \"RightArm\"", "\"chain\": \"LeftArm\"", "/fingers/0/chain"},
          Case{"[1.2, 1.5, 1]", "[1.2, 1.5]", "/fingers/0/maxCurlRad"},
          Case{"[1.2, 1.5, 1]", "[1.2, 4, 1]", "/fingers/0/maxCurlRad/1"},
          Case{"\"hingeAxis\": [0, 0, 1]", "\"hingeAxis\": [0, 0, 0]", "/fingers/0/hingeAxis"},
          Case{"[\"Index1\", \"Index2\", \"Index3\"]", "[\"Index1\"]", "/fingers/0/bones"}}) {
        SCOPED_TRACE(c.to);
        const auto r = loadBodyMap(replaced(kWithFingers, c.from, c.to), "f.json");
        ASSERT_FALSE(r.ok());
        EXPECT_EQ(r.error.field, c.field);
    }
}

} // namespace
