// Preset manifests and marker source choice (A6b, docs/formats/preset.md). Written
// by the implementer under the lighter A4-A6 workflow (CLAUDE.md).

#include "opas/markers/PresetFile.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

using opas::markers::chooseMarkerSource;
using opas::markers::loadPresetManifest;
using opas::markers::MarkerSource;
using opas::markers::PresetManifest;
using opas::markers::writePresetManifest;

constexpr const char* kFull = R"({
  "format": "opas.preset",
  "version": 1,
  "name": "Example Hugs",
  "author": "Someone",
  "presetVersion": "1.0.0",
  "license": "CC-BY-4.0",
  "priority": 10,
  "requires": ["OStim NG 7.3"],
  "description": "Hand markers for the hug scenes."
}
)";

constexpr const char* kMinimal = R"({
  "format": "opas.preset",
  "version": 1,
  "name": "Mini",
  "author": "A",
  "presetVersion": "0.1",
  "license": "MIT OR Apache-2.0",
  "priority": -3
}
)";

std::string replaced(std::string text, const std::string& from, const std::string& to)
{
    const auto at = text.find(from);
    EXPECT_NE(at, std::string::npos) << from;
    return text.replace(at, from.size(), to);
}

TEST(PresetManifest, CanonicalFilesRoundTripByteForByte)
{
    for (const char* text : {kFull, kMinimal}) {
        const auto loaded = loadPresetManifest(text, "preset.json");
        ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
        EXPECT_EQ(writePresetManifest(loaded.value.value_or(PresetManifest{})), text);
    }
}

TEST(PresetManifest, ValuesAreRead)
{
    const auto loaded = loadPresetManifest(kFull, "preset.json");
    ASSERT_TRUE(loaded.ok());
    const PresetManifest m = loaded.value.value_or(PresetManifest{});
    EXPECT_EQ(m.name, "Example Hugs");
    EXPECT_EQ(m.priority, 10);
    ASSERT_EQ(m.requirements.size(), 1U);
    EXPECT_EQ(m.requirements[0], "OStim NG 7.3");
}

TEST(PresetManifest, ErrorsAreLocated)
{
    struct Case {
        std::string from;
        std::string to;
        std::size_t line;
        std::string field;
    };
    for (const Case& c : {Case{"\"CC-BY-4.0\"", "\"see readme!\"", 7, "/license"},
                          Case{"\"license\": \"CC-BY-4.0\",\n", "", 1, "/license"},
                          Case{"\"priority\": 10", "\"priority\": 1.5", 8, "/priority"},
                          Case{"\"version\": 1", "\"version\": 2", 3, "/version"},
                          Case{"\"author\"", "\"autor\"", 5, "/autor"},
                          Case{"[\"OStim NG 7.3\"]", "[\"\"]", 9, "/requires/0"}}) {
        SCOPED_TRACE(c.to);
        const auto r = loadPresetManifest(replaced(kFull, c.from, c.to), "preset.json");
        ASSERT_FALSE(r.ok());
        EXPECT_EQ(r.error.line, c.line);
        EXPECT_EQ(r.error.field, c.field);
    }
}

TEST(MarkerSourceChoice, TheUsersOwnFileWins)
{
    const std::vector<MarkerSource> c{{false, "Big", 100}, {true, "", 0}, {false, "Other", 5}};
    EXPECT_EQ(chooseMarkerSource(c), 1U);
}

TEST(MarkerSourceChoice, HigherPriorityThenSmallerNameWins)
{
    const std::vector<MarkerSource> c{{false, "B", 5}, {false, "A", 5}, {false, "C", 3}};
    EXPECT_EQ(chooseMarkerSource(c), 1U);
    const std::vector<MarkerSource> d{{false, "B", 5}, {false, "A", 4}};
    EXPECT_EQ(chooseMarkerSource(d), 0U);
}

TEST(MarkerSourceChoice, NoCandidatesGiveNothing)
{
    EXPECT_FALSE(chooseMarkerSource({}).has_value());
}

} // namespace
