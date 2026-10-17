#include "ClipTestData.hpp"
#include "TestSupport.hpp"
#include "opas/markers/ContactDetection.hpp"
#include "opas/markers/MarkerFile.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace {

using opas::markers::DetectionProfileLoadResult;
using opas::markers::DetectionSettings;
using opas::markers::loadDetectionProfile;
using opas::markers::loadDetectionProfileFile;
using opas::test::kDetectionText;
using opas::test::kOtherDetectionText;
using opas::test::otherDetectionSettings;

constexpr const char* kFile = "synthetic.detection.json";

std::string replaceOnce(std::string_view original, std::string_view from, std::string_view to)
{
    std::string text(original);
    const std::size_t first = text.find(from);
    if (first == std::string::npos || text.find(from, first + 1) != std::string::npos) {
        ADD_FAILURE() << "test text must contain exactly one: " << from;
        return text;
    }
    text.replace(first, from.size(), to);
    return text;
}

DetectionSettings valueOf(const DetectionProfileLoadResult& result)
{
    return result.value.value_or(DetectionSettings{});
}

void expectRejected(const std::string& text, std::size_t line, const std::string& field)
{
    const DetectionProfileLoadResult result = loadDetectionProfile(text, kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.value.has_value());
    EXPECT_EQ(result.error.file, kFile);
    EXPECT_EQ(result.error.line, line);
    EXPECT_EQ(result.error.field, field);
    EXPECT_FALSE(result.error.message.empty());
}

struct Edit {
    const char* from;
    const char* to;
    std::size_t line;
    const char* field;
};

void expectAllRejected(const std::vector<Edit>& edits)
{
    for (const Edit& e : edits) {
        SCOPED_TRACE(std::string(e.from) + " -> " + e.to);
        expectRejected(replaceOnce(kDetectionText, e.from, e.to), e.line, e.field);
    }
}

// Line numbers refer to kDetectionText (see ClipTestData.hpp): format 2, version 3,
// distanceThresholdM 4, minDurationS 5, maxRelativeSpeedMps 6, markerToleranceM 7,
// includeLowConfidence 8.

TEST(DetectionProfile, DefaultsMatchTheDocumentedStartingValues)
{
    const DetectionSettings d;
    EXPECT_EQ(d.distanceThresholdM, 0.03);
    EXPECT_EQ(d.minDurationS, 0.25);
    EXPECT_EQ(d.maxRelativeSpeedMps, 0.2);
    EXPECT_EQ(d.markerToleranceM, 0.02);
    EXPECT_FALSE(d.includeLowConfidence);
}

TEST(DetectionProfileLoad, CanonicalDefaultFileLoadsToTheDefaults)
{
    const DetectionProfileLoadResult result = loadDetectionProfile(kDetectionText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == DetectionSettings{});
}

TEST(DetectionProfileLoad, AnotherProfileLoadsToItsValues)
{
    const DetectionProfileLoadResult result = loadDetectionProfile(kOtherDetectionText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == otherDetectionSettings());
}

TEST(DetectionProfileLoad, AcceptsValidJsonThatIsNotCanonical)
{
    const std::string loose =
        "{\"includeLowConfidence\":true,\"markerToleranceM\":1e-2,\"maxRelativeSpeedMps\":1E-1,"
        "\"minDurationS\":0.5,\"distanceThresholdM\":5e-2,\"version\":1,\"format\":\"opas."
        "detection\"}";
    const DetectionProfileLoadResult result = loadDetectionProfile(loose, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == otherDetectionSettings());
}

TEST(DetectionProfileLoad, IntegerSpelledNumbersAreAccepted)
{
    const DetectionProfileLoadResult result = loadDetectionProfile(
        replaceOnce(kDetectionText, "\"minDurationS\": 0.25,", "\"minDurationS\": 1,"), kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_EQ(valueOf(result).minDurationS, 1.0);
}

TEST(DetectionProfileLoadErrors, MalformedJson)
{
    expectAllRejected({
        {"\"version\": 1,", "\"version\": @,", 3, ""},
        {"\"minDurationS\": 0.25,", "\"minDurationS\": 0.25x,", 5, ""},
        {"\"includeLowConfidence\": false", "\"includeLowConfidence\": fals", 8, ""},
    });
    expectRejected("[]\n", 1, "");
}

TEST(DetectionProfileLoadErrors, WrongFormatAndVersion)
{
    expectAllRejected({
        {"\"opas.detection\"", "\"opas.poseclip\"", 2, "/format"},
        {"\"format\": \"opas.detection\"", "\"format\": 3", 2, "/format"},
        {"\"version\": 1,", "\"version\": 2,", 3, "/version"},
        {"\"version\": 1,", "\"version\": 0,", 3, "/version"},
        {"\"version\": 1,", "\"version\": \"1\",", 3, "/version"},
        {"\"version\": 1,", "\"version\": 1.5,", 3, "/version"},
    });
}

TEST(DetectionProfileLoadErrors, MissingFieldIsReportedAtLineOne)
{
    expectAllRejected({
        {"  \"format\": \"opas.detection\",\n", "", 1, "/format"},
        {"  \"version\": 1,\n", "", 1, "/version"},
        {"  \"distanceThresholdM\": 0.03,\n", "", 1, "/distanceThresholdM"},
        {"  \"minDurationS\": 0.25,\n", "", 1, "/minDurationS"},
        {"  \"maxRelativeSpeedMps\": 0.2,\n", "", 1, "/maxRelativeSpeedMps"},
        {"  \"markerToleranceM\": 0.02,\n", "", 1, "/markerToleranceM"},
        // The last key has no trailing comma: remove it with the comma before it.
        {"0.02,\n  \"includeLowConfidence\": false\n", "0.02\n", 1, "/includeLowConfidence"},
    });
}

TEST(DetectionProfileLoadErrors, WrongTypes)
{
    expectAllRejected({
        {"\"distanceThresholdM\": 0.03", "\"distanceThresholdM\": \"0.03\"", 4,
         "/distanceThresholdM"},
        {"\"minDurationS\": 0.25", "\"minDurationS\": null", 5, "/minDurationS"},
        {"\"maxRelativeSpeedMps\": 0.2", "\"maxRelativeSpeedMps\": [0.2]", 6,
         "/maxRelativeSpeedMps"},
        {"\"markerToleranceM\": 0.02", "\"markerToleranceM\": true", 7, "/markerToleranceM"},
        {"\"includeLowConfidence\": false", "\"includeLowConfidence\": 0", 8,
         "/includeLowConfidence"},
        {"\"includeLowConfidence\": false", "\"includeLowConfidence\": \"false\"", 8,
         "/includeLowConfidence"},
    });
}

TEST(DetectionProfileLoadErrors, UnknownKey)
{
    expectAllRejected({
        {"  \"distanceThresholdM\": 0.03,\n", "  \"bogus\": 1,\n  \"distanceThresholdM\": 0.03,\n",
         4, "/bogus"},
        {"\"includeLowConfidence\": false\n", "\"includeLowConfidence\": false,\n  \"extra\": {}\n",
         9, "/extra"},
        // A misspelled key is unknown, not the real key missing (the marker-file rule).
        {"\"minDurationS\"", "\"minDuration\"", 5, "/minDuration"},
    });
}

TEST(DetectionProfileLoadErrors, EveryNumberMustBePositive)
{
    expectAllRejected({
        {"\"distanceThresholdM\": 0.03", "\"distanceThresholdM\": 0", 4, "/distanceThresholdM"},
        {"\"distanceThresholdM\": 0.03", "\"distanceThresholdM\": -0.03", 4, "/distanceThresholdM"},
        {"\"minDurationS\": 0.25", "\"minDurationS\": 0", 5, "/minDurationS"},
        {"\"minDurationS\": 0.25", "\"minDurationS\": -1", 5, "/minDurationS"},
        {"\"maxRelativeSpeedMps\": 0.2", "\"maxRelativeSpeedMps\": 0", 6, "/maxRelativeSpeedMps"},
        {"\"maxRelativeSpeedMps\": 0.2", "\"maxRelativeSpeedMps\": -0.2", 6,
         "/maxRelativeSpeedMps"},
        {"\"markerToleranceM\": 0.02", "\"markerToleranceM\": 0", 7, "/markerToleranceM"},
        {"\"markerToleranceM\": 0.02", "\"markerToleranceM\": -0.02", 7, "/markerToleranceM"},
    });
}

TEST(DetectionProfileLoadErrors, NonFiniteNumbersAreRejected)
{
    expectAllRejected({
        {"\"distanceThresholdM\": 0.03", "\"distanceThresholdM\": 1e999", 4, "/distanceThresholdM"},
        {"\"minDurationS\": 0.25", "\"minDurationS\": 1e999", 5, "/minDurationS"},
        {"\"maxRelativeSpeedMps\": 0.2", "\"maxRelativeSpeedMps\": 1e999", 6,
         "/maxRelativeSpeedMps"},
        {"\"markerToleranceM\": 0.02", "\"markerToleranceM\": -1e999", 7, "/markerToleranceM"},
    });
}

TEST(DetectionProfileLoadErrors, ToStringIsFileLineFieldMessage)
{
    const DetectionProfileLoadResult result = loadDetectionProfile(
        replaceOnce(kDetectionText, "\"minDurationS\": 0.25", "\"minDurationS\": 0"), kFile);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error.toString(),
              std::string(kFile) + ":5: /minDurationS: " + result.error.message);
}

std::filesystem::path dataFile(const char* name)
{
    return std::filesystem::path(OPAS_TEST_DATA_DIR) / "detection" / name;
}

TEST(DetectionProfileLoadFile, ReadsAValidFile)
{
    const DetectionProfileLoadResult result =
        loadDetectionProfileFile(dataFile("default.detection.json"));
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == DetectionSettings{});
}

TEST(DetectionProfileLoadFile, ErrorsNameThePathAsPassedAndTheLine)
{
    const std::filesystem::path path = dataFile("bad_duration.detection.json");
    const DetectionProfileLoadResult result = loadDetectionProfileFile(path);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_EQ(result.error.line, 5U);
    EXPECT_EQ(result.error.field, "/minDurationS");
}

TEST(DetectionProfileLoadFile, MissingAndNonAsciiPathsAreErrorsNotExceptions)
{
    const std::filesystem::path path = dataFile("does_not_exist.detection.json");
    DetectionProfileLoadResult result;
    EXPECT_NO_THROW(result = loadDetectionProfileFile(path));
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_FALSE(result.error.message.empty());

    const std::filesystem::path unicode =
        std::filesystem::path(OPAS_TEST_DATA_DIR) / "detection" /
        std::filesystem::path(u8"does_not_exist_日本.detection.json");
    std::string utf8;
    for (const char8_t unit : unicode.u8string()) {
        utf8.push_back(static_cast<char>(unit));
    }
    EXPECT_NO_THROW(result = loadDetectionProfileFile(unicode));
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, utf8);
    EXPECT_EQ(result.error.field, "");
}

} // namespace
