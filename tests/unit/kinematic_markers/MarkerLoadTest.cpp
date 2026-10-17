#include "MarkerTestData.hpp"
#include "TestSupport.hpp"
#include "opas/markers/InteractionMarker.hpp"
#include "opas/markers/MarkerFile.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace {

using opas::markers::loadMarkerSet;
using opas::markers::loadMarkerSetFile;
using opas::markers::LoadResult;
using opas::markers::MarkerSet;
using opas::test::kEmptyMarkersText;
using opas::test::kOneMarkerText;
using opas::test::kTwoMarkerText;
using opas::test::oneMarkerSet;
using opas::test::twoMarkerSet;

constexpr const char* kFile = "synthetic.markers.json";

// Replaces the single occurrence of `from`. A test that names text which is not
// there exactly once is itself broken, so that fails loudly.
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

// Only call after checking ok(); value_or keeps static analysis happy about the optional.
MarkerSet valueOf(const LoadResult& result)
{
    return result.value.value_or(MarkerSet{});
}

void expectRejected(const std::string& text, std::size_t line, const std::string& field)
{
    const LoadResult result = loadMarkerSet(text, kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.value.has_value());
    EXPECT_EQ(result.error.file, kFile);
    EXPECT_EQ(result.error.line, line);
    EXPECT_EQ(result.error.field, field);
    EXPECT_FALSE(result.error.message.empty());
}

// ---- Valid files ---------------------------------------------------------------

TEST(MarkersLoad, CanonicalFileLoadsToTheExpectedValues)
{
    const LoadResult result = loadMarkerSet(kOneMarkerText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    ASSERT_EQ(valueOf(result).markers.size(), 1U);
    const MarkerSet set = valueOf(result);
    const auto& m = set.markers[0];
    EXPECT_EQ(m.id, "rightHandOnShoulder");
    EXPECT_EQ(m.sourceBone, "RightHand");
    EXPECT_EQ(m.targetRegion, "LeftUpperArm");
    EXPECT_EQ(m.offsetM.x, 0.02);
    EXPECT_EQ(m.offsetM.y, -0.01);
    EXPECT_EQ(m.offsetM.z, 0.12);
    EXPECT_EQ(m.contactDirection.x, 0.0);
    EXPECT_EQ(m.contactDirection.y, 0.0);
    EXPECT_EQ(m.contactDirection.z, -1.0);
    EXPECT_EQ(m.window.startS, 1.25);
    EXPECT_EQ(m.window.endS, 2.0);
    EXPECT_EQ(m.priority, 1);
    EXPECT_EQ(m.toleranceM, 0.02);
    EXPECT_TRUE(valueOf(result) == oneMarkerSet());
}

TEST(MarkersLoad, TwoMarkersKeepTheirOrder)
{
    const LoadResult result = loadMarkerSet(kTwoMarkerText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == twoMarkerSet());
    ASSERT_EQ(valueOf(result).markers.size(), 2U);
    EXPECT_EQ(valueOf(result).markers[0].id, "rightHandOnShoulder");
    EXPECT_EQ(valueOf(result).markers[1].id, "leftFootOnFloor");
    EXPECT_EQ(valueOf(result).markers[1].priority, -3);
}

TEST(MarkersLoad, EmptyMarkerArrayIsValid)
{
    const LoadResult result = loadMarkerSet(kEmptyMarkersText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result).markers.empty());
}

TEST(MarkersLoad, AcceptsValidJsonThatIsNotCanonical)
{
    // Test choice: the loader accepts any valid JSON layout, key order and number spelling.
    const std::string loose =
        "{\"markers\":[{\"toleranceM\":2e-2,\"priority\":1,\"window\":{\"endS\":2.0,\"startS\":1."
        "25},"
        "\"contactDirection\":[0.0,0.0,-1.0],\"offsetM\":[2E-2,-0.01,0.12],"
        "\"targetRegion\":\"LeftUpperArm\",\"sourceBone\":\"RightHand\","
        "\"id\":\"rightHandOnShoulder\"}],\"version\":1,\"format\":\"opas.markers\"}";
    const LoadResult result = loadMarkerSet(loose, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == oneMarkerSet());
}

TEST(MarkersLoad, WindowMayStartAtZeroAndMayBeEmpty)
{
    const std::string zeroStart = replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}",
                                              "{\"startS\": 0, \"endS\": 2}");
    EXPECT_TRUE(loadMarkerSet(zeroStart, kFile).ok());
    const std::string instant = replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}",
                                            "{\"startS\": 2, \"endS\": 2}");
    EXPECT_TRUE(loadMarkerSet(instant, kFile).ok());
}

TEST(MarkersLoad, ContactDirectionWithinOneMillionthOfUnitIsAccepted)
{
    // Length error 1e-7 < 1e-6.
    const std::string nearlyUnit = replaceOnce(kOneMarkerText, "[0, 0, -1]", "[0, 0, -1.0000001]");
    EXPECT_TRUE(loadMarkerSet(nearlyUnit, kFile).ok());
}

// ---- Rejections: every error carries file, 1-based line, JSON pointer ---------

TEST(MarkersLoadErrors, MalformedJsonReportsTheLineOfTheSyntaxError)
{
    // Stray character on line 3.
    expectRejected(replaceOnce(kOneMarkerText, "\"version\": 1,", "\"version\": @,"), 3, "");
    // Trailing comma inside an array on line 10.
    expectRejected(replaceOnce(kOneMarkerText, "[0, 0, -1],", "[0, 0, -1,],"), 10, "");
    // Stray character on line 13, so a later line is reported too.
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 0.02x"), 13,
                   "");
}

TEST(MarkersLoadErrors, EmptyOrNonJsonInputIsRejectedWithAnEmptyFieldPointer)
{
    const std::vector<std::string> inputs = {std::string(), std::string("not json at all\n")};
    for (const std::string& text : inputs) {
        SCOPED_TRACE(text);
        const LoadResult result = loadMarkerSet(text, kFile);
        EXPECT_FALSE(result.ok());
        EXPECT_EQ(result.error.file, kFile);
        EXPECT_GE(result.error.line, 1U);
        EXPECT_EQ(result.error.field, "");
        EXPECT_FALSE(result.error.message.empty());
    }
}

TEST(MarkersLoadErrors, TopLevelValueThatIsNotAnObject)
{
    expectRejected("[]\n", 1, "");
}

TEST(MarkersLoadErrors, WrongFormatString)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"opas.markers\"", "\"opas.other\""), 2, "/format");
}

TEST(MarkersLoadErrors, UnsupportedVersion)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"version\": 1,", "\"version\": 2,"), 3,
                   "/version");
    expectRejected(replaceOnce(kOneMarkerText, "\"version\": 1,", "\"version\": 0,"), 3,
                   "/version");
}

TEST(MarkersLoadErrors, MissingFieldReportsTheEnclosingObjectStartAndTheFieldPointer)
{
    // The top-level object starts on line 1.
    expectRejected(replaceOnce(kOneMarkerText, "  \"format\": \"opas.markers\",\n", ""), 1,
                   "/format");
    expectRejected(replaceOnce(kOneMarkerText, "  \"version\": 1,\n", ""), 1, "/version");
    expectRejected("{\n  \"format\": \"opas.markers\",\n  \"version\": 1\n}\n", 1, "/markers");

    // The marker object starts on line 5.
    const struct {
        const char* removedLine;
        const char* field;
    } missing[] = {
        {"      \"id\": \"rightHandOnShoulder\",\n", "/markers/0/id"},
        {"      \"sourceBone\": \"RightHand\",\n", "/markers/0/sourceBone"},
        {"      \"targetRegion\": \"LeftUpperArm\",\n", "/markers/0/targetRegion"},
        {"      \"offsetM\": [0.02, -0.01, 0.12],\n", "/markers/0/offsetM"},
        {"      \"contactDirection\": [0, 0, -1],\n", "/markers/0/contactDirection"},
        {"      \"window\": {\"startS\": 1.25, \"endS\": 2},\n", "/markers/0/window"},
        {"      \"priority\": 1,\n", "/markers/0/priority"},
    };
    for (const auto& c : missing) {
        SCOPED_TRACE(c.field);
        expectRejected(replaceOnce(kOneMarkerText, c.removedLine, ""), 5, c.field);
    }
    // The last field has no trailing comma, so remove it together with the comma before it.
    expectRejected(replaceOnce(kOneMarkerText, "\"priority\": 1,\n      \"toleranceM\": 0.02\n",
                               "\"priority\": 1\n"),
                   5, "/markers/0/toleranceM");

    // The window object starts on line 11.
    expectRejected(replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}", "{\"endS\": 2}"),
                   11, "/markers/0/window/startS");
    expectRejected(
        replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}", "{\"startS\": 1.25}"), 11,
        "/markers/0/window/endS");
}

TEST(MarkersLoadErrors, MissingFieldInTheSecondMarkerNamesThatMarker)
{
    // The second marker object starts on line 15.
    expectRejected(replaceOnce(kTwoMarkerText, "      \"priority\": -3,\n", ""), 15,
                   "/markers/1/priority");
}

TEST(MarkersLoadErrors, WrongTypeReportsTheLineWhereTheValueStarts)
{
    const struct {
        const char* from;
        const char* to;
        std::size_t line;
        const char* field;
    } cases[] = {
        {"\"format\": \"opas.markers\"", "\"format\": 1", 2, "/format"},
        {"\"version\": 1,", "\"version\": \"1\",", 3, "/version"},
        {"\"id\": \"rightHandOnShoulder\"", "\"id\": 5", 6, "/markers/0/id"},
        {"\"sourceBone\": \"RightHand\"", "\"sourceBone\": true", 7, "/markers/0/sourceBone"},
        {"\"targetRegion\": \"LeftUpperArm\"", "\"targetRegion\": null", 8,
         "/markers/0/targetRegion"},
        {"\"offsetM\": [0.02, -0.01, 0.12]", "\"offsetM\": 5", 9, "/markers/0/offsetM"},
        // Test choice: a bad element is reported at the element, a wrong length at the array.
        {"[0.02, -0.01, 0.12]", "[0.02, \"a\", 0.12]", 9, "/markers/0/offsetM/1"},
        {"[0.02, -0.01, 0.12]", "[0.02, -0.01]", 9, "/markers/0/offsetM"},
        {"[0.02, -0.01, 0.12]", "[0.02, -0.01, 0.12, 4]", 9, "/markers/0/offsetM"},
        {"[0, 0, -1]", "[0, 0, \"z\"]", 10, "/markers/0/contactDirection/2"},
        {"[0, 0, -1]", "[0, 0]", 10, "/markers/0/contactDirection"},
        {"\"window\": {\"startS\": 1.25, \"endS\": 2}", "\"window\": [1, 2]", 11,
         "/markers/0/window"},
        {"\"startS\": 1.25", "\"startS\": \"1.25\"", 11, "/markers/0/window/startS"},
        {"\"endS\": 2}", "\"endS\": null}", 11, "/markers/0/window/endS"},
        {"\"priority\": 1,", "\"priority\": \"high\",", 12, "/markers/0/priority"},
        {"\"priority\": 1,", "\"priority\": 1.5,", 12, "/markers/0/priority"},
        {"\"toleranceM\": 0.02", "\"toleranceM\": \"0.02\"", 13, "/markers/0/toleranceM"},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(std::string(c.from) + " -> " + c.to);
        expectRejected(replaceOnce(kOneMarkerText, c.from, c.to), c.line, c.field);
    }
}

TEST(MarkersLoadErrors, MarkersThatIsNotAnArrayOrHoldsANonObject)
{
    expectRejected("{\n  \"format\": \"opas.markers\",\n  \"version\": 1,\n  \"markers\": {}\n}\n",
                   4, "/markers");
    expectRejected(
        "{\n  \"format\": \"opas.markers\",\n  \"version\": 1,\n  \"markers\": [\n    5\n  "
        "]\n}\n",
        5, "/markers/0");
}

TEST(MarkersLoadErrors, UnknownKeyReportsTheLineWhereItsValueStarts)
{
    // Top level, new line 4.
    expectRejected(
        replaceOnce(kOneMarkerText, "  \"markers\": [\n", "  \"extra\": true,\n  \"markers\": [\n"),
        4, "/extra");
    // Inside the marker, new line 7.
    expectRejected(replaceOnce(kOneMarkerText, "      \"sourceBone\"",
                               "      \"bogus\": 1,\n      \"sourceBone\""),
                   7, "/markers/0/bogus");
    // Inside the window, line 11.
    expectRejected(replaceOnce(kOneMarkerText, "\"endS\": 2}", "\"endS\": 2, \"x\": 3}"), 11,
                   "/markers/0/window/x");
}

TEST(MarkersLoadErrors, EmptyStringsAreRejected)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"id\": \"rightHandOnShoulder\"", "\"id\": \"\""),
                   6, "/markers/0/id");
    // The format document also requires non-empty bone and region names.
    expectRejected(
        replaceOnce(kOneMarkerText, "\"sourceBone\": \"RightHand\"", "\"sourceBone\": \"\""), 7,
        "/markers/0/sourceBone");
    expectRejected(
        replaceOnce(kOneMarkerText, "\"targetRegion\": \"LeftUpperArm\"", "\"targetRegion\": \"\""),
        8, "/markers/0/targetRegion");
}

TEST(MarkersLoadErrors, DuplicateIdIsReportedAtTheSecondOccurrence)
{
    // The second marker's id value is on line 16.
    expectRejected(replaceOnce(kTwoMarkerText, "\"id\": \"leftFootOnFloor\"",
                               "\"id\": \"rightHandOnShoulder\""),
                   16, "/markers/1/id");
}

TEST(MarkersLoadErrors, ContactDirectionThatIsNotUnitLength)
{
    const char* bad[] = {"[0, 0, -1.001]", "[0, 0, 0]", "[1, 1, 0]", "[0, 0, 1.00001]",
                         "[0, 0, 2]"};
    for (const char* direction : bad) {
        SCOPED_TRACE(direction);
        expectRejected(replaceOnce(kOneMarkerText, "[0, 0, -1]", direction), 10,
                       "/markers/0/contactDirection");
    }
}

TEST(MarkersLoadErrors, WindowStartBelowZero)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"startS\": 1.25", "\"startS\": -0.5"), 11,
                   "/markers/0/window/startS");
}

TEST(MarkersLoadErrors, WindowStartAfterEnd)
{
    // Test choice: the pointer names startS (the spec says "startS > endS").
    expectRejected(replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}",
                               "{\"startS\": 3, \"endS\": 2}"),
                   11, "/markers/0/window/startS");
}

TEST(MarkersLoadErrors, ToleranceMustBePositive)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 0"), 13,
                   "/markers/0/toleranceM");
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": -0.01"), 13,
                   "/markers/0/toleranceM");
}

TEST(MarkersLoadErrors, NonFiniteNumbersAreRejected)
{
    // JSON has no NaN or Infinity literal; an out-of-range exponent parses to infinity.
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 1e999"), 13,
                   "/markers/0/toleranceM");
    expectRejected(replaceOnce(kOneMarkerText, "[0.02, -0.01, 0.12]", "[0.02, -1e999, 0.12]"), 9,
                   "/markers/0/offsetM/1");
    expectRejected(replaceOnce(kOneMarkerText, "[0, 0, -1]", "[1e999, 0, 0]"), 10,
                   "/markers/0/contactDirection/0");
    expectRejected(replaceOnce(kOneMarkerText, "\"endS\": 2}", "\"endS\": 1e999}"), 11,
                   "/markers/0/window/endS");
}

TEST(MarkersLoadErrors, ErrorsInALaterMarkerUseThatMarkersIndexAndLine)
{
    // The second marker's toleranceM is on line 23.
    expectRejected(replaceOnce(kTwoMarkerText, "\"toleranceM\": 0.005", "\"toleranceM\": 0"), 23,
                   "/markers/1/toleranceM");
}

TEST(MarkersLoadErrors, ToStringIsFileLineFieldMessage)
{
    const LoadResult result = loadMarkerSet(
        replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 0"), kFile);
    ASSERT_FALSE(result.ok());
    const std::string expected =
        std::string(kFile) + ":13: /markers/0/toleranceM: " + result.error.message;
    EXPECT_EQ(result.error.toString(), expected);
}

// ---- Files on disk ---------------------------------------------------------------

std::filesystem::path dataFile(const char* name)
{
    return std::filesystem::path(OPAS_TEST_DATA_DIR) / "markers" / name;
}

TEST(MarkersLoadFile, ReadsAValidFileFromDisk)
{
    const LoadResult result = loadMarkerSetFile(dataFile("two_markers.markers.json"));
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == twoMarkerSet());
}

TEST(MarkersLoadFile, ErrorsNameThePathAsPassedAndTheLine)
{
    const std::filesystem::path path = dataFile("bad_tolerance.markers.json");
    const LoadResult result = loadMarkerSetFile(path);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_EQ(result.error.line, 13U);
    EXPECT_EQ(result.error.field, "/markers/0/toleranceM");
}

TEST(MarkersLoadFile, MissingFileIsAnErrorNotAnException)
{
    const std::filesystem::path path = dataFile("does_not_exist.markers.json");
    const LoadResult result = loadMarkerSetFile(path);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_EQ(result.error.field, "");
    EXPECT_FALSE(result.error.message.empty());
}

} // namespace
