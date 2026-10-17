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

// Line numbers below refer to kOneMarkerText / kTwoMarkerText (see MarkerTestData.hpp):
//   one marker:  5 "{", 6 id, 7 sourceActor, 8 targetActor, 9 sourceBone, 10 targetRegion,
//                11 offsetM, 12 contactDirection, 13 window, 14 priority, 15 toleranceM
//   second marker (kTwoMarkerText): 17 "{", 18 id, 19 sourceActor, 20 targetActor, 21 sourceBone,
//                22 targetRegion, 23 offsetM, 24 contactDirection, 25 window, 26 priority,
//                27 toleranceM

// ---- Valid files ---------------------------------------------------------------

TEST(MarkersLoad, CanonicalFileLoadsToTheExpectedValues)
{
    const LoadResult result = loadMarkerSet(kOneMarkerText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    ASSERT_EQ(valueOf(result).markers.size(), 1U);
    const MarkerSet set = valueOf(result);
    const auto& m = set.markers[0];
    EXPECT_EQ(m.id, "rightHandOnShoulder");
    EXPECT_EQ(m.sourceActor, 0);
    EXPECT_EQ(m.targetActor, 1);
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
    // The second marker is authored the other way round: actor 1 acts on actor 0.
    EXPECT_EQ(valueOf(result).markers[1].sourceActor, 1);
    EXPECT_EQ(valueOf(result).markers[1].targetActor, 0);
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
        "\"targetActor\":1,\"sourceActor\":0,"
        "\"id\":\"rightHandOnShoulder\"}],\"version\":2,\"format\":\"opas.markers\"}";
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

TEST(MarkersLoad, ActorIndicesAboveOneAreAcceptedByTheFormat)
{
    // The format allows any non-negative indices (room for more actors later); only the
    // solver is limited to two. Test choice: 5 and 3.
    std::string text = replaceOnce(kOneMarkerText, "\"sourceActor\": 0", "\"sourceActor\": 5");
    text = replaceOnce(text, "\"targetActor\": 1", "\"targetActor\": 3");
    const LoadResult result = loadMarkerSet(text, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_EQ(valueOf(result).markers[0].sourceActor, 5);
    EXPECT_EQ(valueOf(result).markers[0].targetActor, 3);
}

// ---- Rejections: every error carries file, 1-based line, JSON pointer ---------

TEST(MarkersLoadErrors, MalformedJsonReportsTheLineOfTheSyntaxError)
{
    // Stray character on line 3.
    expectRejected(replaceOnce(kOneMarkerText, "\"version\": 2,", "\"version\": @,"), 3, "");
    // Trailing comma inside an array on line 12.
    expectRejected(replaceOnce(kOneMarkerText, "[0, 0, -1],", "[0, 0, -1,],"), 12, "");
    // Stray character on line 15, so a later line is reported too.
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 0.02x"), 15,
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
    for (const char* version : {"1", "0", "3"}) {
        SCOPED_TRACE(version);
        expectRejected(replaceOnce(kOneMarkerText, "\"version\": 2,",
                                   std::string("\"version\": ") + version + ","),
                       3, "/version");
    }
}

TEST(MarkersLoadErrors, VersionOneFileIsNoLongerReadEvenWhenItIsAValidV1Document)
{
    // A complete v1 file (no actor fields). It must be rejected at /version, line 3, before
    // anything else is looked at.
    const std::string v1 = R"json({
  "format": "opas.markers",
  "version": 1,
  "markers": [
    {
      "id": "rightHandOnShoulder",
      "sourceBone": "RightHand",
      "targetRegion": "LeftUpperArm",
      "offsetM": [0.02, -0.01, 0.12],
      "contactDirection": [0, 0, -1],
      "window": {"startS": 1.25, "endS": 2},
      "priority": 1,
      "toleranceM": 0.02
    }
  ]
}
)json";
    expectRejected(v1, 3, "/version");
}

TEST(MarkersLoadErrors, MissingFieldReportsTheEnclosingObjectStartAndTheFieldPointer)
{
    // The top-level object starts on line 1.
    expectRejected(replaceOnce(kOneMarkerText, "  \"format\": \"opas.markers\",\n", ""), 1,
                   "/format");
    expectRejected(replaceOnce(kOneMarkerText, "  \"version\": 2,\n", ""), 1, "/version");
    expectRejected("{\n  \"format\": \"opas.markers\",\n  \"version\": 2\n}\n", 1, "/markers");

    // The marker object starts on line 5.
    const struct {
        const char* removedLine;
        const char* field;
    } missing[] = {
        {"      \"id\": \"rightHandOnShoulder\",\n", "/markers/0/id"},
        {"      \"sourceActor\": 0,\n", "/markers/0/sourceActor"},
        {"      \"targetActor\": 1,\n", "/markers/0/targetActor"},
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

    // The window object starts on line 13.
    expectRejected(replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}", "{\"endS\": 2}"),
                   13, "/markers/0/window/startS");
    expectRejected(
        replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}", "{\"startS\": 1.25}"), 13,
        "/markers/0/window/endS");
}

TEST(MarkersLoadErrors, MissingFieldInTheSecondMarkerNamesThatMarker)
{
    // The second marker object starts on line 17.
    expectRejected(replaceOnce(kTwoMarkerText, "      \"priority\": -3,\n", ""), 17,
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
        {"\"version\": 2,", "\"version\": \"2\",", 3, "/version"},
        {"\"id\": \"rightHandOnShoulder\"", "\"id\": 5", 6, "/markers/0/id"},
        {"\"sourceBone\": \"RightHand\"", "\"sourceBone\": true", 9, "/markers/0/sourceBone"},
        {"\"targetRegion\": \"LeftUpperArm\"", "\"targetRegion\": null", 10,
         "/markers/0/targetRegion"},
        {"\"offsetM\": [0.02, -0.01, 0.12]", "\"offsetM\": 5", 11, "/markers/0/offsetM"},
        // Test choice: a bad element is reported at the element, a wrong length at the array.
        {"[0.02, -0.01, 0.12]", "[0.02, \"a\", 0.12]", 11, "/markers/0/offsetM/1"},
        {"[0.02, -0.01, 0.12]", "[0.02, -0.01]", 11, "/markers/0/offsetM"},
        {"[0.02, -0.01, 0.12]", "[0.02, -0.01, 0.12, 4]", 11, "/markers/0/offsetM"},
        {"[0, 0, -1]", "[0, 0, \"z\"]", 12, "/markers/0/contactDirection/2"},
        {"[0, 0, -1]", "[0, 0]", 12, "/markers/0/contactDirection"},
        {"\"window\": {\"startS\": 1.25, \"endS\": 2}", "\"window\": [1, 2]", 13,
         "/markers/0/window"},
        {"\"startS\": 1.25", "\"startS\": \"1.25\"", 13, "/markers/0/window/startS"},
        {"\"endS\": 2}", "\"endS\": null}", 13, "/markers/0/window/endS"},
        {"\"priority\": 1,", "\"priority\": \"high\",", 14, "/markers/0/priority"},
        {"\"priority\": 1,", "\"priority\": 1.5,", 14, "/markers/0/priority"},
        {"\"toleranceM\": 0.02", "\"toleranceM\": \"0.02\"", 15, "/markers/0/toleranceM"},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(std::string(c.from) + " -> " + c.to);
        expectRejected(replaceOnce(kOneMarkerText, c.from, c.to), c.line, c.field);
    }
}

TEST(MarkersLoadErrors, MarkersThatIsNotAnArrayOrHoldsANonObject)
{
    expectRejected("{\n  \"format\": \"opas.markers\",\n  \"version\": 2,\n  \"markers\": {}\n}\n",
                   4, "/markers");
    expectRejected(
        "{\n  \"format\": \"opas.markers\",\n  \"version\": 2,\n  \"markers\": [\n    5\n  "
        "]\n}\n",
        5, "/markers/0");
}

TEST(MarkersLoadErrors, UnknownKeyReportsTheLineWhereItsValueStarts)
{
    // Top level, new line 4.
    expectRejected(
        replaceOnce(kOneMarkerText, "  \"markers\": [\n", "  \"extra\": true,\n  \"markers\": [\n"),
        4, "/extra");
    // Inside the marker, new line 9 (after the two actor lines).
    expectRejected(replaceOnce(kOneMarkerText, "      \"sourceBone\"",
                               "      \"bogus\": 1,\n      \"sourceBone\""),
                   9, "/markers/0/bogus");
    // Inside the window, line 13.
    expectRejected(replaceOnce(kOneMarkerText, "\"endS\": 2}", "\"endS\": 2, \"x\": 3}"), 13,
                   "/markers/0/window/x");
}

TEST(MarkersLoadErrors, EmptyStringsAreRejected)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"id\": \"rightHandOnShoulder\"", "\"id\": \"\""),
                   6, "/markers/0/id");
    // The format document also requires non-empty bone and region names.
    expectRejected(
        replaceOnce(kOneMarkerText, "\"sourceBone\": \"RightHand\"", "\"sourceBone\": \"\""), 9,
        "/markers/0/sourceBone");
    expectRejected(
        replaceOnce(kOneMarkerText, "\"targetRegion\": \"LeftUpperArm\"", "\"targetRegion\": \"\""),
        10, "/markers/0/targetRegion");
}

TEST(MarkersLoadErrors, DuplicateIdIsReportedAtTheSecondOccurrence)
{
    // The second marker's id value is on line 18.
    expectRejected(replaceOnce(kTwoMarkerText, "\"id\": \"leftFootOnFloor\"",
                               "\"id\": \"rightHandOnShoulder\""),
                   18, "/markers/1/id");
}

TEST(MarkersLoadErrors, ContactDirectionThatIsNotUnitLength)
{
    const char* bad[] = {"[0, 0, -1.001]", "[0, 0, 0]", "[1, 1, 0]", "[0, 0, 1.00001]",
                         "[0, 0, 2]"};
    for (const char* direction : bad) {
        SCOPED_TRACE(direction);
        expectRejected(replaceOnce(kOneMarkerText, "[0, 0, -1]", direction), 12,
                       "/markers/0/contactDirection");
    }
}

TEST(MarkersLoadErrors, WindowStartBelowZero)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"startS\": 1.25", "\"startS\": -0.5"), 13,
                   "/markers/0/window/startS");
}

TEST(MarkersLoadErrors, WindowStartAfterEnd)
{
    // Test choice: the pointer names startS (the spec says "startS > endS").
    expectRejected(replaceOnce(kOneMarkerText, "{\"startS\": 1.25, \"endS\": 2}",
                               "{\"startS\": 3, \"endS\": 2}"),
                   13, "/markers/0/window/startS");
}

TEST(MarkersLoadErrors, ToleranceMustBePositive)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 0"), 15,
                   "/markers/0/toleranceM");
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": -0.01"), 15,
                   "/markers/0/toleranceM");
}

TEST(MarkersLoadErrors, NonFiniteNumbersAreRejected)
{
    // JSON has no NaN or Infinity literal; an out-of-range exponent parses to infinity.
    expectRejected(replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 1e999"), 15,
                   "/markers/0/toleranceM");
    expectRejected(replaceOnce(kOneMarkerText, "[0.02, -0.01, 0.12]", "[0.02, -1e999, 0.12]"), 11,
                   "/markers/0/offsetM/1");
    expectRejected(replaceOnce(kOneMarkerText, "[0, 0, -1]", "[1e999, 0, 0]"), 12,
                   "/markers/0/contactDirection/0");
    expectRejected(replaceOnce(kOneMarkerText, "\"endS\": 2}", "\"endS\": 1e999}"), 13,
                   "/markers/0/window/endS");
}

TEST(MarkersLoadErrors, ErrorsInALaterMarkerUseThatMarkersIndexAndLine)
{
    // The second marker's toleranceM is on line 27.
    expectRejected(replaceOnce(kTwoMarkerText, "\"toleranceM\": 0.005", "\"toleranceM\": 0"), 27,
                   "/markers/1/toleranceM");
}

TEST(MarkersLoadErrors, ToStringIsFileLineFieldMessage)
{
    const LoadResult result = loadMarkerSet(
        replaceOnce(kOneMarkerText, "\"toleranceM\": 0.02", "\"toleranceM\": 0"), kFile);
    ASSERT_FALSE(result.ok());
    const std::string expected =
        std::string(kFile) + ":15: /markers/0/toleranceM: " + result.error.message;
    EXPECT_EQ(result.error.toString(), expected);
}

// ---- Format v2: actor fields -------------------------------------------------------

TEST(MarkersLoadErrors, ActorFieldMissingIsReportedAtTheMarkerObject)
{
    // Covered with the other required fields above; here also in the second marker.
    expectRejected(replaceOnce(kTwoMarkerText, "      \"sourceActor\": 1,\n", ""), 17,
                   "/markers/1/sourceActor");
    expectRejected(replaceOnce(kTwoMarkerText, "      \"targetActor\": 0,\n", ""), 17,
                   "/markers/1/targetActor");
}

TEST(MarkersLoadErrors, ActorIndexThatIsNotAnIntegerIsAWrongType)
{
    const struct {
        const char* from;
        const char* to;
        std::size_t line;
        const char* field;
    } cases[] = {
        {"\"sourceActor\": 0", "\"sourceActor\": \"0\"", 7, "/markers/0/sourceActor"},
        {"\"sourceActor\": 0", "\"sourceActor\": 0.5", 7, "/markers/0/sourceActor"},
        {"\"sourceActor\": 0", "\"sourceActor\": null", 7, "/markers/0/sourceActor"},
        {"\"sourceActor\": 0", "\"sourceActor\": true", 7, "/markers/0/sourceActor"},
        {"\"targetActor\": 1", "\"targetActor\": \"1\"", 8, "/markers/0/targetActor"},
        {"\"targetActor\": 1", "\"targetActor\": 1.5", 8, "/markers/0/targetActor"},
        {"\"targetActor\": 1", "\"targetActor\": [1]", 8, "/markers/0/targetActor"},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(std::string(c.from) + " -> " + c.to);
        expectRejected(replaceOnce(kOneMarkerText, c.from, c.to), c.line, c.field);
    }
}

TEST(MarkersLoadErrors, NegativeActorIndexIsRejected)
{
    expectRejected(replaceOnce(kOneMarkerText, "\"sourceActor\": 0", "\"sourceActor\": -1"), 7,
                   "/markers/0/sourceActor");
    expectRejected(replaceOnce(kOneMarkerText, "\"targetActor\": 1", "\"targetActor\": -2"), 8,
                   "/markers/0/targetActor");
}

TEST(MarkersLoadErrors, TargetActorEqualToSourceActorIsRejectedAtTargetActor)
{
    // Test choice: the pointer names targetActor (the second of the two, which "must differ").
    expectRejected(replaceOnce(kOneMarkerText, "\"targetActor\": 1", "\"targetActor\": 0"), 8,
                   "/markers/0/targetActor");
    // In the second marker (source 1, target 0): make the target 1.
    expectRejected(replaceOnce(kTwoMarkerText, "\"targetActor\": 0", "\"targetActor\": 1"), 20,
                   "/markers/1/targetActor");
    // Both equal and non-zero.
    std::string both = replaceOnce(kOneMarkerText, "\"sourceActor\": 0", "\"sourceActor\": 2");
    both = replaceOnce(both, "\"targetActor\": 1", "\"targetActor\": 2");
    expectRejected(both, 8, "/markers/0/targetActor");
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
    EXPECT_EQ(result.error.line, 15U);
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

// ---- Amendments after the fresh review: non-ASCII paths ----------------------------

TEST(MarkersLoadFile, MissingFileWithNonAsciiPathReportsTheUtf8PathAndDoesNotThrow)
{
    const std::filesystem::path path = std::filesystem::path(OPAS_TEST_DATA_DIR) / "markers" /
                                       std::filesystem::path(u8"does_not_exist_日本.markers.json");
    // The expected text is the path as UTF-8 bytes, whatever the platform's native encoding.
    std::string utf8;
    for (const char8_t unit : path.u8string()) {
        utf8.push_back(static_cast<char>(unit));
    }

    LoadResult result;
    EXPECT_NO_THROW(result = loadMarkerSetFile(path));
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, utf8);
    EXPECT_EQ(result.error.field, "");
    EXPECT_FALSE(result.error.message.empty());
}

} // namespace
