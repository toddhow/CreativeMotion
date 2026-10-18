#include "BodyMapTestData.hpp"
#include "TestSupport.hpp"
#include "opas/markers/BodyMapFile.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/markers/MarkerFile.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace {

using opas::markers::BodyMapLoadResult;
using opas::markers::BodyMapping;
using opas::markers::loadBodyMap;
using opas::markers::loadBodyMapFile;
using opas::test::bodyMapOfText;
using opas::test::kBodyMapText;
using opas::test::kEmptyBodyMapText;
using opas::test::sameMapping;

constexpr const char* kFile = "synthetic.bodymap.json";

// Replaces the single occurrence of `from`; a test naming text that is not there exactly once
// is itself broken, so that fails loudly.
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
BodyMapping valueOf(const BodyMapLoadResult& result)
{
    return result.value.value_or(BodyMapping{});
}

void expectRejected(const std::string& text, std::size_t line, const std::string& field)
{
    const BodyMapLoadResult result = loadBodyMap(text, kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.value.has_value());
    EXPECT_EQ(result.error.file, kFile);
    EXPECT_EQ(result.error.line, line);
    EXPECT_EQ(result.error.field, field);
    EXPECT_FALSE(result.error.message.empty());
}

// A table row: replace `from` by `to` in kBodyMapText; expect this line and pointer.
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
        expectRejected(replaceOnce(kBodyMapText, e.from, e.to), e.line, e.field);
    }
}

// Line numbers refer to kBodyMapText (see BodyMapTestData.hpp): limbs 5-6, capsules 9-10,
// chain 0 at 13 (name 14, rootBone 15, midBone 16, endBone 17, minBendRad 18, maxBendRad 19,
// poleDirection 20, limbRadiusM 21), chain 1 at 23 (name 24 ... endBone 27, minBendRad 28 ...).

// ---- Valid files ---------------------------------------------------------------------------

TEST(BodyMapLoad, CanonicalFileLoadsToTheExpectedMapping)
{
    const BodyMapLoadResult result = loadBodyMap(kBodyMapText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    const BodyMapping m = valueOf(result);
    EXPECT_TRUE(sameMapping(m, bodyMapOfText()));

    ASSERT_EQ(m.limbs.size(), 2U);
    EXPECT_EQ(m.limbs[0].name, "RightArm");
    EXPECT_EQ(m.limbs[0].bones.size(), 3U);
    ASSERT_EQ(m.capsules.size(), 2U);
    EXPECT_EQ(m.capsules[1].region, "UpperBack");
    EXPECT_EQ(m.capsules[1].radiusRatio, 0.1667);
    ASSERT_EQ(m.ikChains.size(), 2U);
    EXPECT_EQ(m.ikChains[0].limits.minBendRad, 0.0);
    EXPECT_EQ(m.ikChains[0].limits.maxBendRad, 2.6);
    EXPECT_EQ(m.ikChains[1].poleDirection.x, -0.35);
    EXPECT_EQ(m.ikChains[1].limbRadiusM, 0.03);
}

TEST(BodyMapLoad, EmptyArraysAreValid)
{
    const BodyMapLoadResult result = loadBodyMap(kEmptyBodyMapText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    const BodyMapping m = valueOf(result);
    EXPECT_TRUE(m.limbs.empty());
    EXPECT_TRUE(m.capsules.empty());
    EXPECT_TRUE(m.ikChains.empty());
}

TEST(BodyMapLoad, AcceptsValidJsonThatIsNotCanonical)
{
    // Test choice: any valid JSON layout, key order and number spelling is accepted.
    const std::string loose =
        "{\"ikChains\":[{\"limbRadiusM\":4e-2,\"poleDirection\":[0.0,-1.0,0.0],"
        "\"maxBendRad\":2.6,\"minBendRad\":0.0,\"endBone\":\"RightHand\",\"midBone\":"
        "\"RightForearm\",\"rootBone\":\"RightUpperArm\",\"name\":\"RightArm\"}],"
        "\"capsules\":[{\"radiusRatio\":1.5E-1,\"endBone\":\"RightHand\",\"bone\":"
        "\"RightForearm\",\"region\":\"RightForearm\"}],"
        "\"limbs\":[{\"bones\":[\"RightUpperArm\",\"RightForearm\",\"RightHand\"],\"name\":"
        "\"RightArm\"}],\"version\":1,\"format\":\"opas.bodymap\"}";
    const BodyMapLoadResult result = loadBodyMap(loose, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    const BodyMapping m = valueOf(result);
    ASSERT_EQ(m.ikChains.size(), 1U);
    EXPECT_EQ(m.ikChains[0].limbRadiusM, 0.04);
    EXPECT_EQ(m.capsules.at(0).radiusRatio, 0.15);
}

TEST(BodyMapLoad, ChainWithBendRangeEdgesAndPartlyZeroPoleIsAccepted)
{
    // min == max, max exactly pi, and a pole with zero components are all fine.
    std::string text =
        replaceOnce(kBodyMapText, "\"minBendRad\": 0,", "\"minBendRad\": 3.14159265358979323846,");
    text = replaceOnce(text, "\"maxBendRad\": 2.6,", "\"maxBendRad\": 3.14159265358979323846,");
    text = replaceOnce(text, "[0, -1, 0]", "[0, 0, 5]");
    const BodyMapLoadResult result = loadBodyMap(text, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_EQ(valueOf(result).ikChains[0].poleDirection.z, 5.0);
}

TEST(BodyMapLoad, ABoneMayAppearInSeveralLimbsAndNamesAreNotCheckedAgainstASkeleton)
{
    // Out of scope for the loader: bone names are checked when the map is used. The same
    // bones in two limbs, and chain bones that are not parent and child, load fine.
    std::string text =
        replaceOnce(kBodyMapText, "\"midBone\": \"RightForearm\"", "\"midBone\": \"NoSuchBone\"");
    text = replaceOnce(
        text,
        "{\"name\": \"LeftArm\", \"bones\": [\"LeftUpperArm\", \"LeftForearm\", \"LeftHand\"]}",
        "{\"name\": \"LeftArm\", \"bones\": [\"RightUpperArm\", \"RightForearm\"]}");
    const BodyMapLoadResult result = loadBodyMap(text, kFile);
    EXPECT_TRUE(result.ok()) << result.error.toString();
}

TEST(BodyMapLoad, ALimbAndARegionMayShareAName)
{
    // A limb and a chain may share a name; a capsule region may equal a limb name.
    const std::string text =
        replaceOnce(kBodyMapText, "\"region\": \"RightForearm\"", "\"region\": \"RightArm\"");
    EXPECT_TRUE(loadBodyMap(text, kFile).ok());
}

// ---- Rejections: format, version, structure -------------------------------------------------

TEST(BodyMapLoadErrors, MalformedJsonReportsTheLineOfTheSyntaxError)
{
    expectAllRejected({
        {"\"version\": 1,", "\"version\": @,", 3, ""},
        {"[0, -1, 0],", "[0, -1, 0,],", 20, ""},
        {"\"limbRadiusM\": 0.04", "\"limbRadiusM\": 0.04x", 21, ""},
        {"\"radiusRatio\": 0.1667}", "\"radiusRatio\": 0.1667", 11,
         ""}, // no closing brace: the error is at the next token, the "]" on line 11
    });
}

TEST(BodyMapLoadErrors, EmptyOrNonJsonInputIsRejectedWithAnEmptyFieldPointer)
{
    const std::vector<std::string> inputs = {std::string(), std::string("not json at all\n")};
    for (const std::string& text : inputs) {
        SCOPED_TRACE(text);
        const BodyMapLoadResult result = loadBodyMap(text, kFile);
        EXPECT_FALSE(result.ok());
        EXPECT_EQ(result.error.file, kFile);
        EXPECT_GE(result.error.line, 1U);
        EXPECT_EQ(result.error.field, "");
        EXPECT_FALSE(result.error.message.empty());
    }
}

TEST(BodyMapLoadErrors, TopLevelValueThatIsNotAnObject)
{
    expectRejected("[]\n", 1, "");
    expectRejected("5\n", 1, "");
}

TEST(BodyMapLoadErrors, WrongFormatAndVersion)
{
    expectAllRejected({
        {"\"opas.bodymap\"", "\"opas.markers\"", 2, "/format"},
        {"\"opas.bodymap\"", "\"\"", 2, "/format"},
        {"\"format\": \"opas.bodymap\"", "\"format\": 1", 2, "/format"},
        // Version 2 (fingers) is valid since A5c (toddhow approved, 2026-10-17); 3 is not.
        {"\"version\": 1,", "\"version\": 3,", 3, "/version"},
        {"\"version\": 1,", "\"version\": 0,", 3, "/version"},
        {"\"version\": 1,", "\"version\": \"1\",", 3, "/version"},
        {"\"version\": 1,", "\"version\": 1.5,", 3, "/version"},
    });
}

TEST(BodyMapLoadErrors, MissingTopLevelFieldReportsLineOneAndTheFieldPointer)
{
    const std::string head = "{\n  \"format\": \"opas.bodymap\",\n  \"version\": 1,\n";
    expectRejected(head + "  \"capsules\": [],\n  \"ikChains\": []\n}\n", 1, "/limbs");
    expectRejected(head + "  \"limbs\": [],\n  \"ikChains\": []\n}\n", 1, "/capsules");
    expectRejected(head + "  \"limbs\": [],\n  \"capsules\": []\n}\n", 1, "/ikChains");
    expectRejected(
        "{\n  \"version\": 1,\n  \"limbs\": [],\n  \"capsules\": [],\n  \"ikChains\": []\n}\n", 1,
        "/format");
    expectRejected("{\n  \"format\": \"opas.bodymap\",\n  \"limbs\": [],\n  \"capsules\": [],\n  "
                   "\"ikChains\": []\n}\n",
                   1, "/version");
}

TEST(BodyMapLoadErrors, MissingFieldInAnEntryReportsTheLineWhereThatEntryStarts)
{
    expectAllRejected({
        // Limb at line 5.
        {"\"name\": \"RightArm\", \"bones\"", "\"bones\"", 5, "/limbs/0/name"},
        {", \"bones\": [\"RightUpperArm\", \"RightForearm\", \"RightHand\"]}", "}", 5,
         "/limbs/0/bones"},
        // Capsule at line 9.
        {"\"region\": \"RightForearm\", ", "", 9, "/capsules/0/region"},
        {"\"bone\": \"RightForearm\", ", "", 9, "/capsules/0/bone"},
        {"\"endBone\": \"RightHand\", ", "", 9, "/capsules/0/endBone"},
        {", \"radiusRatio\": 0.15", "", 9, "/capsules/0/radiusRatio"},
        // Second capsule at line 10.
        {"\"region\": \"UpperBack\", ", "", 10, "/capsules/1/region"},
        // Chain 0 at line 13.
        {"      \"name\": \"RightArm\",\n", "", 13, "/ikChains/0/name"},
        {"      \"rootBone\": \"RightUpperArm\",\n", "", 13, "/ikChains/0/rootBone"},
        {"      \"midBone\": \"RightForearm\",\n", "", 13, "/ikChains/0/midBone"},
        {"      \"endBone\": \"RightHand\",\n", "", 13, "/ikChains/0/endBone"},
        {"      \"minBendRad\": 0,\n", "", 13, "/ikChains/0/minBendRad"},
        {"      \"maxBendRad\": 2.6,\n", "", 13, "/ikChains/0/maxBendRad"},
        {"      \"poleDirection\": [0, -1, 0],\n", "", 13, "/ikChains/0/poleDirection"},
        // The last key has no trailing comma: remove it with the comma before it.
        {"\"poleDirection\": [0, -1, 0],\n      \"limbRadiusM\": 0.04\n",
         "\"poleDirection\": [0, -1, 0]\n", 13, "/ikChains/0/limbRadiusM"},
        // Chain 1 at line 23.
        {"      \"midBone\": \"LeftForearm\",\n", "", 23, "/ikChains/1/midBone"},
    });
}

TEST(BodyMapLoadErrors, TopLevelArraysThatAreNotArraysOrHoldNonObjects)
{
    const std::string head = "{\n  \"format\": \"opas.bodymap\",\n  \"version\": 1,\n";
    expectRejected(head + "  \"limbs\": {},\n  \"capsules\": [],\n  \"ikChains\": []\n}\n", 4,
                   "/limbs");
    expectRejected(head + "  \"limbs\": [],\n  \"capsules\": \"x\",\n  \"ikChains\": []\n}\n", 5,
                   "/capsules");
    expectRejected(head + "  \"limbs\": [],\n  \"capsules\": [],\n  \"ikChains\": 7\n}\n", 6,
                   "/ikChains");
    expectRejected(head + "  \"limbs\": [5],\n  \"capsules\": [],\n  \"ikChains\": []\n}\n", 4,
                   "/limbs/0");
    expectRejected(head + "  \"limbs\": [],\n  \"capsules\": [[]],\n  \"ikChains\": []\n}\n", 5,
                   "/capsules/0");
    expectRejected(head + "  \"limbs\": [],\n  \"capsules\": [],\n  \"ikChains\": [null]\n}\n", 6,
                   "/ikChains/0");
}

TEST(BodyMapLoadErrors, WrongTypeInsideEntries)
{
    expectAllRejected({
        // Limbs (line 5).
        {"\"name\": \"RightArm\", \"bones\"", "\"name\": 5, \"bones\"", 5, "/limbs/0/name"},
        {"\"bones\": [\"RightUpperArm\", \"RightForearm\", \"RightHand\"]",
         "\"bones\": \"RightUpperArm\"", 5, "/limbs/0/bones"},
        {"[\"RightUpperArm\", \"RightForearm\", \"RightHand\"]",
         "[\"RightUpperArm\", 5, \"RightHand\"]", 5, "/limbs/0/bones/1"},
        {"[\"RightUpperArm\", \"RightForearm\", \"RightHand\"]",
         "[\"RightUpperArm\", \"RightForearm\", null]", 5, "/limbs/0/bones/2"},
        // Capsules (line 9).
        {"\"region\": \"RightForearm\"", "\"region\": true", 9, "/capsules/0/region"},
        {"\"bone\": \"RightForearm\"", "\"bone\": 1", 9, "/capsules/0/bone"},
        {"\"endBone\": \"RightHand\", \"radiusRatio\"", "\"endBone\": [], \"radiusRatio\"", 9,
         "/capsules/0/endBone"},
        {"\"radiusRatio\": 0.15", "\"radiusRatio\": \"0.15\"", 9, "/capsules/0/radiusRatio"},
        // Chains (chain 0 starts at line 13).
        {"\"name\": \"RightArm\",\n      \"rootBone\"", "\"name\": 5,\n      \"rootBone\"", 14,
         "/ikChains/0/name"},
        {"\"rootBone\": \"RightUpperArm\"", "\"rootBone\": 5", 15, "/ikChains/0/rootBone"},
        {"\"midBone\": \"RightForearm\"", "\"midBone\": null", 16, "/ikChains/0/midBone"},
        {"\"endBone\": \"RightHand\",\n      \"minBendRad\"",
         "\"endBone\": false,\n      \"minBendRad\"", 17, "/ikChains/0/endBone"},
        {"\"minBendRad\": 0,", "\"minBendRad\": \"0\",", 18, "/ikChains/0/minBendRad"},
        {"\"maxBendRad\": 2.6", "\"maxBendRad\": [2.6]", 19, "/ikChains/0/maxBendRad"},
        {"\"poleDirection\": [0, -1, 0]", "\"poleDirection\": 5", 20, "/ikChains/0/poleDirection"},
        // Test choice: a bad element is reported at the element, a wrong length at the array.
        {"[0, -1, 0]", "[0, \"a\", 0]", 20, "/ikChains/0/poleDirection/1"},
        {"[0, -1, 0]", "[0, -1]", 20, "/ikChains/0/poleDirection"},
        {"[0, -1, 0]", "[0, -1, 0, 1]", 20, "/ikChains/0/poleDirection"},
        {"\"limbRadiusM\": 0.04", "\"limbRadiusM\": \"0.04\"", 21, "/ikChains/0/limbRadiusM"},
    });
}

TEST(BodyMapLoadErrors, UnknownKeyReportsTheLineWhereItsValueStarts)
{
    expectAllRejected({
        {"  \"limbs\": [\n", "  \"extra\": true,\n  \"limbs\": [\n", 4, "/extra"},
        {"{\"name\": \"RightArm\", \"bones\"", "{\"bogus\": 1, \"name\": \"RightArm\", \"bones\"",
         5, "/limbs/0/bogus"},
        {"{\"region\": \"UpperBack\"", "{\"bogus\": {}, \"region\": \"UpperBack\"", 10,
         "/capsules/1/bogus"},
        {"      \"name\": \"RightArm\",\n", "      \"bogus\": 1,\n      \"name\": \"RightArm\",\n",
         14, "/ikChains/0/bogus"},
        {"\"limbRadiusM\": 0.03\n", "\"limbRadiusM\": 0.03,\n      \"bogus\": [1]\n", 32,
         "/ikChains/1/bogus"},
    });
}

TEST(BodyMapLoadErrors, MisspelledKeyIsReportedAsUnknownRatherThanAsTheRealKeyMissing)
{
    // Test choice: as for marker files, unknown keys are checked before missing ones.
    expectAllRejected({
        {"\"radiusRatio\": 0.15", "\"radiusRatioo\": 0.15", 9, "/capsules/0/radiusRatioo"},
    });
}

// ---- Rejections: values -------------------------------------------------------------------------

TEST(BodyMapLoadErrors, EmptyStringsAreRejected)
{
    expectAllRejected({
        {"\"name\": \"RightArm\", \"bones\"", "\"name\": \"\", \"bones\"", 5, "/limbs/0/name"},
        {"[\"RightUpperArm\", \"RightForearm\", \"RightHand\"]",
         "[\"\", \"RightForearm\", \"RightHand\"]", 5, "/limbs/0/bones/0"},
        {"[\"RightUpperArm\", \"RightForearm\", \"RightHand\"]",
         "[\"RightUpperArm\", \"RightForearm\", \"\"]", 5, "/limbs/0/bones/2"},
        {"\"region\": \"RightForearm\"", "\"region\": \"\"", 9, "/capsules/0/region"},
        {"\"bone\": \"RightForearm\"", "\"bone\": \"\"", 9, "/capsules/0/bone"},
        {"\"endBone\": \"RightHand\", \"radiusRatio\"", "\"endBone\": \"\", \"radiusRatio\"", 9,
         "/capsules/0/endBone"},
        {"\"name\": \"RightArm\",\n      \"rootBone\"", "\"name\": \"\",\n      \"rootBone\"", 14,
         "/ikChains/0/name"},
        {"\"rootBone\": \"RightUpperArm\"", "\"rootBone\": \"\"", 15, "/ikChains/0/rootBone"},
        {"\"midBone\": \"RightForearm\"", "\"midBone\": \"\"", 16, "/ikChains/0/midBone"},
        {"\"endBone\": \"RightHand\",\n      \"minBendRad\"",
         "\"endBone\": \"\",\n      \"minBendRad\"", 17, "/ikChains/0/endBone"},
    });
}

TEST(BodyMapLoadErrors, LimbWithFewerThanTwoBones)
{
    expectAllRejected({
        {"[\"RightUpperArm\", \"RightForearm\", \"RightHand\"]", "[]", 5, "/limbs/0/bones"},
        {"[\"RightUpperArm\", \"RightForearm\", \"RightHand\"]", "[\"RightUpperArm\"]", 5,
         "/limbs/0/bones"},
        {"[\"LeftUpperArm\", \"LeftForearm\", \"LeftHand\"]", "[\"LeftHand\"]", 6,
         "/limbs/1/bones"},
    });
    // Two bones are enough.
    EXPECT_TRUE(
        loadBodyMap(replaceOnce(kBodyMapText, "[\"LeftUpperArm\", \"LeftForearm\", \"LeftHand\"]",
                                "[\"LeftForearm\", \"LeftHand\"]"),
                    kFile)
            .ok());
}

TEST(BodyMapLoadErrors, DuplicatesAreReportedAtTheSecondOccurrence)
{
    expectAllRejected({
        // Limb names.
        {"{\"name\": \"LeftArm\", \"bones\"", "{\"name\": \"RightArm\", \"bones\"", 6,
         "/limbs/1/name"},
        // Capsule regions.
        {"{\"region\": \"UpperBack\"", "{\"region\": \"RightForearm\"", 10, "/capsules/1/region"},
        // Chain names.
        {"      \"name\": \"LeftArm\",\n", "      \"name\": \"RightArm\",\n", 24,
         "/ikChains/1/name"},
        // Chain end bones.
        {"      \"endBone\": \"LeftHand\",\n", "      \"endBone\": \"RightHand\",\n", 27,
         "/ikChains/1/endBone"},
    });
}

TEST(BodyMapLoadErrors, DuplicateReportedAtTheSecondOccurrenceEvenWithThreeEntries)
{
    // The duplicate is the third limb; the pointer says /limbs/2/name, on its own line.
    const std::string text = R"json({
  "format": "opas.bodymap",
  "version": 1,
  "limbs": [
    {"name": "A", "bones": ["a", "b"]},
    {"name": "B", "bones": ["a", "b"]},
    {"name": "A", "bones": ["a", "b"]}
  ],
  "capsules": [],
  "ikChains": []
}
)json";
    expectRejected(text, 7, "/limbs/2/name");
}

TEST(BodyMapLoadErrors, RadiusRatioMustBePositive)
{
    expectAllRejected({
        {"\"radiusRatio\": 0.15", "\"radiusRatio\": 0", 9, "/capsules/0/radiusRatio"},
        {"\"radiusRatio\": 0.15", "\"radiusRatio\": -0.1", 9, "/capsules/0/radiusRatio"},
        {"\"radiusRatio\": 0.1667", "\"radiusRatio\": -1e-9", 10, "/capsules/1/radiusRatio"},
    });
    EXPECT_TRUE(
        loadBodyMap(replaceOnce(kBodyMapText, "\"radiusRatio\": 0.15", "\"radiusRatio\": 1e-9"),
                    kFile)
            .ok());
}

TEST(BodyMapLoadErrors, BendLimitsMustBeOrderedAndWithinZeroToPi)
{
    expectAllRejected({
        // Below 0.
        {"\"minBendRad\": 0,", "\"minBendRad\": -0.1,", 18, "/ikChains/0/minBendRad"},
        // Above pi.
        {"\"maxBendRad\": 2.6", "\"maxBendRad\": 3.2", 19, "/ikChains/0/maxBendRad"},
        {"\"maxBendRad\": 2.5", "\"maxBendRad\": 3.1416", 29, "/ikChains/1/maxBendRad"},
        // min > max is reported at minBendRad (chain 0: max 2.6; chain 1: max 2.5).
        {"\"minBendRad\": 0,", "\"minBendRad\": 2.7,", 18, "/ikChains/0/minBendRad"},
        {"\"minBendRad\": 0.1,", "\"minBendRad\": 2.6,", 28, "/ikChains/1/minBendRad"},
    });
}

TEST(BodyMapLoadErrors, MinAboveMaxWhenBothAreInRange)
{
    // Both inside [0, pi] but out of order: reported at minBendRad (one line above maxBendRad).
    std::string text = replaceOnce(kBodyMapText, "\"minBendRad\": 0,", "\"minBendRad\": 3,");
    text = replaceOnce(text, "\"maxBendRad\": 2.6", "\"maxBendRad\": 1");
    expectRejected(text, 18, "/ikChains/0/minBendRad");
}

TEST(BodyMapLoadErrors, PoleDirectionMustBeNonZero)
{
    expectAllRejected({
        {"[0, -1, 0]", "[0, 0, 0]", 20, "/ikChains/0/poleDirection"},
        {"[0, -1, 0]", "[0.0, -0.0, 0.0]", 20, "/ikChains/0/poleDirection"},
        {"[-0.35, -0.2, -0.9]", "[0, 0, 0]", 30, "/ikChains/1/poleDirection"},
    });
}

TEST(BodyMapLoadErrors, LimbRadiusMustBePositive)
{
    expectAllRejected({
        {"\"limbRadiusM\": 0.04", "\"limbRadiusM\": 0", 21, "/ikChains/0/limbRadiusM"},
        {"\"limbRadiusM\": 0.04", "\"limbRadiusM\": -0.01", 21, "/ikChains/0/limbRadiusM"},
        {"\"limbRadiusM\": 0.03\n", "\"limbRadiusM\": 0\n", 31, "/ikChains/1/limbRadiusM"},
    });
}

TEST(BodyMapLoadErrors, NonFiniteNumbersAreRejected)
{
    // JSON has no NaN or Infinity literal; an out-of-range exponent parses to infinity.
    expectAllRejected({
        {"\"radiusRatio\": 0.15", "\"radiusRatio\": 1e999", 9, "/capsules/0/radiusRatio"},
        {"\"minBendRad\": 0,", "\"minBendRad\": 1e999,", 18, "/ikChains/0/minBendRad"},
        {"\"maxBendRad\": 2.6", "\"maxBendRad\": 1e999", 19, "/ikChains/0/maxBendRad"},
        {"\"maxBendRad\": 2.6", "\"maxBendRad\": -1e999", 19, "/ikChains/0/maxBendRad"},
        {"[0, -1, 0]", "[0, 1e999, 0]", 20, "/ikChains/0/poleDirection/1"},
        {"[0, -1, 0]", "[-1e999, 0, 0]", 20, "/ikChains/0/poleDirection/0"},
        {"\"limbRadiusM\": 0.04", "\"limbRadiusM\": 1e999", 21, "/ikChains/0/limbRadiusM"},
    });
}

TEST(BodyMapLoadErrors, ErrorsInALaterEntryUseThatEntrysIndexAndLine)
{
    expectAllRejected({
        {"\"limbRadiusM\": 0.03\n", "\"limbRadiusM\": 0\n", 31, "/ikChains/1/limbRadiusM"},
        {"\"radiusRatio\": 0.1667", "\"radiusRatio\": 0", 10, "/capsules/1/radiusRatio"},
        {"      \"rootBone\": \"LeftUpperArm\",\n", "      \"rootBone\": \"\",\n", 25,
         "/ikChains/1/rootBone"},
    });
}

TEST(BodyMapLoadErrors, ToStringIsFileLineFieldMessage)
{
    const BodyMapLoadResult result = loadBodyMap(
        replaceOnce(kBodyMapText, "\"limbRadiusM\": 0.04", "\"limbRadiusM\": 0"), kFile);
    ASSERT_FALSE(result.ok());
    const std::string expected =
        std::string(kFile) + ":21: /ikChains/0/limbRadiusM: " + result.error.message;
    EXPECT_EQ(result.error.toString(), expected);
}

// ---- Files on disk ------------------------------------------------------------------------------

std::filesystem::path dataFile(const char* name)
{
    return std::filesystem::path(OPAS_TEST_DATA_DIR) / "bodymap" / name;
}

TEST(BodyMapLoadFile, ReadsAValidFileFromDisk)
{
    const BodyMapLoadResult result = loadBodyMapFile(dataFile("humanoid.bodymap.json"));
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(sameMapping(valueOf(result), bodyMapOfText()));
}

TEST(BodyMapLoadFile, ErrorsNameThePathAsPassedAndTheLine)
{
    const std::filesystem::path path = dataFile("bad_radius.bodymap.json");
    const BodyMapLoadResult result = loadBodyMapFile(path);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_EQ(result.error.line, 21U);
    EXPECT_EQ(result.error.field, "/ikChains/0/limbRadiusM");
}

TEST(BodyMapLoadFile, MissingFileIsAnErrorNotAnException)
{
    const std::filesystem::path path = dataFile("does_not_exist.bodymap.json");
    BodyMapLoadResult result;
    EXPECT_NO_THROW(result = loadBodyMapFile(path));
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_EQ(result.error.field, "");
    EXPECT_FALSE(result.error.message.empty());
}

TEST(BodyMapLoadFile, MissingFileWithNonAsciiPathReportsTheUtf8PathAndDoesNotThrow)
{
    const std::filesystem::path path = std::filesystem::path(OPAS_TEST_DATA_DIR) / "bodymap" /
                                       std::filesystem::path(u8"does_not_exist_日本.bodymap.json");
    // The expected text is the path as UTF-8 bytes, whatever the platform's native encoding.
    std::string utf8;
    for (const char8_t unit : path.u8string()) {
        utf8.push_back(static_cast<char>(unit));
    }

    BodyMapLoadResult result;
    EXPECT_NO_THROW(result = loadBodyMapFile(path));
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, utf8);
    EXPECT_EQ(result.error.field, "");
    EXPECT_FALSE(result.error.message.empty());
}

TEST(BodyMapLoadFile, ADirectoryInPlaceOfAFileIsAnErrorNotAnException)
{
    const std::filesystem::path path = std::filesystem::path(OPAS_TEST_DATA_DIR) / "bodymap";
    BodyMapLoadResult result;
    EXPECT_NO_THROW(result = loadBodyMapFile(path));
    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.error.message.empty());
}

} // namespace
