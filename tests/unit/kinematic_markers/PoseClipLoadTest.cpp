#include "ClipTestData.hpp"
#include "TestSupport.hpp"
#include "opas/markers/MarkerFile.hpp"
#include "opas/markers/PoseClip.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace {

using opas::markers::loadPoseClip;
using opas::markers::loadPoseClipFile;
using opas::markers::PoseClip;
using opas::markers::PoseClipLoadResult;
using opas::test::kPoseClipText;
using opas::test::poseClipOfText;

constexpr const char* kFile = "synthetic.poseclip.json";

// Replaces the single occurrence of `from`; a test naming text that is not there exactly once is
// itself broken, so that fails loudly.
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

PoseClip valueOf(const PoseClipLoadResult& result)
{
    return result.value.value_or(PoseClip{});
}

void expectRejected(const std::string& text, std::size_t line, const std::string& field)
{
    const PoseClipLoadResult result = loadPoseClip(text, kFile);
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
        expectRejected(replaceOnce(kPoseClipText, e.from, e.to), e.line, e.field);
    }
}

// A clip with one bone per actor and one frame each, with the actors object spelled out, for
// the structural cases that are awkward to edit out of kPoseClipText. Line 5 is `"actors": [`.
std::string withActors(const std::string& actorsBody)
{
    return "{\n  \"format\": \"opas.poseclip\",\n  \"version\": 1,\n  \"sampleRateHz\": 30,\n  "
           "\"actors\": [\n" +
           actorsBody + "\n  ]\n}\n";
}

std::string oneBoneActor()
{
    return "    {\n      \"bodyMap\": \"h\",\n      \"bones\": [\"A\"],\n      \"frames\": [\n     "
           "   "
           "[[0, 0, 0, 0, 0, 0, 1]]\n      ]\n    }";
}

// Line numbers refer to kPoseClipText (see ClipTestData.hpp): actor 0 at 6 (bodyMap 7, bones 8,
// frames 9, frame 0 at 10, frame 1 at 11), actor 1 at 14 (bodyMap 15, bones 16, frames 17,
// frame 0 at 18, frame 1 at 19).

// ---- Valid files --------------------------------------------------------------------------

TEST(PoseClipLoad, CanonicalFileLoadsToTheExpectedClip)
{
    const PoseClipLoadResult result = loadPoseClip(kPoseClipText, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    const PoseClip clip = valueOf(result);
    EXPECT_TRUE(clip == poseClipOfText());
    EXPECT_EQ(clip.sampleRateHz, 30.0);
    EXPECT_EQ(clip.frameCount(), 2U);
    ASSERT_EQ(clip.actors[0].bones.size(), 2U);
    EXPECT_EQ(clip.actors[0].bones[1], "RightHand");
    EXPECT_EQ(clip.actors[0].frames[1][1].translation.x, 0.31);
    EXPECT_EQ(clip.actors[1].frames[0][0].rotation.z, 1.0);
}

TEST(PoseClipLoad, AcceptsValidJsonThatIsNotCanonical)
{
    // Test choice: any valid JSON layout, key order and number spelling is accepted.
    const std::string loose =
        "{\"actors\":[{\"frames\":[[[0.0,0.0,1.0,0.0,0.0,0.0,1.0],[3e-1,-0.2,1.1,0,0,0,1]],"
        "[[0,0,1,0,0,0,1],[3.1E-1,-0.2,1.1,0,0,0,1]]],\"bones\":[\"Pelvis\",\"RightHand\"],"
        "\"bodyMap\":\"humanoid\"},{\"frames\":[[[0.6,0,1,0,0,1,0]],[[0.6,0,1,0,0,1,0]]],"
        "\"bones\":[\"Pelvis\"],\"bodyMap\":\"humanoid\"}],\"sampleRateHz\":3e1,\"version\":1,"
        "\"format\":\"opas.poseclip\"}";
    const PoseClipLoadResult result = loadPoseClip(loose, kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == poseClipOfText());
}

TEST(PoseClipLoad, SampleRateNeedNotBeAnInteger)
{
    const PoseClipLoadResult result = loadPoseClip(
        replaceOnce(kPoseClipText, "\"sampleRateHz\": 30,", "\"sampleRateHz\": 29.97,"), kFile);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_EQ(valueOf(result).sampleRateHz, 29.97);
}

TEST(PoseClipLoad, QuaternionWithinOneMillionthOfUnitIsAccepted)
{
    // Length error about 5e-7 < 1e-6.
    const std::string text = replaceOnce(kPoseClipText, "[0.3, -0.2, 1.1, 0, 0, 0, 1]",
                                         "[0.3, -0.2, 1.1, 0, 0, 0, 1.0000005]");
    EXPECT_TRUE(loadPoseClip(text, kFile).ok());
}

TEST(PoseClipLoad, ActorsMayHaveDifferentBoneSetsAndTheSameBoneNamesAsEachOther)
{
    // The canonical clip already does (2 bones against 1); both use "Pelvis".
    const PoseClip clip = valueOf(loadPoseClip(kPoseClipText, kFile));
    EXPECT_NE(clip.actors[0].bones.size(), clip.actors[1].bones.size());
    EXPECT_EQ(clip.actors[0].bones[0], clip.actors[1].bones[0]);
}

// ---- Rejections ------------------------------------------------------------------------------

TEST(PoseClipLoadErrors, MalformedJsonReportsTheLineOfTheSyntaxError)
{
    expectAllRejected({
        {"\"version\": 1,", "\"version\": @,", 3, ""},
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, 1.1, 0, 0, 0, 1,]]", 10, ""},
        {"\"sampleRateHz\": 30,", "\"sampleRateHz\": 30x,", 4, ""},
        {"[[0.6, 0, 1, 0, 0, 1, 0]],\n", "[[0.6, 0, 1, 0, 0, 1, 0]\n", 19,
         ""}, // unbalanced: the error is at the next "[", line 19
    });
}

TEST(PoseClipLoadErrors, EmptyOrNonJsonInputAndNonObjectTopLevel)
{
    const std::vector<std::string> inputs = {std::string(), std::string("not json at all\n")};
    for (const std::string& text : inputs) {
        const PoseClipLoadResult result = loadPoseClip(text, kFile);
        EXPECT_FALSE(result.ok());
        EXPECT_GE(result.error.line, 1U);
        EXPECT_EQ(result.error.field, "");
        EXPECT_FALSE(result.error.message.empty());
    }
    expectRejected("[]\n", 1, "");
}

TEST(PoseClipLoadErrors, WrongFormatVersionAndSampleRate)
{
    expectAllRejected({
        {"\"opas.poseclip\"", "\"opas.markers\"", 2, "/format"},
        {"\"format\": \"opas.poseclip\"", "\"format\": 1", 2, "/format"},
        {"\"version\": 1,", "\"version\": 2,", 3, "/version"},
        {"\"version\": 1,", "\"version\": 0,", 3, "/version"},
        {"\"version\": 1,", "\"version\": \"1\",", 3, "/version"},
        {"\"sampleRateHz\": 30,", "\"sampleRateHz\": 0,", 4, "/sampleRateHz"},
        {"\"sampleRateHz\": 30,", "\"sampleRateHz\": -30,", 4, "/sampleRateHz"},
        {"\"sampleRateHz\": 30,", "\"sampleRateHz\": \"30\",", 4, "/sampleRateHz"},
        {"\"sampleRateHz\": 30,", "\"sampleRateHz\": 1e999,", 4, "/sampleRateHz"},
    });
}

TEST(PoseClipLoadErrors, MissingTopLevelField)
{
    expectAllRejected({
        {"  \"format\": \"opas.poseclip\",\n", "", 1, "/format"},
        {"  \"version\": 1,\n", "", 1, "/version"},
        {"  \"sampleRateHz\": 30,\n", "", 1, "/sampleRateHz"},
    });
    expectRejected(
        "{\n  \"format\": \"opas.poseclip\",\n  \"version\": 1,\n  \"sampleRateHz\": 30\n}\n", 1,
        "/actors");
}

TEST(PoseClipLoadErrors, ActorsMustBeAnArrayOfExactlyTwoObjects)
{
    expectRejected(withActors(oneBoneActor()), 5, "/actors");
    expectRejected(withActors(oneBoneActor() + ",\n" + oneBoneActor() + ",\n" + oneBoneActor()), 5,
                   "/actors");
    expectRejected(withActors(""), 5, "/actors");
    expectRejected(
        "{\n  \"format\": \"opas.poseclip\",\n  \"version\": 1,\n  \"sampleRateHz\": 30,\n  "
        "\"actors\": {}\n}\n",
        5, "/actors");
    expectRejected(withActors(oneBoneActor() + ",\n    5"), 13, "/actors/1");
}

TEST(PoseClipLoadErrors, MissingFieldInAnActorReportsTheLineWhereThatActorStarts)
{
    expectAllRejected({
        {"      \"bodyMap\": \"humanoid\",\n      \"bones\": [\"Pelvis\", \"RightHand\"],\n",
         "      \"bones\": [\"Pelvis\", \"RightHand\"],\n", 6, "/actors/0/bodyMap"},
        {"      \"bones\": [\"Pelvis\", \"RightHand\"],\n", "", 6, "/actors/0/bones"},
        {"      \"bodyMap\": \"humanoid\",\n      \"bones\": [\"Pelvis\"],\n",
         "      \"bones\": [\"Pelvis\"],\n", 14, "/actors/1/bodyMap"},
        {"      \"bones\": [\"Pelvis\"],\n", "", 14, "/actors/1/bones"},
    });
    // frames is the last key of an actor: spell the actor out.
    const std::string noFrames = withActors(
        "    {\n      \"bodyMap\": \"h\",\n      \"bones\": [\"A\"]\n    },\n" + oneBoneActor());
    expectRejected(noFrames, 6, "/actors/0/frames");
}

TEST(PoseClipLoadErrors, WrongTypes)
{
    expectAllRejected({
        {"\"bodyMap\": \"humanoid\",\n      \"bones\": [\"Pelvis\", \"RightHand\"]",
         "\"bodyMap\": 5,\n      \"bones\": [\"Pelvis\", \"RightHand\"]", 7, "/actors/0/bodyMap"},
        {"\"bones\": [\"Pelvis\", \"RightHand\"]", "\"bones\": \"Pelvis\"", 8, "/actors/0/bones"},
        {"[\"Pelvis\", \"RightHand\"]", "[\"Pelvis\", 7]", 8, "/actors/0/bones/1"},
        {"\"bones\": [\"Pelvis\"],", "\"bones\": [null],", 16, "/actors/1/bones/0"},
        // frames: not an array / a frame that is not an array / a transform that is not an
        // array.
        {"[[0, 0, 1, 0, 0, 0, 1], [0.3, -0.2, 1.1, 0, 0, 0, 1]],", "5,", 10, "/actors/0/frames/0"},
        {"[[0.6, 0, 1, 0, 0, 1, 0]],\n", "7,\n", 18, "/actors/1/frames/0"},
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "\"x\"]", 10, "/actors/0/frames/0/1"},
        // Elements of a transform.
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, \"a\", 1.1, 0, 0, 0, 1]]", 10,
         "/actors/0/frames/0/1/1"},
        {"[0.31, -0.2, 1.1, 0, 0, 0, 1]", "[0.31, -0.2, 1.1, 0, 0, 0, null]", 11,
         "/actors/0/frames/1/1/6"},
        {"[0.6, 0, 1, 0, 0, 1, 0]],\n", "[0.6, 0, 1, 0, 0, 1, true]],\n", 18,
         "/actors/1/frames/0/0/6"},
        // Test choice: a transform of the wrong length is reported at the transform.
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, 1.1, 0, 0, 0]]", 10, "/actors/0/frames/0/1"},
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, 1.1, 0, 0, 0, 1, 1]]", 10,
         "/actors/0/frames/0/1"},
    });
}

TEST(PoseClipLoadErrors, UnknownKeyReportsTheLineWhereItsValueStarts)
{
    expectAllRejected({
        {"  \"actors\": [\n", "  \"extra\": true,\n  \"actors\": [\n", 5, "/extra"},
        {"      \"bones\": [\"Pelvis\", \"RightHand\"],\n",
         "      \"bogus\": 1,\n      \"bones\": [\"Pelvis\", \"RightHand\"],\n", 8,
         "/actors/0/bogus"},
        {"      \"bones\": [\"Pelvis\"],\n",
         "      \"bogus\": [1],\n      \"bones\": [\"Pelvis\"],\n", 16, "/actors/1/bogus"},
    });
}

TEST(PoseClipLoadErrors, EmptyAndDuplicateNames)
{
    expectAllRejected({
        {"\"bodyMap\": \"humanoid\",\n      \"bones\": [\"Pelvis\", \"RightHand\"]",
         "\"bodyMap\": \"\",\n      \"bones\": [\"Pelvis\", \"RightHand\"]", 7,
         "/actors/0/bodyMap"},
        {"[\"Pelvis\", \"RightHand\"]", "[\"\", \"RightHand\"]", 8, "/actors/0/bones/0"},
        {"[\"Pelvis\", \"RightHand\"]", "[\"Pelvis\", \"\"]", 8, "/actors/0/bones/1"},
        // Duplicates are reported at the second occurrence (test choice; the same rule as
        // duplicate ids in marker files).
        {"[\"Pelvis\", \"RightHand\"]", "[\"Pelvis\", \"Pelvis\"]", 8, "/actors/0/bones/1"},
    });
}

TEST(PoseClipLoadErrors, BonesMustNotBeEmptyWhenThereAreFrames)
{
    expectAllRejected({
        // Test choice: the doc allows an empty bones list only with no frames, so a clip with
        // frames and no bones is rejected at bones.
        {"      \"bones\": [\"Pelvis\"],\n      \"frames\": [\n        [[0.6, 0, 1, 0, 0, 1, "
         "0]],\n        "
         "[[0.6, 0, 1, 0, 0, 1, 0]]\n      ]",
         "      \"bones\": [],\n      \"frames\": [\n        [],\n        []\n      ]", 16,
         "/actors/1/bones"},
    });
}

TEST(PoseClipLoadErrors, FrameLengthMustMatchTheBoneCount)
{
    expectAllRejected({
        // Too few / too many transforms in a frame: reported at that frame.
        {"[[0, 0, 1, 0, 0, 0, 1], [0.3, -0.2, 1.1, 0, 0, 0, 1]],\n", "[[0, 0, 1, 0, 0, 0, 1]],\n",
         10, "/actors/0/frames/0"},
        {"[[0, 0, 1, 0, 0, 0, 1], [0.31, -0.2, 1.1, 0, 0, 0, 1]]\n",
         "[[0, 0, 1, 0, 0, 0, 1], [0.31, -0.2, 1.1, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1]]\n", 11,
         "/actors/0/frames/1"},
        {"[[0.6, 0, 1, 0, 0, 1, 0]],\n", "[],\n", 18, "/actors/1/frames/0"},
    });
}

TEST(PoseClipLoadErrors, ActorsMustHaveTheSameNumberOfFrames)
{
    // Actor 1 with one frame against actor 0's two: reported at actor 1's frames.
    expectAllRejected({
        {"        [[0.6, 0, 1, 0, 0, 1, 0]],\n        [[0.6, 0, 1, 0, 0, 1, 0]]\n",
         "        [[0.6, 0, 1, 0, 0, 1, 0]]\n", 17, "/actors/1/frames"},
        {"        [[0.6, 0, 1, 0, 0, 1, 0]],\n        [[0.6, 0, 1, 0, 0, 1, 0]]\n",
         "        [[0.6, 0, 1, 0, 0, 1, 0]],\n        [[0.6, 0, 1, 0, 0, 1, 0]],\n        "
         "[[0.6, 0, 1, 0, 0, 1, 0]]\n",
         17, "/actors/1/frames"},
    });
}

TEST(PoseClipLoadErrors, QuaternionMustBeUnitWithinOneMillionth)
{
    expectAllRejected({
        // Reported at the transform (test choice).
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, 1.1, 0, 0, 0, 2]]", 10,
         "/actors/0/frames/0/1"},
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, 1.1, 0, 0, 0, 0]]", 10,
         "/actors/0/frames/0/1"},
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, 1.1, 0.1, 0, 0, 1]]", 10,
         "/actors/0/frames/0/1"},
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, 1.1, 0, 0, 0, 1.00001]]", 10,
         "/actors/0/frames/0/1"},
        {"[0.31, -0.2, 1.1, 0, 0, 0, 1]", "[0.31, -0.2, 1.1, 0, 0, 0, 0.999]", 11,
         "/actors/0/frames/1/1"},
        {"[0.6, 0, 1, 0, 0, 1, 0]],\n", "[0.6, 0, 1, 0, 0, 1, 1]],\n", 18, "/actors/1/frames/0/0"},
    });
}

TEST(PoseClipLoadErrors, NonFiniteNumbersAreRejected)
{
    // JSON has no NaN or Infinity literal; an out-of-range exponent parses to infinity.
    expectAllRejected({
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[1e999, -0.2, 1.1, 0, 0, 0, 1]]", 10,
         "/actors/0/frames/0/1/0"},
        {"[0.3, -0.2, 1.1, 0, 0, 0, 1]]", "[0.3, -0.2, -1e999, 0, 0, 0, 1]]", 10,
         "/actors/0/frames/0/1/2"},
        {"[0.31, -0.2, 1.1, 0, 0, 0, 1]", "[0.31, -0.2, 1.1, 0, 0, 0, 1e999]", 11,
         "/actors/0/frames/1/1/6"},
        {"[0.6, 0, 1, 0, 0, 1, 0]],\n", "[0.6, 0, 1, 0, 1e999, 1, 0]],\n", 18,
         "/actors/1/frames/0/0/4"},
    });
}

TEST(PoseClipLoadErrors, ErrorsInALaterFrameUseThatFramesIndexAndLine)
{
    expectRejected(replaceOnce(kPoseClipText, "[0.31, -0.2, 1.1, 0, 0, 0, 1]",
                               "[0.31, \"y\", 1.1, 0, 0, 0, 1]"),
                   11, "/actors/0/frames/1/1/1");
}

TEST(PoseClipLoadErrors, ToStringIsFileLineFieldMessage)
{
    const PoseClipLoadResult result = loadPoseClip(
        replaceOnce(kPoseClipText, "\"sampleRateHz\": 30,", "\"sampleRateHz\": 0,"), kFile);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error.toString(),
              std::string(kFile) + ":4: /sampleRateHz: " + result.error.message);
}

// ---- Files on disk
// ---------------------------------------------------------------------------

std::filesystem::path dataFile(const char* name)
{
    return std::filesystem::path(OPAS_TEST_DATA_DIR) / "poseclip" / name;
}

TEST(PoseClipLoadFile, ReadsAValidFileFromDisk)
{
    const PoseClipLoadResult result = loadPoseClipFile(dataFile("two_actors.poseclip.json"));
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_TRUE(valueOf(result) == poseClipOfText());
}

TEST(PoseClipLoadFile, ErrorsNameThePathAsPassedAndTheLine)
{
    const std::filesystem::path path = dataFile("bad_quaternion.poseclip.json");
    const PoseClipLoadResult result = loadPoseClipFile(path);
    ASSERT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_EQ(result.error.line, 18U);
    EXPECT_EQ(result.error.field, "/actors/1/frames/0/0");
}

TEST(PoseClipLoadFile, MissingFileIsAnErrorNotAnException)
{
    const std::filesystem::path path = dataFile("does_not_exist.poseclip.json");
    PoseClipLoadResult result;
    EXPECT_NO_THROW(result = loadPoseClipFile(path));
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, path.string());
    EXPECT_EQ(result.error.field, "");
    EXPECT_FALSE(result.error.message.empty());
}

TEST(PoseClipLoadFile, MissingFileWithNonAsciiPathReportsTheUtf8PathAndDoesNotThrow)
{
    const std::filesystem::path path = std::filesystem::path(OPAS_TEST_DATA_DIR) / "poseclip" /
                                       std::filesystem::path(u8"does_not_exist_日本.poseclip.json");
    std::string utf8;
    for (const char8_t unit : path.u8string()) {
        utf8.push_back(static_cast<char>(unit));
    }
    PoseClipLoadResult result;
    EXPECT_NO_THROW(result = loadPoseClipFile(path));
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, utf8);
    EXPECT_EQ(result.error.field, "");
    EXPECT_FALSE(result.error.message.empty());
}

TEST(PoseClipLoadFile, ADirectoryInPlaceOfAFileIsAnErrorNotAnException)
{
    PoseClipLoadResult result;
    EXPECT_NO_THROW(result =
                        loadPoseClipFile(std::filesystem::path(OPAS_TEST_DATA_DIR) / "poseclip"));
    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.error.message.empty());
}

} // namespace
