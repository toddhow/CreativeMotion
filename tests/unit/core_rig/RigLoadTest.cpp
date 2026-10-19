#include "RigTestData.hpp"
#include "opas/rig/Rig.hpp"
#include "opas/rig/RigFile.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace {

using opas::rig::BoxShape;
using opas::rig::CapsuleShape;
using opas::rig::findBody;
using opas::rig::findJoint;
using opas::rig::loadRig;
using opas::rig::loadRigFile;
using opas::rig::parentBody;
using opas::rig::Rig;
using opas::rig::RigLoadResult;
using opas::rig::SphereShape;
using opas::rig::validateRig;
using opas::rig::writeRig;
using opas::test::kRigText;

constexpr const char* kFile = "synthetic.rig.json";

// Replaces the single occurrence of `from`; a test naming text that is not
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

// Only call after checking ok(); value_or keeps static analysis happy.
Rig valueOf(const RigLoadResult& result)
{
    return result.value.value_or(Rig{});
}

Rig loadedTestRig()
{
    const RigLoadResult result = loadRig(kRigText, kFile);
    EXPECT_TRUE(result.ok()) << result.error.toString();
    return valueOf(result);
}

void expectRejected(const std::string& text, std::size_t line, const std::string& field)
{
    const RigLoadResult result = loadRig(text, kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, kFile);
    EXPECT_EQ(result.error.line, line) << result.error.toString();
    EXPECT_EQ(result.error.field, field) << result.error.toString();
    EXPECT_FALSE(result.error.message.empty());
}

TEST(RigLoad, LoadsTheSyntheticRig)
{
    const Rig rig = loadedTestRig();
    EXPECT_EQ(rig.name, "test-rig");
    ASSERT_EQ(rig.skeleton.bones.size(), 4U);
    EXPECT_EQ(rig.skeleton.bones[0].parent, -1);
    EXPECT_EQ(rig.skeleton.bones[3].parent, 2);
    EXPECT_DOUBLE_EQ(rig.skeleton.bones[1].localRest.translation.z, -0.3);

    ASSERT_EQ(rig.bodies.size(), 3U);
    EXPECT_TRUE(std::holds_alternative<BoxShape>(rig.bodies[0].shape));
    ASSERT_TRUE(std::holds_alternative<CapsuleShape>(rig.bodies[1].shape));
    EXPECT_DOUBLE_EQ(std::get<CapsuleShape>(rig.bodies[1].shape).radiusM, 0.04);
    EXPECT_TRUE(std::holds_alternative<SphereShape>(rig.bodies[2].shape));
    EXPECT_DOUBLE_EQ(rig.bodies[1].massKg, 1.5);

    ASSERT_EQ(rig.joints.size(), 2U);
    EXPECT_EQ(rig.joints[1].bone, "Ball");
    EXPECT_DOUBLE_EQ(rig.joints[1].twistMinRad, -0.3);
    EXPECT_EQ(validateRig(rig), "");
}

TEST(RigLoad, JointsSkipBonesWithoutBodies)
{
    const Rig rig = loadedTestRig();
    // Tip has no body, so Ball's joint connects it to Mid's body.
    EXPECT_EQ(parentBody(rig, 2), std::optional<std::size_t>(1));
    EXPECT_EQ(parentBody(rig, 1), std::optional<std::size_t>(0));
    EXPECT_FALSE(parentBody(rig, 0).has_value());
    EXPECT_EQ(findBody(rig, "Ball"), std::optional<std::size_t>(2));
    EXPECT_FALSE(findBody(rig, "Tip").has_value());
    EXPECT_EQ(findJoint(rig, "Ball"), std::optional<std::size_t>(1));
    EXPECT_FALSE(findJoint(rig, "Root").has_value());
}

// A table row: replace `from` by `to` in kRigText; expect this line and pointer.
struct Edit {
    const char* name;
    const char* from;
    const char* to;
    std::size_t line;
    const char* field;
};

class RigRejects : public ::testing::TestWithParam<Edit> {};

TEST_P(RigRejects, WithLineAndField)
{
    const Edit& e = GetParam();
    expectRejected(replaceOnce(kRigText, e.from, e.to), e.line, e.field);
}

const Edit kEdits[] = {
    {"SyntaxError", R"("version": 1,)", R"("version": 1)", 4, ""},
    {"UnknownKey", R"("name": "test-rig",)", R"("name": "test-rig", "extra": 1,)", 4, "/extra"},
    {"MissingField", "\"rotation\": [0, 0, 0, 1],\n      \"massKg\": 5",
     "\"rotation\": [0, 0, 0, 1]", 12, "/bodies/0/massKg"},
    {"WrongFormat", R"("opas.rig")", R"("opas.rog")", 2, "/format"},
    {"WrongVersion", R"("version": 1)", R"("version": 2)", 3, "/version"},
    {"EmptyName", R"("name": "test-rig")", R"("name": "")", 4, "/name"},
    {"DuplicateBone", R"({"name": "Tip", "parent": "Mid")", R"({"name": "Mid", "parent": "Mid")", 8,
     "/bones/2/name"},
    {"UnknownParent", R"("parent": "Mid", "translationM": [0, 0, -0.25])",
     R"("parent": "Nope", "translationM": [0, 0, -0.25])", 8, "/bones/2/parent"},
    {"ParentListedAfterChild", R"("parent": "Root", "translationM": [0, 0, -0.3])",
     R"("parent": "Tip", "translationM": [0, 0, -0.3])", 7, "/bones/1/parent"},
    {"OwnParent", R"("parent": "Root", "translationM": [0, 0, -0.3])",
     R"("parent": "Mid", "translationM": [0, 0, -0.3])", 7, "/bones/1/parent"},
    {"NonUnitBoneRotation", R"([0, 0, -0.3], "rotation": [0, 0, 0, 1])",
     R"([0, 0, -0.3], "rotation": [0, 0, 0, 2])", 7, "/bones/1/rotation"},
    {"NonFiniteNumber", R"("massKg": 1.5)", R"("massKg": 1e999)", 24, "/bodies/1/massKg"},
    {"BodyOnUnknownBone", "\"bone\": \"Root\",", "\"bone\": \"Nope\",", 13, "/bodies/0/bone"},
    {"TwoBodiesOnOneBone", "\"bone\": \"Ball\",\n      \"shape\"",
     "\"bone\": \"Mid\",\n      \"shape\"", 27, "/bodies/2/bone"},
    {"BadShapeType", R"("type": "sphere")", R"("type": "cone")", 28, "/bodies/2/shape/type"},
    {"KeyOfAnotherShape", R"("radiusM": 0.05})", R"("radiusM": 0.05, "halfHeightM": 1})", 28,
     "/bodies/2/shape/halfHeightM"},
    {"ZeroCapsuleRadius", R"("radiusM": 0.04)", R"("radiusM": 0)", 21, "/bodies/1/shape/radiusM"},
    {"NegativeBoxExtent", "[0.1, 0.1, 0.1]", "[0.1, -0.1, 0.1]", 14,
     "/bodies/0/shape/halfExtentsM"},
    {"ZeroSphereRadius", R"("radiusM": 0.05)", R"("radiusM": 0)", 28, "/bodies/2/shape/radiusM"},
    {"NonUnitBodyRotation", "[0, 0, -0.14],\n      \"rotation\": [0, 0, 0, 1]",
     "[0, 0, -0.14],\n      \"rotation\": [0.1, 0, 0, 1]", 23, "/bodies/1/rotation"},
    {"ZeroMass", R"("massKg": 0.5)", R"("massKg": 0)", 31, "/bodies/2/massKg"},
    {"JointOnBoneWithoutBody", "\"bone\": \"Ball\",\n      \"twistAxis\"",
     "\"bone\": \"Tip\",\n      \"twistAxis\"", 45, "/joints/1/bone"},
    {"JointOnRootBody", "\"bone\": \"Mid\",\n      \"twistAxis\"",
     "\"bone\": \"Root\",\n      \"twistAxis\"", 36, "/joints/0/bone"},
    {"TwoJointsOnOneBone", "\"bone\": \"Ball\",\n      \"twistAxis\"",
     "\"bone\": \"Mid\",\n      \"twistAxis\"", 45, "/joints/1/bone"},
    {"ZeroAxis", R"("twistAxis": [1, 0, 0])", R"("twistAxis": [0, 0, 0])", 37,
     "/joints/0/twistAxis"},
    {"NonUnitAxis", R"("twistAxis": [0, 0, -1])", R"("twistAxis": [0, 0, -1.1])", 46,
     "/joints/1/twistAxis"},
    {"NonUnitPlaneAxis", R"("planeAxis": [0, 1, 0])", R"("planeAxis": [0, 2, 0])", 38,
     "/joints/0/planeAxis"},
    {"AxesNotPerpendicular", R"("planeAxis": [1, 0, 0])", R"("planeAxis": [0.6, 0, -0.8])", 47,
     "/joints/1/planeAxis"},
    {"ConeAbovePi", R"("normalHalfConeRad": 0.5)", R"("normalHalfConeRad": 4)", 48,
     "/joints/1/normalHalfConeRad"},
    {"NegativeCone", R"("planeHalfConeRad": 0.25)", R"("planeHalfConeRad": -0.1)", 49,
     "/joints/1/planeHalfConeRad"},
    {"TwistAbovePi", R"("twistMaxRad": 2.6)", R"("twistMaxRad": 3.5)", 42, "/joints/0/twistMaxRad"},
    {"TwistBelowMinusPi", R"("twistMinRad": -0.3)", R"("twistMinRad": -3.5)", 50,
     "/joints/1/twistMinRad"},
    {"TwistMinAboveMax", R"("twistMinRad": -0.3)", R"("twistMinRad": 0.4)", 50,
     "/joints/1/twistMinRad"},
};

INSTANTIATE_TEST_SUITE_P(Edits, RigRejects, ::testing::ValuesIn(kEdits),
                         [](const ::testing::TestParamInfo<Edit>& info) {
                             return std::string(info.param.name);
                         });

// The body-tree rules are easiest to break on a loaded rig and write it back;
// the written text keeps kRigText's layout, so the lines are known.
TEST(RigRejects, BodyWithoutJoint)
{
    Rig rig = loadedTestRig();
    rig.joints.erase(rig.joints.begin()); // Mid's joint
    EXPECT_EQ(validateRig(rig).rfind("/bodies/1: ", 0), 0U) << validateRig(rig);
    expectRejected(writeRig(rig), 19, "/bodies/1");
}

TEST(RigRejects, SecondRootBody)
{
    Rig rig = loadedTestRig();
    rig.joints.pop_back();             // Ball's joint
    rig.skeleton.bones[3].parent = -1; // Ball becomes a root bone with a body
    EXPECT_EQ(validateRig(rig).rfind("/bodies/2/bone: ", 0), 0U) << validateRig(rig);
    expectRejected(writeRig(rig), 27, "/bodies/2/bone");
}

TEST(RigRejects, EmptyBodiesAndBones)
{
    Rig rig = loadedTestRig();
    rig.bodies.clear();
    rig.joints.clear();
    EXPECT_EQ(validateRig(rig).rfind("/bodies: ", 0), 0U) << validateRig(rig);
    rig.skeleton.bones.clear();
    EXPECT_EQ(validateRig(rig).rfind("/bones: ", 0), 0U) << validateRig(rig);
}

TEST(RigValidate, ReportsProgrammaticRigsWithPointers)
{
    Rig rig = loadedTestRig();
    rig.bodies[1].massKg = -1.0;
    EXPECT_EQ(validateRig(rig), "/bodies/1/massKg: must be > 0");
    rig = loadedTestRig();
    rig.skeleton.bones[2].parent = 3;
    EXPECT_EQ(validateRig(rig).rfind("/bones/2/parent: ", 0), 0U) << validateRig(rig);
}

TEST(RigLoad, MissingFileIsAnErrorNotAThrow)
{
    const RigLoadResult result = loadRigFile(OPAS_TEST_DATA_DIR "/does_not_exist.rig.json");
    EXPECT_FALSE(result.ok());
    EXPECT_NE(result.error.file.find("does_not_exist.rig.json"), std::string::npos);
}

TEST(RigLoad, DocumentMustBeAnObject)
{
    expectRejected("[1, 2]", 1, "");
}

} // namespace
