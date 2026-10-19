#include "ScenarioTestData.hpp"
#include "opas/sandbox/Runner.hpp"
#include "opas/sandbox/Scenario.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <string_view>

// Scenario files (M4, docs/tasks/0015, docs/formats/scenario.md).

namespace {

using namespace opas;
using opas::test::kScenarioText;

constexpr const char* kFile = "synthetic.scenario.json";

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

sandbox::Scenario loaded()
{
    const auto result = sandbox::loadScenario(kScenarioText, kFile);
    EXPECT_TRUE(result.ok()) << result.error.toString();
    return result.value.value_or(sandbox::Scenario{});
}

TEST(ScenarioLoad, LoadsTheSyntheticScenario)
{
    const sandbox::Scenario s = loaded();
    EXPECT_EQ(s.name, "push");
    EXPECT_EQ(s.stepsPerSecond, 60);
    EXPECT_DOUBLE_EQ(s.durationS, 3.0);
    EXPECT_DOUBLE_EQ(s.gravityMps2.z, -9.81);
    ASSERT_EQ(s.statics.size(), 1U);
    EXPECT_DOUBLE_EQ(s.statics[0].halfExtentsM.x, 20.0);
    ASSERT_EQ(s.ragdolls.size(), 1U);
    EXPECT_EQ(s.ragdolls[0].rootMotion, motor::RootMotion::Dynamic);
    EXPECT_EQ(s.ragdolls[0].motors, sandbox::MotorMode::Profile);
    EXPECT_EQ(s.ragdolls[0].pose, "rest");
    EXPECT_EQ(s.ragdolls[0].line, 12U);
    ASSERT_EQ(s.impulses.size(), 1U);
    EXPECT_EQ(s.impulses[0].bone, "Chest");
    EXPECT_EQ(s.impulses[0].line, 24U);
}

TEST(ScenarioLoad, StepCountAndImpulseTiming)
{
    sandbox::Scenario s = loaded();
    EXPECT_EQ(sandbox::stepCount(s), 180);
    EXPECT_EQ(sandbox::impulseStep(s, 0.5), 30); // an exact boundary is not pushed to 31
    EXPECT_EQ(sandbox::impulseStep(s, 0.0), 0);
    EXPECT_EQ(sandbox::impulseStep(s, 0.501), 31);
    s.durationS = 1.0 / 120.0; // half a step rounds to the nearest: 1 (lround rounds half away)
    EXPECT_EQ(sandbox::stepCount(s), 1);
}

struct Edit {
    const char* name;
    const char* from;
    const char* to;
    std::size_t line;
    const char* field;
};

class ScenarioRejects : public ::testing::TestWithParam<Edit> {};

TEST_P(ScenarioRejects, WithLineAndField)
{
    const Edit& e = GetParam();
    const auto result = sandbox::loadScenario(replaceOnce(kScenarioText, e.from, e.to), kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, kFile);
    EXPECT_EQ(result.error.line, e.line) << result.error.toString();
    EXPECT_EQ(result.error.field, e.field) << result.error.toString();
}

const Edit kEdits[] = {
    {"SyntaxError", R"("version": 1,)", R"("version": 1)", 4, ""},
    {"UnknownKey", R"("name": "push",)", R"("name": "push", "extra": 1,)", 4, "/extra"},
    {"MissingField", "\"durationS\": 3,\n", "", 1, "/durationS"},
    {"WrongFormat", R"("opas.scenario")", R"("opas.scene")", 2, "/format"},
    {"WrongVersion", R"("version": 1)", R"("version": 2)", 3, "/version"},
    {"ZeroStepRate", R"("stepsPerSecond": 60)", R"("stepsPerSecond": 0)", 5, "/stepsPerSecond"},
    {"FractionalStepRate", R"("stepsPerSecond": 60)", R"("stepsPerSecond": 60.5)", 5,
     "/stepsPerSecond"},
    {"ZeroDuration", R"("durationS": 3)", R"("durationS": 0)", 6, "/durationS"},
    {"TooLong", R"("durationS": 3)", R"("durationS": 61)", 6, "/durationS"},
    {"NonUnitStaticRotation", R"("translationM": [0, 0, -0.5], "rotation": [0, 0, 0, 1])",
     R"("translationM": [0, 0, -0.5], "rotation": [0, 0, 0, 2])", 9, "/statics/0/rotation"},
    {"ZeroBoxExtent", "[20, 20, 0.5]", "[20, 0, 0.5]", 9, "/statics/0/halfExtentsM"},
    {"NegativeFriction", R"("friction": 0.8)", R"("friction": -1)", 9, "/statics/0/friction"},
    {"DuplicateStaticName", R"(    {"name": "floor",)",
     R"(    {"name": "floor", "halfExtentsM": [1, 1, 1], "translationM": [0, 0, 0], "rotation": [0, 0, 0, 1], "friction": 0},
    {"name": "floor",)",
     10, "/statics/1/name"},
    {"UnknownRootMotion", R"("rootMotion": "dynamic")", R"("rootMotion": "floating")", 18,
     "/ragdolls/0/rootMotion"},
    {"UnknownMotors", R"("motors": "profile")", R"("motors": "stiff")", 19, "/ragdolls/0/motors"},
    {"UnknownPose", R"("pose": "rest")", R"("pose": "dance")", 20, "/ragdolls/0/pose"},
    {"NonUnitRagdollRotation", "[0, 0, 0.02],\n      \"rotation\": [0, 0, 0, 1]",
     "[0, 0, 0.02],\n      \"rotation\": [0.5, 0, 0, 1]", 17, "/ragdolls/0/rotation"},
    {"ImpulseOnUnknownRagdoll", R"("ragdoll": "hero")", R"("ragdoll": "villain")", 24,
     "/impulses/0/ragdoll"},
    {"ImpulseAfterTheEnd", R"("atS": 0.5)", R"("atS": 3)", 24, "/impulses/0/atS"},
    {"NegativeImpulseTime", R"("atS": 0.5)", R"("atS": -0.1)", 24, "/impulses/0/atS"},
    {"NonFiniteImpulse", "[0, 60, 0]", "[0, 1e999, 0]", 24, "/impulses/0/impulseNs/1"},
};

INSTANTIATE_TEST_SUITE_P(Edits, ScenarioRejects, ::testing::ValuesIn(kEdits),
                         [](const ::testing::TestParamInfo<Edit>& info) {
                             return std::string(info.param.name);
                         });

TEST(ScenarioRejects, DuplicateRagdollName)
{
    sandbox::Scenario s = loaded();
    s.ragdolls.push_back(s.ragdolls[0]);
    const auto result = sandbox::loadScenario(sandbox::writeScenario(s), kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.line, 23U) << result.error.toString(); // the second ragdoll's name
    EXPECT_EQ(result.error.field, "/ragdolls/1/name");
}

TEST(ScenarioRejects, EmptyRagdollList)
{
    sandbox::Scenario s = loaded();
    s.ragdolls.clear();
    s.impulses.clear();
    const auto result = sandbox::loadScenario(sandbox::writeScenario(s), kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.field, "/ragdolls");
}

// Errors found while running point at the scenario entry.
TEST(ScenarioRun, MissingRigFileIsReportedAtTheRagdollEntry)
{
    sandbox::Scenario s = loaded();
    s.ragdolls[0].rigPath = "does_not_exist.rig.json";
    const sandbox::RunResult result =
        sandbox::run(s, sandbox::RunOptions{OPAS_REPO_DATA_DIR, "push.scenario.json"});
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, "push.scenario.json");
    EXPECT_EQ(result.error.line, 12U);
    EXPECT_EQ(result.error.field, "/ragdolls/0/rig");
    EXPECT_NE(result.error.message.find("does_not_exist.rig.json"), std::string::npos)
        << result.error.message;
}

TEST(ScenarioRun, ImpulseOnABoneWithoutABodyIsReportedAtTheImpulse)
{
    sandbox::Scenario s = loaded();
    s.ragdolls[0].rigPath = "rigs/humanoid.rig.json";
    s.ragdolls[0].profilePath = "profiles/humanoid-default.profile.json";
    s.impulses[0].bone = "Neck"; // a bone, but without a body
    const sandbox::RunResult result =
        sandbox::run(s, sandbox::RunOptions{OPAS_REPO_DATA_DIR, "push.scenario.json"});
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.line, 24U);
    EXPECT_EQ(result.error.field, "/impulses/0/bone");
}

TEST(ScenarioRun, PoseSourcesByName)
{
    EXPECT_TRUE(sandbox::isPoseSourceName("rest"));
    EXPECT_TRUE(sandbox::isPoseSourceName("armWave"));
    EXPECT_FALSE(sandbox::isPoseSourceName("dance"));
    rig::Rig noArms;
    noArms.skeleton.bones.push_back(rig::Bone{"Root", -1, {}});
    std::string error;
    EXPECT_FALSE(sandbox::makePoseSource("armWave", noArms, &error).has_value());
    EXPECT_NE(error.find("RightUpperArm"), std::string::npos) << error;
    EXPECT_TRUE(sandbox::makePoseSource("rest", noArms, &error).has_value());
}

} // namespace
