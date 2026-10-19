#include "RigTestData.hpp"
#include "opas/rig/Profile.hpp"
#include "opas/rig/ProfileFile.hpp"
#include "opas/rig/RigFile.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <string_view>

namespace {

using opas::rig::loadProfile;
using opas::rig::loadRig;
using opas::rig::materialFor;
using opas::rig::motorFor;
using opas::rig::Profile;
using opas::rig::ProfileLoadResult;
using opas::rig::Rig;
using opas::rig::validateProfile;
using opas::test::kProfileText;
using opas::test::kRigText;

constexpr const char* kFile = "synthetic.profile.json";

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

Profile loadedTestProfile()
{
    const ProfileLoadResult result = loadProfile(kProfileText, kFile);
    EXPECT_TRUE(result.ok()) << result.error.toString();
    return result.value.value_or(Profile{});
}

Rig loadedTestRig()
{
    const auto result = loadRig(kRigText, "synthetic.rig.json");
    EXPECT_TRUE(result.ok()) << result.error.toString();
    return result.value.value_or(Rig{});
}

bool startsWith(const std::string& text, std::string_view prefix)
{
    return text.rfind(prefix, 0) == 0;
}

TEST(ProfileLoad, LoadsTheSyntheticProfile)
{
    const Profile profile = loadedTestProfile();
    EXPECT_EQ(profile.rig, "test-rig");
    EXPECT_DOUBLE_EQ(profile.defaultMotor.stiffnessNmPerRad, 300.0);
    EXPECT_DOUBLE_EQ(profile.defaultMaterial.friction, 0.8);
    ASSERT_EQ(profile.joints.size(), 1U);
    EXPECT_EQ(profile.joints[0].bone, "Mid");
    ASSERT_EQ(profile.bodies.size(), 1U);
    EXPECT_DOUBLE_EQ(profile.bodies[0].material.angularDamping, 0.3);
    EXPECT_EQ(validateProfile(profile, loadedTestRig()), "");
}

TEST(ProfileLoad, OverridesReplaceTheDefaultsWhole)
{
    const Profile profile = loadedTestProfile();
    EXPECT_DOUBLE_EQ(motorFor(profile, "Mid").stiffnessNmPerRad, 600.0);
    EXPECT_DOUBLE_EQ(motorFor(profile, "Mid").maxTorqueNm, 900.0);
    EXPECT_EQ(motorFor(profile, "Ball"), profile.defaultMotor);
    EXPECT_DOUBLE_EQ(materialFor(profile, "Ball").restitution, 0.1);
    EXPECT_EQ(materialFor(profile, "Root"), profile.defaultMaterial);
}

struct Edit {
    const char* name;
    const char* from;
    const char* to;
    std::size_t line;
    const char* field;
};

class ProfileRejects : public ::testing::TestWithParam<Edit> {};

TEST_P(ProfileRejects, WithLineAndField)
{
    const Edit& e = GetParam();
    const ProfileLoadResult result = loadProfile(replaceOnce(kProfileText, e.from, e.to), kFile);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.error.file, kFile);
    EXPECT_EQ(result.error.line, e.line) << result.error.toString();
    EXPECT_EQ(result.error.field, e.field) << result.error.toString();
}

const Edit kEdits[] = {
    {"SyntaxError", R"("rig": "test-rig",)", R"("rig": "test-rig")", 5, ""},
    {"WrongFormat", R"("opas.profile")", R"("opas.rig")", 2, "/format"},
    {"WrongVersion", R"("version": 1)", R"("version": 2)", 3, "/version"},
    {"EmptyRig", R"("rig": "test-rig")", R"("rig": "")", 4, "/rig"},
    {"UnknownMaterialKey", R"("angularDamping": 0.05})", R"("angularDamping": 0.05, "bounce": 1})",
     7, "/defaults/material/bounce"},
    {"MissingDefaultMotor",
     "\"motor\": {\"stiffnessNmPerRad\": 300, \"dampingNmsPerRad\": 30, \"maxTorqueNm\": 500},\n   "
     " ",
     "", 5, "/defaults/motor"},
    {"NegativeDefaultStiffness", R"("stiffnessNmPerRad": 300)", R"("stiffnessNmPerRad": -1)", 6,
     "/defaults/motor/stiffnessNmPerRad"},
    {"ZeroMaxTorque", R"("maxTorqueNm": 500)", R"("maxTorqueNm": 0)", 6,
     "/defaults/motor/maxTorqueNm"},
    {"NegativeOverrideDamping", R"("dampingNmsPerRad": 60)", R"("dampingNmsPerRad": -5)", 10,
     "/joints/0/motor/dampingNmsPerRad"},
    {"NegativeFriction", R"("friction": 1,)", R"("friction": -1,)", 13,
     "/bodies/0/material/friction"},
    {"NegativeRestitution", R"("restitution": 0,)", R"("restitution": -0.5,)", 7,
     "/defaults/material/restitution"},
    {"NegativeLinearDamping", R"("linearDamping": 0.2)", R"("linearDamping": -0.2)", 13,
     "/bodies/0/material/linearDamping"},
    {"NonFiniteGain", R"("stiffnessNmPerRad": 600)", R"("stiffnessNmPerRad": 1e999)", 10,
     "/joints/0/motor/stiffnessNmPerRad"},
    {"DuplicateJointOverride", R"({"bone": "Mid", "motor")",
     R"({"bone": "Mid", "motor": {"stiffnessNmPerRad": 1, "dampingNmsPerRad": 1, "maxTorqueNm": 1}},
    {"bone": "Mid", "motor")",
     11, "/joints/1/bone"},
    {"DuplicateBodyOverride", R"({"bone": "Ball", "material")",
     R"({"bone": "Ball", "material": {"friction": 1, "restitution": 0, "linearDamping": 0, "angularDamping": 0}},
    {"bone": "Ball", "material")",
     14, "/bodies/1/bone"},
    {"OverrideNotAnObject", R"({"bone": "Ball", "material")", R"(3, {"bone": "Ball", "material")",
     13, "/bodies/0"},
};

INSTANTIATE_TEST_SUITE_P(Edits, ProfileRejects, ::testing::ValuesIn(kEdits),
                         [](const ::testing::TestParamInfo<Edit>& info) {
                             return std::string(info.param.name);
                         });

TEST(ProfileValidate, RigNameMustMatch)
{
    Profile profile = loadedTestProfile();
    profile.rig = "other-rig";
    EXPECT_TRUE(startsWith(validateProfile(profile, loadedTestRig()), "/rig: "))
        << validateProfile(profile, loadedTestRig());
}

TEST(ProfileValidate, JointOverrideNeedsAJointInTheRig)
{
    Profile profile = loadedTestProfile();
    profile.joints[0].bone = "Root"; // the root body has no joint
    EXPECT_TRUE(startsWith(validateProfile(profile, loadedTestRig()), "/joints/0/bone: "))
        << validateProfile(profile, loadedTestRig());
}

TEST(ProfileValidate, BodyOverrideNeedsABodyInTheRig)
{
    Profile profile = loadedTestProfile();
    profile.bodies[0].bone = "Tip"; // Tip has no body
    EXPECT_TRUE(startsWith(validateProfile(profile, loadedTestRig()), "/bodies/0/bone: "))
        << validateProfile(profile, loadedTestRig());
}

TEST(ProfileValidate, ChecksTheProfilesOwnRulesToo)
{
    Profile profile = loadedTestProfile();
    profile.defaultMaterial.friction = -1.0;
    EXPECT_EQ(validateProfile(profile, loadedTestRig()),
              "/defaults/material/friction: must be >= 0");
}

TEST(ProfileValidate, RejectsNamesThatAreNotCleanUtf8)
{
    Profile profile = loadedTestProfile();
    profile.bodies[0].bone = "Ball\x7F";
    EXPECT_EQ(validateProfile(profile, loadedTestRig()),
              "/bodies/0/bone: must be valid UTF-8 without control characters");
    profile = loadedTestProfile();
    profile.rig = "\xE2\x82"; // truncated 3-byte sequence
    EXPECT_TRUE(startsWith(validateProfile(profile, loadedTestRig()), "/rig: "))
        << validateProfile(profile, loadedTestRig());
}

} // namespace
