#include "opas/rig/Profile.hpp"
#include "opas/rig/ProfileFile.hpp"
#include "opas/rig/RigFile.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

// The sample humanoid in data/ (data/rigs/README.md): it loads, validates,
// matches the spec's shape (docs/tasks/0012) and is stored in canonical form.

namespace {

using opas::rig::findBody;
using opas::rig::loadProfileFile;
using opas::rig::loadRigFile;
using opas::rig::parentBody;
using opas::rig::Profile;
using opas::rig::Rig;
using opas::rig::validateProfile;
using opas::rig::validateRig;
using opas::rig::validateSkeleton;
using opas::rig::writeProfile;
using opas::rig::writeRig;

constexpr const char* kRigPath = OPAS_REPO_DATA_DIR "/rigs/humanoid.rig.json";
constexpr const char* kProfilePath = OPAS_REPO_DATA_DIR "/profiles/humanoid-default.profile.json";

std::string fileText(const char* path)
{
    const std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

Rig sampleRig()
{
    const auto result = loadRigFile(kRigPath);
    EXPECT_TRUE(result.ok()) << result.error.toString();
    return result.value.value_or(Rig{});
}

TEST(SampleHumanoid, RigLoadsAndValidates)
{
    const Rig rig = sampleRig();
    EXPECT_EQ(rig.name, "humanoid");
    EXPECT_EQ(validateRig(rig), "");
    EXPECT_EQ(validateSkeleton(rig.skeleton), "");
    EXPECT_EQ(rig.bodies.size(), 16U);
    EXPECT_EQ(rig.joints.size(), 15U);
}

TEST(SampleHumanoid, TotalMassIs75Kg)
{
    double totalKg = 0.0;
    for (const auto& body : sampleRig().bodies) {
        totalKg += body.massKg;
    }
    EXPECT_NEAR(totalKg, 75.0, 0.01);
}

TEST(SampleHumanoid, RootBodyIsOnThePelvisAndEveryOtherBodyHasAJoint)
{
    const Rig rig = sampleRig();
    const auto pelvis = findBody(rig, "Pelvis");
    ASSERT_TRUE(pelvis.has_value());
    EXPECT_FALSE(parentBody(rig, pelvis.value_or(0)).has_value());
    for (std::size_t i = 0; i < rig.bodies.size(); ++i) {
        if (i != pelvis) {
            EXPECT_TRUE(parentBody(rig, i).has_value()) << rig.bodies[i].bone;
            EXPECT_TRUE(opas::rig::findJoint(rig, rig.bodies[i].bone).has_value())
                << rig.bodies[i].bone;
        }
    }
    // The neck has no body: the head joins the chest.
    const auto head = findBody(rig, "Head");
    ASSERT_TRUE(head.has_value());
    EXPECT_EQ(parentBody(rig, head.value_or(0)), findBody(rig, "Chest"));
}

TEST(SampleHumanoid, ProfileLoadsAndFitsTheRig)
{
    const auto result = loadProfileFile(kProfilePath);
    ASSERT_TRUE(result.ok()) << result.error.toString();
    EXPECT_EQ(validateProfile(result.value.value_or(Profile{}), sampleRig()), "");
}

TEST(SampleHumanoid, FilesAreInCanonicalForm)
{
    EXPECT_EQ(writeRig(sampleRig()), fileText(kRigPath));
    const auto profile = loadProfileFile(kProfilePath);
    ASSERT_TRUE(profile.ok()) << profile.error.toString();
    EXPECT_EQ(writeProfile(profile.value.value_or(Profile{})), fileText(kProfilePath));
}

} // namespace
