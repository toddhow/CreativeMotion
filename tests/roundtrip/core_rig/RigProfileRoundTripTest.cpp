#include "RigTestData.hpp"
#include "opas/rig/ProfileFile.hpp"
#include "opas/rig/RigFile.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace {

using opas::math::Quat;
using opas::math::Vec3;
using opas::rig::loadProfile;
using opas::rig::loadRig;
using opas::rig::Profile;
using opas::rig::Rig;
using opas::rig::writeProfile;
using opas::rig::writeRig;
using opas::test::kProfileText;
using opas::test::kRigText;

TEST(RigRoundTrip, CanonicalTextIsReproducedByteForByte)
{
    const auto loaded = loadRig(kRigText, "synthetic.rig.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_EQ(writeRig(loaded.value.value_or(Rig{})), kRigText);
}

TEST(ProfileRoundTrip, CanonicalTextIsReproducedByteForByte)
{
    const auto loaded = loadProfile(kProfileText, "synthetic.profile.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_EQ(writeProfile(loaded.value.value_or(Profile{})), kProfileText);
}

// Numbers that are not short decimals (0.1 + 0.2, a normalized rotation)
// must survive write -> load -> write exactly.
TEST(RigRoundTrip, AwkwardNumbersSurviveExactly)
{
    const auto loaded = loadRig(kRigText, "synthetic.rig.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    Rig rig = loaded.value.value_or(Rig{});
    rig.skeleton.bones[1].localRest.translation = Vec3{0.1 + 0.2, -1.0 / 3.0, 1e-7};
    rig.bodies[2].local.rotation = opas::math::normalized(Quat{0.3, -0.2, 0.1, 0.9});
    rig.bodies[1].massKg = 2.0 / 3.0;
    rig.joints[1].planeHalfConeRad = std::nextafter(0.25, 1.0);

    const std::string text = writeRig(rig);
    const auto reloaded = loadRig(text, "written.rig.json");
    ASSERT_TRUE(reloaded.ok()) << reloaded.error.toString();
    EXPECT_EQ(reloaded.value.value_or(Rig{}), rig);
    EXPECT_EQ(writeRig(reloaded.value.value_or(Rig{})), text);
}

TEST(ProfileRoundTrip, AwkwardNumbersAndEmptyOverridesSurviveExactly)
{
    const auto loaded = loadProfile(kProfileText, "synthetic.profile.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    Profile profile = loaded.value.value_or(Profile{});
    profile.defaultMotor.dampingNmsPerRad = 0.1 + 0.2;
    profile.joints.clear();
    profile.bodies.clear();

    const std::string text = writeProfile(profile);
    EXPECT_NE(text.find("\"joints\": [],"), std::string::npos);
    const auto reloaded = loadProfile(text, "written.profile.json");
    ASSERT_TRUE(reloaded.ok()) << reloaded.error.toString();
    EXPECT_EQ(reloaded.value.value_or(Profile{}), profile);
    EXPECT_EQ(writeProfile(reloaded.value.value_or(Profile{})), text);
}

TEST(RigRoundTrip, NegativeZeroIsWrittenAsZero)
{
    const auto loaded = loadRig(kRigText, "synthetic.rig.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    Rig rig = loaded.value.value_or(Rig{});
    rig.bodies[0].local.translation.x = -0.0;
    EXPECT_EQ(writeRig(rig), kRigText);
}

// Quotes, backslashes, JSON-pointer characters and non-ASCII UTF-8 in names
// are escaped or passed through so the written file loads back as the same rig.
TEST(RigRoundTrip, SpecialCharactersInNamesSurvive)
{
    const auto loaded = loadRig(kRigText, "synthetic.rig.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    Rig rig = loaded.value.value_or(Rig{});
    rig.name = R"(rig "quoted" \ back/slash~tilde)";
    const std::string mid = "Mid-\xC3\x84-\xF0\x9F\xA6\xB4"; // "Mid-Ä-" and a 4-byte emoji
    rig.skeleton.bones[1].name = mid;
    rig.bodies[1].bone = mid;
    rig.joints[0].bone = mid;
    ASSERT_EQ(opas::rig::validateRig(rig), "");

    const std::string text = writeRig(rig);
    const auto reloaded = loadRig(text, "written.rig.json");
    ASSERT_TRUE(reloaded.ok()) << reloaded.error.toString();
    EXPECT_EQ(reloaded.value.value_or(Rig{}), rig);
    EXPECT_EQ(writeRig(reloaded.value.value_or(Rig{})), text);
}

} // namespace
