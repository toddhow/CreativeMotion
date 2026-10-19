#include "opas/rig/Profile.hpp"
#include "opas/rig/ProfileFile.hpp"
#include "opas/rig/RigFile.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

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

namespace {

using opas::math::Vec3;

// A body bounded by a capsule in world space: the segment a-b swept by radius.
// Capsules and spheres are exact; a box is bounded by the capsule along its
// longest axis, which is larger than the box, so the check is conservative.
struct Bound {
    Vec3 a;
    Vec3 b;
    double radiusM = 0.0;
};

Bound boundOf(const opas::rig::Body& body, const opas::math::Transform& boneWorld)
{
    const opas::math::Transform world = boneWorld * body.local;
    Vec3 halfAxis;
    double radius = 0.0;
    if (const auto* capsule = std::get_if<opas::rig::CapsuleShape>(&body.shape)) {
        halfAxis = Vec3{0.0, 0.0, capsule->halfHeightM};
        radius = capsule->radiusM;
    } else if (const auto* sphere = std::get_if<opas::rig::SphereShape>(&body.shape)) {
        radius = sphere->radiusM;
    } else {
        const Vec3 e = std::get<opas::rig::BoxShape>(body.shape).halfExtentsM;
        if (e.x >= e.y && e.x >= e.z) {
            halfAxis = Vec3{e.x, 0.0, 0.0};
            radius = std::hypot(e.y, e.z);
        } else if (e.y >= e.z) {
            halfAxis = Vec3{0.0, e.y, 0.0};
            radius = std::hypot(e.x, e.z);
        } else {
            halfAxis = Vec3{0.0, 0.0, e.z};
            radius = std::hypot(e.x, e.y);
        }
    }
    const Vec3 offset = opas::math::rotate(world.rotation, halfAxis);
    return {world.translation - offset, world.translation + offset, radius};
}

// Closest distance between segments p1-q1 and p2-q2 (Ericson, "Real-Time
// Collision Detection", 5.1.9).
double segmentDistance(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2)
{
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = opas::math::dot(d1, d1);
    const double e = opas::math::dot(d2, d2);
    const double f = opas::math::dot(d2, r);
    constexpr double kEps = 1e-12;
    double s = 0.0;
    double t = 0.0;
    if (a <= kEps && e <= kEps) {
        return opas::math::length(r);
    }
    if (a <= kEps) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = opas::math::dot(d1, r);
        if (e <= kEps) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = opas::math::dot(d1, d2);
            const double denom = a * e - b * b;
            s = denom > kEps ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    const Vec3 c1 = p1 + d1 * s;
    const Vec3 c2 = p2 + d2 * t;
    return opas::math::length(c1 - c2);
}

TEST(SampleHumanoid, HasTheBonesTheReadmeLists)
{
    EXPECT_EQ(sampleRig().skeleton.bones.size(), 19U);
}

// Bodies with no joint between them must not start inside each other, or the
// ragdoll pushes itself apart on its first step (M3).
TEST(SampleHumanoid, UnjointedBodiesDoNotOverlapAtRest)
{
    const Rig rig = sampleRig();
    const std::vector<opas::math::Transform> boneWorld = opas::rig::computeWorldRest(rig.skeleton);
    std::vector<Bound> bounds;
    for (const auto& body : rig.bodies) {
        const auto bone = opas::rig::findBone(rig.skeleton, body.bone);
        ASSERT_TRUE(bone.has_value()) << body.bone;
        bounds.push_back(boundOf(body, boneWorld[bone.value_or(0)]));
    }
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        for (std::size_t j = i + 1; j < bounds.size(); ++j) {
            if (parentBody(rig, i) == j || parentBody(rig, j) == i) {
                continue;
            }
            const double gapM =
                segmentDistance(bounds[i].a, bounds[i].b, bounds[j].a, bounds[j].b) -
                bounds[i].radiusM - bounds[j].radiusM;
            EXPECT_GT(gapM, 0.0) << rig.bodies[i].bone << " overlaps " << rig.bodies[j].bone;
        }
    }
}

} // namespace
