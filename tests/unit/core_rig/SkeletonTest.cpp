#include "opas/rig/Skeleton.hpp"
#include "TestSupport.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using opas::math::fromAxisAngle;
using opas::math::Quat;
using opas::math::rotate;
using opas::math::Transform;
using opas::math::Vec3;
using opas::rig::Bone;
using opas::rig::computeWorldRest;
using opas::rig::findBone;
using opas::rig::Skeleton;
using opas::rig::validateSkeleton;
using opas::test::contains;
using opas::test::kPi;
using opas::test::QuatSameRotation;
using opas::test::Vec3Near;

constexpr double kTol = 1e-12;
constexpr Vec3 kX{1.0, 0.0, 0.0};
constexpr Vec3 kY{0.0, 1.0, 0.0};
constexpr Vec3 kZ{0.0, 0.0, 1.0};

Bone makeBone(std::string name, int parent, Vec3 translation = {}, Quat rotation = {})
{
    return Bone{std::move(name), parent, Transform{translation, rotation}};
}

// Root -> Spine -> Head, all identity rotations: a valid starting point for the
// rejection tests, which each break exactly one thing.
Skeleton validChain()
{
    Skeleton s;
    s.bones.push_back(makeBone("Root", -1));
    s.bones.push_back(makeBone("Spine", 0, Vec3{0.0, 0.0, 1.0}));
    s.bones.push_back(makeBone("Head", 1, Vec3{0.0, 0.0, 0.5}));
    return s;
}

TEST(RigSkeleton, ValidSkeletonHasNoError)
{
    EXPECT_EQ(validateSkeleton(validChain()), "");

    // A rotated bone with a unit quaternion is fine, and so is a second root.
    Skeleton s = validChain();
    s.bones.push_back(makeBone("Other", -1, Vec3{1.0, 0.0, 0.0}, fromAxisAngle(kZ, 0.4)));
    EXPECT_EQ(validateSkeleton(s), "");
}

TEST(RigSkeleton, RejectsEmptyName)
{
    Skeleton s = validChain();
    s.bones[1].name = "";
    const std::string message = validateSkeleton(s);
    EXPECT_NE(message, "");
    EXPECT_TRUE(contains(message, "name")) << message;
    // The bone has no name to quote, so the message must identify it by index.
    EXPECT_TRUE(contains(message, "1")) << message;
}

TEST(RigSkeleton, RejectsDuplicateName)
{
    Skeleton s = validChain();
    s.bones[2].name = "Spine";
    const std::string message = validateSkeleton(s);
    EXPECT_NE(message, "");
    EXPECT_TRUE(contains(message, "Spine")) << message;
    EXPECT_TRUE(contains(message, "name")) << message;
}

TEST(RigSkeleton, RejectsParentEqualToOwnIndex)
{
    Skeleton s = validChain();
    s.bones[1].parent = 1;
    const std::string message = validateSkeleton(s);
    EXPECT_NE(message, "");
    EXPECT_TRUE(contains(message, "Spine")) << message;
    EXPECT_TRUE(contains(message, "parent")) << message;
}

TEST(RigSkeleton, RejectsParentAfterTheBone)
{
    Skeleton s = validChain();
    s.bones[0].parent = 2; // parents must come first
    const std::string message = validateSkeleton(s);
    EXPECT_NE(message, "");
    EXPECT_TRUE(contains(message, "Root")) << message;
    EXPECT_TRUE(contains(message, "parent")) << message;
}

TEST(RigSkeleton, RejectsParentIndexOutOfRangeBelowMinusOne)
{
    Skeleton s = validChain();
    s.bones[2].parent = -2;
    const std::string message = validateSkeleton(s);
    EXPECT_NE(message, "");
    EXPECT_TRUE(contains(message, "Head")) << message;
    EXPECT_TRUE(contains(message, "parent")) << message;
}

TEST(RigSkeleton, RejectsNonUnitRotation)
{
    const Quat badRotations[] = {
        Quat{0.0, 0.0, 0.0, 2.0},   // too long
        Quat{0.0, 0.0, 0.0, 0.0},   // zero
        Quat{0.1, 0.0, 0.0, 1.0},   // not normalized
        Quat{0.0, 0.0, 0.0, 1.001}, // test choice: 1e-3 off counts as non-unit
    };
    for (const Quat& bad : badRotations) {
        SCOPED_TRACE(opas::test::str(bad));
        Skeleton s = validChain();
        s.bones[1].localRest.rotation = bad;
        const std::string message = validateSkeleton(s);
        EXPECT_NE(message, "");
        EXPECT_TRUE(contains(message, "Spine")) << message;
        EXPECT_TRUE(contains(message, "rotation")) << message;
    }
}

TEST(RigSkeleton, FindBoneReturnsIndexOrNothing)
{
    const Skeleton s = validChain();
    ASSERT_TRUE(findBone(s, "Root").has_value());
    EXPECT_EQ(findBone(s, "Root").value_or(99U), 0U);
    ASSERT_TRUE(findBone(s, "Head").has_value());
    EXPECT_EQ(findBone(s, "Head").value_or(99U), 2U);
    EXPECT_FALSE(findBone(s, "Tail").has_value());
    EXPECT_FALSE(findBone(s, "").has_value());
    EXPECT_FALSE(findBone(s, "root").has_value()); // names are case-sensitive
}

TEST(RigSkeleton, ComputeWorldRestMatchesHandComputedChain)
{
    // Rz90: (x,y,z) -> (-y,x,z).  Rx90: (x,y,z) -> (x,-z,y).
    const Quat rz = fromAxisAngle(kZ, kPi / 2.0);
    const Quat rx = fromAxisAngle(kX, kPi / 2.0);

    Skeleton s;
    s.bones.push_back(makeBone("Root", -1, Vec3{1.0, 0.0, 0.0}, rz));
    s.bones.push_back(makeBone("Mid", 0, Vec3{0.0, 2.0, 0.0}, rx));
    s.bones.push_back(makeBone("Tip", 1, Vec3{0.0, 0.0, 3.0}, rz));
    ASSERT_EQ(validateSkeleton(s), "");

    const std::vector<Transform> world = computeWorldRest(s);
    ASSERT_EQ(world.size(), 3U);

    // Root: as authored.
    EXPECT_PRED_FORMAT3(Vec3Near, world[0].translation, (Vec3{1.0, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[0].rotation, kX), kY, kTol);

    // Mid: Rz90 * (0,2,0) = (-2,0,0); + (1,0,0) = (-1,0,0). Rotation Rz90*Rx90: x -> y, y -> z.
    EXPECT_PRED_FORMAT3(Vec3Near, world[1].translation, (Vec3{-1.0, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[1].rotation, kX), kY, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[1].rotation, kY), kZ, kTol);

    // Tip: (Rz90*Rx90) * (0,0,3): Rx90 -> (0,-3,0), Rz90 -> (3,0,0); + (-1,0,0) = (2,0,0).
    // Rotation Rz90*Rx90*Rz90: x -> y -> z -> z, so x -> z; y -> -x -> -x -> -y.
    EXPECT_PRED_FORMAT3(Vec3Near, world[2].translation, (Vec3{2.0, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[2].rotation, kX), kZ, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[2].rotation, kY), (Vec3{0.0, -1.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(QuatSameRotation, world[2].rotation, rz * rx * rz, kTol);
}

TEST(RigSkeleton, ComputeWorldRestHandlesABranchAndASecondRoot)
{
    const Quat rz = fromAxisAngle(kZ, kPi / 2.0);

    Skeleton s;
    s.bones.push_back(makeBone("Root", -1, Vec3{1.0, 0.0, 0.0}, rz));
    s.bones.push_back(makeBone("A", 0, Vec3{0.0, 2.0, 0.0}));
    s.bones.push_back(makeBone("B", 0, Vec3{0.0, 0.0, 1.0})); // sibling of A
    s.bones.push_back(makeBone("Free", -1, Vec3{5.0, 6.0, 7.0}));
    ASSERT_EQ(validateSkeleton(s), "");

    const std::vector<Transform> world = computeWorldRest(s);
    ASSERT_EQ(world.size(), 4U);
    // A: Rz90 * (0,2,0) + (1,0,0) = (-2,0,0) + (1,0,0) = (-1,0,0)
    EXPECT_PRED_FORMAT3(Vec3Near, world[1].translation, (Vec3{-1.0, 0.0, 0.0}), kTol);
    // B: Rz90 * (0,0,1) + (1,0,0) = (1,0,1); inherits the parent rotation.
    EXPECT_PRED_FORMAT3(Vec3Near, world[2].translation, (Vec3{1.0, 0.0, 1.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[2].rotation, kX), kY, kTol);
    // A second root is not affected by the first.
    EXPECT_PRED_FORMAT3(Vec3Near, world[3].translation, (Vec3{5.0, 6.0, 7.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[3].rotation, kX), kX, kTol);
}

// ---- Amendments after the fresh review: non-finite values ----------------------

TEST(RigSkeleton, RejectsNonFiniteRotation)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Quat bad[] = {Quat{nan, 0.0, 0.0, 1.0}, Quat{0.0, 0.0, 0.0, nan},
                        Quat{inf, 0.0, 0.0, 1.0}};
    for (const Quat& q : bad) {
        SCOPED_TRACE(opas::test::str(q));
        Skeleton s = validChain();
        s.bones[1].localRest.rotation = q;
        const std::string message = validateSkeleton(s);
        EXPECT_NE(message, "");
        EXPECT_TRUE(contains(message, "Spine")) << message;
        EXPECT_TRUE(contains(message, "rotation")) << message;
    }
}

TEST(RigSkeleton, RejectsNonFiniteTranslation)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Vec3 bad[] = {Vec3{nan, 0.0, 0.0}, Vec3{0.0, inf, 0.0}, Vec3{0.0, 0.0, -inf}};
    for (const Vec3& t : bad) {
        SCOPED_TRACE(opas::test::str(t));
        Skeleton s = validChain();
        s.bones[2].localRest.translation = t;
        const std::string message = validateSkeleton(s);
        EXPECT_NE(message, "");
        EXPECT_TRUE(contains(message, "Head")) << message;
        EXPECT_TRUE(contains(message, "translation")) << message;
    }
}

// ---- computeWorldPose (A2) ----------------------------------------------------------

TEST(RigSkeleton, ComputeWorldPoseMatchesHandComputedChainWithANonIdentityRoot)
{
    const Quat rz = fromAxisAngle(kZ, kPi / 2.0);
    const Quat rx = fromAxisAngle(kX, kPi / 2.0);
    const Quat ry = fromAxisAngle(kY, kPi / 2.0);

    // The rest pose is deliberately different from the pose: only the pose may matter.
    Skeleton s;
    s.bones.push_back(makeBone("Root", -1, Vec3{9.0, 9.0, 9.0}));
    s.bones.push_back(makeBone("Mid", 0, Vec3{9.0, 9.0, 9.0}));
    s.bones.push_back(makeBone("Tip", 1, Vec3{9.0, 9.0, 9.0}));
    ASSERT_EQ(validateSkeleton(s), "");

    const std::vector<Transform> pose = {
        Transform{Vec3{1.0, 0.0, 0.0}, rz},
        Transform{Vec3{0.0, 2.0, 0.0}, rx},
        Transform{Vec3{0.0, 0.0, 3.0}, rz},
    };
    const Transform root{Vec3{10.0, 20.0, 30.0}, ry};

    const std::vector<Transform> world = computeWorldPose(s, pose, root);
    ASSERT_EQ(world.size(), 3U);

    // In model space (see ComputeWorldRestMatchesHandComputedChain, same numbers) the bones are at
    // (1,0,0), (-1,0,0), (2,0,0). Ry90 maps (x,y,z) -> (z,y,-x), so in the world:
    //   (1,0,0) -> (0,0,-1), (-1,0,0) -> (0,0,1), (2,0,0) -> (0,0,-2), each plus (10,20,30).
    EXPECT_PRED_FORMAT3(Vec3Near, world[0].translation, (Vec3{10.0, 20.0, 29.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, world[1].translation, (Vec3{10.0, 20.0, 31.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, world[2].translation, (Vec3{10.0, 20.0, 28.0}), kTol);

    // Model-space rotations: bone 0 sends x -> y, y -> -x; bone 1 sends x -> y, y -> z;
    // bone 2 sends x -> z, y -> -y. Then Ry90 on top: y -> y, z -> x, -x -> z, x -> -z.
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[0].rotation, kX), kY, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[0].rotation, kY), kZ, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[1].rotation, kX), kY, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[1].rotation, kY), kX, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[2].rotation, kX), kX, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world[2].rotation, kY), (Vec3{0.0, -1.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(QuatSameRotation, world[2].rotation, ry * rz * rx * rz, kTol);
}

TEST(RigSkeleton, ComputeWorldPoseWithRestPoseAndIdentityRootEqualsComputeWorldRest)
{
    const Quat rz = fromAxisAngle(kZ, kPi / 2.0);
    const Quat rx = fromAxisAngle(kX, kPi / 2.0);
    Skeleton s;
    s.bones.push_back(makeBone("Root", -1, Vec3{1.0, 0.0, 0.0}, rz));
    s.bones.push_back(makeBone("Mid", 0, Vec3{0.0, 2.0, 0.0}, rx));
    s.bones.push_back(makeBone("Tip", 1, Vec3{0.0, 0.0, 3.0}, rz));
    std::vector<Transform> pose;
    pose.reserve(s.bones.size());
    for (const auto& bone : s.bones) {
        pose.push_back(bone.localRest);
    }
    const std::vector<Transform> rest = computeWorldRest(s);
    const std::vector<Transform> posed = computeWorldPose(s, pose, Transform{});
    ASSERT_EQ(posed.size(), rest.size());
    for (std::size_t i = 0; i < rest.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_PRED_FORMAT3(Vec3Near, posed[i].translation, rest[i].translation, kTol);
        EXPECT_PRED_FORMAT3(QuatSameRotation, posed[i].rotation, rest[i].rotation, kTol);
    }
}

} // namespace
