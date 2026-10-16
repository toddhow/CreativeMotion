#include "opas/rig/Skeleton.hpp"
#include "TestSupport.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <cstddef>
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
    EXPECT_EQ(*findBone(s, "Root"), 0U);
    ASSERT_TRUE(findBone(s, "Head").has_value());
    EXPECT_EQ(*findBone(s, "Head"), 2U);
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

} // namespace
