#pragma once

// A synthetic humanoid for the alignment tests (A2): skeleton, body mapping and
// a small forward-kinematics helper. Built from numbers in this file only; no
// game data.
//
// Conventions: Z up, the actor faces +Y at identity root, so its right side is
// +X (right-handed coordinates). At scale 1 (all lengths in meters):
//
//   Pelvis (root bone)  z = 1.00
//   Spine               z = 1.15
//   Chest               z = 1.40
//   Neck                z = 1.60
//   Head                z = 1.75
//   Shoulders (upper arm bones)  x = +-0.20, z = 1.48
//   Upper arm 0.34 m, forearm 0.32 m, hanging straight down in the rest pose
//   Thigh x = +-0.09, z = 0.95; shin 0.45 m; foot z = 0.05
//
// `scale` multiplies every translation, so a scale 1.25 actor is 1.25 times
// taller, wider and longer-armed than the nominal one.

#include "opas/ik/TwoBoneIk.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"
#include "opas/rig/Skeleton.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace opas::test {

// Bone indices of the synthetic humanoid (parents come first).
namespace bone {
inline constexpr std::size_t kPelvis = 0;
inline constexpr std::size_t kSpine = 1;
inline constexpr std::size_t kChest = 2;
inline constexpr std::size_t kNeck = 3;
inline constexpr std::size_t kHead = 4;
inline constexpr std::size_t kRightUpperArm = 5;
inline constexpr std::size_t kRightForearm = 6;
inline constexpr std::size_t kRightHand = 7;
inline constexpr std::size_t kLeftUpperArm = 8;
inline constexpr std::size_t kLeftForearm = 9;
inline constexpr std::size_t kLeftHand = 10;
inline constexpr std::size_t kRightThigh = 11;
inline constexpr std::size_t kRightShin = 12;
inline constexpr std::size_t kRightFoot = 13;
inline constexpr std::size_t kLeftThigh = 14;
inline constexpr std::size_t kLeftShin = 15;
inline constexpr std::size_t kLeftFoot = 16;
inline constexpr std::size_t kCount = 17;
} // namespace bone

struct Humanoid {
    double scale = 1.0;
    rig::Skeleton skeleton;
    markers::BodyMapping mapping;
    std::vector<math::Transform> restPose; // local, one per bone (= each bone's localRest)
};

// Nominal sizes (meters, scale 1).
inline constexpr double kUpperArmLengthM = 0.36;
inline constexpr double kForearmLengthM = 0.34;
inline constexpr double kShoulderHalfWidthM = 0.18;
inline constexpr double kShoulderHeightAboveChestM = 0.08;
inline constexpr double kLimbRadiusM = 0.025; // IK chain limb thickness, scale 1

namespace detail {

inline rig::Bone makeBone(std::string name, int parent, math::Vec3 translation, double scale)
{
    return rig::Bone{std::move(name), parent, math::Transform{translation * scale, math::Quat{}}};
}

inline markers::IkChain makeArmChain(const std::string& side, double outwardSign, double scale)
{
    markers::IkChain chain;
    chain.name = side + "Arm";
    chain.rootBone = side + "UpperArm";
    chain.midBone = side + "Forearm";
    chain.endBone = side + "Hand";
    // Elbow from straight (0) to 150 degrees folded.
    chain.limits = ik::HingeLimits{0.0, 2.6};
    // Elbows point down, a little back and a little outward, as on a relaxed arm.
    chain.poleDirection = math::normalized(math::Vec3{0.35 * outwardSign, -0.2, -0.9});
    chain.limbRadiusM = kLimbRadiusM * scale;
    return chain;
}

} // namespace detail

inline Humanoid makeHumanoid(double scale)
{
    using math::Vec3;
    Humanoid h;
    h.scale = scale;
    const double sx = kShoulderHalfWidthM;
    const double sz = kShoulderHeightAboveChestM;
    h.skeleton.bones = {
        detail::makeBone("Pelvis", -1, Vec3{0.0, 0.0, 1.0}, scale),
        detail::makeBone("Spine", 0, Vec3{0.0, 0.0, 0.15}, scale),
        detail::makeBone("Chest", 1, Vec3{0.0, 0.0, 0.25}, scale),
        detail::makeBone("Neck", 2, Vec3{0.0, 0.0, 0.20}, scale),
        detail::makeBone("Head", 3, Vec3{0.0, 0.0, 0.15}, scale),
        detail::makeBone("RightUpperArm", 2, Vec3{sx, 0.0, sz}, scale),
        detail::makeBone("RightForearm", 5, Vec3{0.0, 0.0, -kUpperArmLengthM}, scale),
        detail::makeBone("RightHand", 6, Vec3{0.0, 0.0, -kForearmLengthM}, scale),
        detail::makeBone("LeftUpperArm", 2, Vec3{-sx, 0.0, sz}, scale),
        detail::makeBone("LeftForearm", 8, Vec3{0.0, 0.0, -kUpperArmLengthM}, scale),
        detail::makeBone("LeftHand", 9, Vec3{0.0, 0.0, -kForearmLengthM}, scale),
        detail::makeBone("RightThigh", 0, Vec3{0.09, 0.0, -0.05}, scale),
        detail::makeBone("RightShin", 11, Vec3{0.0, 0.0, -0.45}, scale),
        detail::makeBone("RightFoot", 12, Vec3{0.0, 0.0, -0.45}, scale),
        detail::makeBone("LeftThigh", 0, Vec3{-0.09, 0.0, -0.05}, scale),
        detail::makeBone("LeftShin", 14, Vec3{0.0, 0.0, -0.45}, scale),
        detail::makeBone("LeftFoot", 15, Vec3{0.0, 0.0, -0.45}, scale),
    };
    for (const rig::Bone& b : h.skeleton.bones) {
        h.restPose.push_back(b.localRest);
    }

    // Body regions. Capsule radius = ratio * the capsule's length (bone to endBone).
    // The torso capsule only covers the abdomen (pelvis to spine) so it does not overlap
    // the upper-back capsule; markers on the back sit far from it.
    h.mapping.capsules = {
        {"Torso", "Pelvis", "Spine", 0.50},                // 0.15 m long, radius 0.075 m
        {"UpperBack", "Spine", "Neck", 0.1667},            // 0.45 m long, radius 0.075 m
        {"Head", "Neck", "Head", 0.40},                    // 0.15 m long, radius 0.06 m
        {"RightShoulder", "Chest", "RightUpperArm", 0.15}, // 0.19 m long, radius 0.029 m
        {"LeftShoulder", "Chest", "LeftUpperArm", 0.15},
        {"RightUpperArm", "RightUpperArm", "RightForearm", 0.08}, // radius 0.029 m
        {"RightForearm", "RightForearm", "RightHand", 0.08},      // radius 0.027 m
        {"LeftUpperArm", "LeftUpperArm", "LeftForearm", 0.08},
        {"LeftForearm", "LeftForearm", "LeftHand", 0.08},
        {"RightThigh", "RightThigh", "RightShin", 0.12},
        {"RightShin", "RightShin", "RightFoot", 0.10},
        {"LeftThigh", "LeftThigh", "LeftShin", 0.12},
        {"LeftShin", "LeftShin", "LeftFoot", 0.10},
    };
    h.mapping.limbs = {
        {"RightArm", {"RightUpperArm", "RightForearm", "RightHand"}},
        {"LeftArm", {"LeftUpperArm", "LeftForearm", "LeftHand"}},
        {"RightLeg", {"RightThigh", "RightShin", "RightFoot"}},
        {"LeftLeg", {"LeftThigh", "LeftShin", "LeftFoot"}},
    };
    h.mapping.ikChains = {
        detail::makeArmChain("Right", 1.0, scale),
        detail::makeArmChain("Left", -1.0, scale),
    };
    return h;
}

// Forward kinematics, written independently of rig::computeWorldPose so the alignment
// tests do not depend on it: world transform of every bone for a local pose and a root.
inline std::vector<math::Transform> worldPose(const rig::Skeleton& skeleton,
                                              const std::vector<math::Transform>& localPose,
                                              const math::Transform& root)
{
    std::vector<math::Transform> world(skeleton.bones.size());
    for (std::size_t i = 0; i < skeleton.bones.size(); ++i) {
        const int parent = skeleton.bones[i].parent;
        const math::Transform& parentWorld =
            parent < 0 ? root : world[static_cast<std::size_t>(parent)];
        world[i] = parentWorld * localPose[i];
    }
    return world;
}

// Arm pose helper: raises the arm forward by pitchRad (0 = hanging down, pi/2 = pointing
// forward), swings it outward by outwardRad about the vertical axis, and bends the elbow by
// elbowRad (positive curls the forearm up and forward). `side` is +1 for the right arm and
// -1 for the left arm.
inline void setArmPose(std::vector<math::Transform>& pose, double side, double pitchRad,
                       double outwardRad, double elbowRad)
{
    const std::size_t upper = side > 0.0 ? bone::kRightUpperArm : bone::kLeftUpperArm;
    const std::size_t fore = side > 0.0 ? bone::kRightForearm : bone::kLeftForearm;
    const math::Vec3 xAxis{1.0, 0.0, 0.0};
    const math::Vec3 zAxis{0.0, 0.0, 1.0};
    // Rotating +Y about +Z by a positive angle swings it towards -X (left), so the right arm
    // goes outward with a negative angle.
    pose[upper].rotation = math::normalized(math::fromAxisAngle(zAxis, -side * outwardRad) *
                                            math::fromAxisAngle(xAxis, pitchRad));
    pose[fore].rotation = math::fromAxisAngle(xAxis, elbowRad);
}

} // namespace opas::test
