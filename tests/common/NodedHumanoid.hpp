#pragma once

// The synthetic humanoid with intermediate nodes between the bones of its limb chains, like the
// extra "CME" nodes in XPMSSE's skeleton (A4c-1). Same sizes and body regions as
// SyntheticHumanoid.hpp; extra bones, each with a NON-identity local transform:
//
//   arm:  UpperArm -> ForearmRoll -> Forearm -> Hand        (one node between upper arm and
//   forearm) leg:  Thigh -> Shin -> ShinNodeA -> ShinNodeB -> Foot   (two nodes between shin and
//   foot)
//
// IK chains: both arms (root UpperArm, mid Forearm, end Hand) and both legs (root Thigh, mid Shin,
// end Foot). The mid bone of an arm chain is two levels below its root, the end bone of a leg chain
// three levels below its mid bone.

#include "SyntheticHumanoid.hpp"
#include "opas/ik/TwoBoneIk.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"
#include "opas/rig/Skeleton.hpp"

#include <string>
#include <utility>
#include <vector>

namespace opas::test {

namespace detail {

inline rig::Bone nodedBone(std::string name, int parent, math::Vec3 translation,
                           math::Quat rotation, double scale)
{
    return rig::Bone{std::move(name), parent, math::Transform{translation * scale, rotation}};
}

} // namespace detail

inline Humanoid makeNodedHumanoid(double scale)
{
    using math::fromAxisAngle;
    using math::Quat;
    using math::Vec3;
    const Vec3 xAxis{1.0, 0.0, 0.0};
    const Vec3 yAxis{0.0, 1.0, 0.0};

    Humanoid h;
    h.scale = scale;
    std::vector<rig::Bone>& bones = h.skeleton.bones;
    const auto add = [&](std::string name, int parent, Vec3 t, Quat q = Quat{}) {
        bones.push_back(detail::nodedBone(std::move(name), parent, t, q, scale));
        return static_cast<int>(bones.size()) - 1;
    };

    const int pelvis = add("Pelvis", -1, {0.0, 0.0, 1.0});
    const int spine = add("Spine", pelvis, {0.0, 0.0, 0.15});
    const int chest = add("Chest", spine, {0.0, 0.0, 0.25});
    const int neck = add("Neck", chest, {0.0, 0.0, 0.20});
    add("Head", neck, {0.0, 0.0, 0.15});

    // side: +1 right, -1 left. Rotations of the nodes are mirrored with the side.
    for (const double side : {1.0, -1.0}) {
        const std::string prefix = side > 0.0 ? "Right" : "Left";
        const int upper = add(prefix + "UpperArm", chest,
                              {side * kShoulderHalfWidthM, 0.0, kShoulderHeightAboveChestM});
        const int roll = add(prefix + "ForearmRoll", upper, {0.0, 0.01, -0.18},
                             fromAxisAngle(xAxis, 0.2 * side));
        const int fore =
            add(prefix + "Forearm", roll, {0.0, -0.005, -0.18}, fromAxisAngle(xAxis, -0.1 * side));
        add(prefix + "Hand", fore, {0.0, 0.0, -kForearmLengthM}, fromAxisAngle(yAxis, 0.1 * side));
    }
    for (const double side : {1.0, -1.0}) {
        const std::string prefix = side > 0.0 ? "Right" : "Left";
        const int thigh = add(prefix + "Thigh", pelvis, {side * 0.09, 0.0, -0.05});
        const int shin = add(prefix + "Shin", thigh, {0.0, 0.0, -0.45});
        const int nodeA = add(prefix + "ShinNodeA", shin, {side * 0.01, 0.0, -0.20},
                              fromAxisAngle(yAxis, 0.15 * side));
        const int nodeB = add(prefix + "ShinNodeB", nodeA, {side * -0.01, 0.005, -0.15},
                              fromAxisAngle(yAxis, -0.3 * side));
        add(prefix + "Foot", nodeB, {0.0, 0.0, -0.10}, fromAxisAngle(yAxis, 0.15 * side));
    }
    for (const rig::Bone& b : bones) {
        h.restPose.push_back(b.localRest);
    }

    // Body regions and limbs as in the plain humanoid (all their bones exist here too).
    const Humanoid plain = makeHumanoid(scale);
    h.mapping.capsules = plain.mapping.capsules;
    h.mapping.limbs = plain.mapping.limbs;
    h.mapping.ikChains = plain.mapping.ikChains; // arms: UpperArm / Forearm / Hand
    for (const double side : {1.0, -1.0}) {
        const std::string prefix = side > 0.0 ? "Right" : "Left";
        markers::IkChain leg;
        leg.name = prefix + "Leg";
        leg.rootBone = prefix + "Thigh";
        leg.midBone = prefix + "Shin";
        leg.endBone = prefix + "Foot";
        leg.limits = ik::HingeLimits{0.0, 2.4};
        leg.poleDirection = math::Vec3{0.0, 1.0, 0.0}; // knees point forward
        leg.limbRadiusM = kLimbRadiusM * scale;
        h.mapping.ikChains.push_back(leg);
    }
    return h;
}

} // namespace opas::test
