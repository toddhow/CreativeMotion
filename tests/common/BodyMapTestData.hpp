#pragma once

// Synthetic body maps and the canonical text that describes them (docs/formats/bodymap.md),
// shared by the body-map load and round-trip tests. The loader tests use hard-coded line
// numbers, so keep this layout stable.

#include "opas/ik/TwoBoneIk.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace opas::test {

// Lines: 1 "{", 2 format, 3 version, 4 "limbs": [, 5-6 limbs, 7 "],", 8 "capsules": [,
// 9-10 capsules, 11 "],", 12 "ikChains": [, then chain 0: 13 "{", 14 name, 15 rootBone,
// 16 midBone, 17 endBone, 18 minBendRad, 19 maxBendRad, 20 poleDirection, 21 limbRadiusM,
// 22 "},"; chain 1: 23 "{", 24 name, 25 rootBone, 26 midBone, 27 endBone, 28 minBendRad,
// 29 maxBendRad, 30 poleDirection, 31 limbRadiusM, 32 "}"; 33 "]", 34 "}".
inline constexpr std::string_view kBodyMapText = R"json({
  "format": "opas.bodymap",
  "version": 1,
  "limbs": [
    {"name": "RightArm", "bones": ["RightUpperArm", "RightForearm", "RightHand"]},
    {"name": "LeftArm", "bones": ["LeftUpperArm", "LeftForearm", "LeftHand"]}
  ],
  "capsules": [
    {"region": "RightForearm", "bone": "RightForearm", "endBone": "RightHand", "radiusRatio": 0.15},
    {"region": "UpperBack", "bone": "Spine", "endBone": "Neck", "radiusRatio": 0.1667}
  ],
  "ikChains": [
    {
      "name": "RightArm",
      "rootBone": "RightUpperArm",
      "midBone": "RightForearm",
      "endBone": "RightHand",
      "minBendRad": 0,
      "maxBendRad": 2.6,
      "poleDirection": [0, -1, 0],
      "limbRadiusM": 0.04
    },
    {
      "name": "LeftArm",
      "rootBone": "LeftUpperArm",
      "midBone": "LeftForearm",
      "endBone": "LeftHand",
      "minBendRad": 0.1,
      "maxBendRad": 2.5,
      "poleDirection": [-0.35, -0.2, -0.9],
      "limbRadiusM": 0.03
    }
  ]
}
)json";

inline constexpr std::string_view kEmptyBodyMapText = R"json({
  "format": "opas.bodymap",
  "version": 1,
  "limbs": [],
  "capsules": [],
  "ikChains": []
}
)json";

// The mapping kBodyMapText describes.
inline markers::BodyMapping bodyMapOfText()
{
    markers::BodyMapping m;
    m.limbs = {
        {"RightArm", {"RightUpperArm", "RightForearm", "RightHand"}},
        {"LeftArm", {"LeftUpperArm", "LeftForearm", "LeftHand"}},
    };
    m.capsules = {
        {"RightForearm", "RightForearm", "RightHand", 0.15},
        {"UpperBack", "Spine", "Neck", 0.1667},
    };
    markers::IkChain right;
    right.name = "RightArm";
    right.rootBone = "RightUpperArm";
    right.midBone = "RightForearm";
    right.endBone = "RightHand";
    right.limits = ik::HingeLimits{0.0, 2.6};
    right.poleDirection = math::Vec3{0.0, -1.0, 0.0};
    right.limbRadiusM = 0.04;
    markers::IkChain left;
    left.name = "LeftArm";
    left.rootBone = "LeftUpperArm";
    left.midBone = "LeftForearm";
    left.endBone = "LeftHand";
    left.limits = ik::HingeLimits{0.1, 2.5};
    left.poleDirection = math::Vec3{-0.35, -0.2, -0.9};
    left.limbRadiusM = 0.03;
    m.ikChains = {right, left};
    return m;
}

inline bool sameBits(double a, double b)
{
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

// BodyMapping has no operator==, so compare field by field; numbers must match bit for bit.
inline ::testing::AssertionResult sameMapping(const markers::BodyMapping& a,
                                              const markers::BodyMapping& b)
{
    if (a.limbs.size() != b.limbs.size() || a.capsules.size() != b.capsules.size() ||
        a.ikChains.size() != b.ikChains.size()) {
        return ::testing::AssertionFailure() << "different numbers of limbs, capsules or chains";
    }
    for (std::size_t i = 0; i < a.limbs.size(); ++i) {
        if (a.limbs[i].name != b.limbs[i].name || a.limbs[i].bones != b.limbs[i].bones) {
            return ::testing::AssertionFailure() << "limb " << i << " differs";
        }
    }
    for (std::size_t i = 0; i < a.capsules.size(); ++i) {
        const markers::CapsuleRule& x = a.capsules[i];
        const markers::CapsuleRule& y = b.capsules[i];
        if (x.region != y.region || x.bone != y.bone || x.endBone != y.endBone ||
            !sameBits(x.radiusRatio, y.radiusRatio)) {
            return ::testing::AssertionFailure() << "capsule " << i << " differs";
        }
    }
    for (std::size_t i = 0; i < a.ikChains.size(); ++i) {
        const markers::IkChain& x = a.ikChains[i];
        const markers::IkChain& y = b.ikChains[i];
        if (x.name != y.name || x.rootBone != y.rootBone || x.midBone != y.midBone ||
            x.endBone != y.endBone || !sameBits(x.limits.minBendRad, y.limits.minBendRad) ||
            !sameBits(x.limits.maxBendRad, y.limits.maxBendRad) ||
            !sameBits(x.poleDirection.x, y.poleDirection.x) ||
            !sameBits(x.poleDirection.y, y.poleDirection.y) ||
            !sameBits(x.poleDirection.z, y.poleDirection.z) ||
            !sameBits(x.limbRadiusM, y.limbRadiusM)) {
            return ::testing::AssertionFailure() << "chain " << i << " differs";
        }
    }
    return ::testing::AssertionSuccess();
}

} // namespace opas::test
