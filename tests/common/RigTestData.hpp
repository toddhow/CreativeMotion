#pragma once

// Synthetic rig and profile texts for the core_rig tests, in canonical form
// (writeRig/writeProfile output), so they serve both the loader tests (line
// numbers below) and the round-trip tests.
//
// The rig is a four-bone chain Root -> Mid -> Tip -> Ball with bodies on
// Root (box), Mid (capsule) and Ball (sphere). Tip has no body, so Ball's
// joint connects it to Mid. Line numbers, for the error tests:
//   bones 6-9 (Root 6, Mid 7, Tip 8, Ball 9); bodies: Root 12-18, Mid 19-25,
//   Ball 26-32; joints: Mid 35-43, Ball 44-52.

#include <string_view>

namespace opas::test {

inline constexpr std::string_view kRigText = R"({
  "format": "opas.rig",
  "version": 1,
  "name": "test-rig",
  "bones": [
    {"name": "Root", "parent": "", "translationM": [0, 0, 1], "rotation": [0, 0, 0, 1]},
    {"name": "Mid", "parent": "Root", "translationM": [0, 0, -0.3], "rotation": [0, 0, 0, 1]},
    {"name": "Tip", "parent": "Mid", "translationM": [0, 0, -0.25], "rotation": [0, 0, 0, 1]},
    {"name": "Ball", "parent": "Tip", "translationM": [0, 0, -0.1], "rotation": [0, 0, 0, 1]}
  ],
  "bodies": [
    {
      "bone": "Root",
      "shape": {"type": "box", "halfExtentsM": [0.1, 0.1, 0.1]},
      "translationM": [0, 0, 0],
      "rotation": [0, 0, 0, 1],
      "massKg": 5
    },
    {
      "bone": "Mid",
      "shape": {"type": "capsule", "halfHeightM": 0.1, "radiusM": 0.04},
      "translationM": [0, 0, -0.14],
      "rotation": [0, 0, 0, 1],
      "massKg": 1.5
    },
    {
      "bone": "Ball",
      "shape": {"type": "sphere", "radiusM": 0.05},
      "translationM": [0, 0, 0],
      "rotation": [0, 0, 0, 1],
      "massKg": 0.5
    }
  ],
  "joints": [
    {
      "bone": "Mid",
      "twistAxis": [1, 0, 0],
      "planeAxis": [0, 1, 0],
      "normalHalfConeRad": 0,
      "planeHalfConeRad": 0,
      "twistMinRad": 0,
      "twistMaxRad": 2.6
    },
    {
      "bone": "Ball",
      "twistAxis": [0, 0, -1],
      "planeAxis": [1, 0, 0],
      "normalHalfConeRad": 0.5,
      "planeHalfConeRad": 0.25,
      "twistMinRad": -0.3,
      "twistMaxRad": 0.3
    }
  ]
}
)";

// A profile for kRigText. Lines: defaults motor 6, material 7; joint
// override 10; body override 13.
inline constexpr std::string_view kProfileText = R"({
  "format": "opas.profile",
  "version": 1,
  "rig": "test-rig",
  "defaults": {
    "motor": {"stiffnessNmPerRad": 300, "dampingNmsPerRad": 30, "maxTorqueNm": 500},
    "material": {"friction": 0.8, "restitution": 0, "linearDamping": 0.05, "angularDamping": 0.05}
  },
  "joints": [
    {"bone": "Mid", "motor": {"stiffnessNmPerRad": 600, "dampingNmsPerRad": 60, "maxTorqueNm": 900}}
  ],
  "bodies": [
    {"bone": "Ball", "material": {"friction": 1, "restitution": 0.1, "linearDamping": 0.2, "angularDamping": 0.3}}
  ]
}
)";

} // namespace opas::test
