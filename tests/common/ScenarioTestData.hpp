#pragma once

// A synthetic scenario text for the loader tests, in canonical form. Lines:
// statics 9; ragdoll object 12-21 (name 13, rootMotion 18, motors 19, pose 20);
// impulse 24.

#include <string_view>

namespace opas::test {

inline constexpr std::string_view kScenarioText = R"({
  "format": "opas.scenario",
  "version": 1,
  "name": "push",
  "stepsPerSecond": 60,
  "durationS": 3,
  "gravityMps2": [0, 0, -9.81],
  "statics": [
    {"name": "floor", "halfExtentsM": [20, 20, 0.5], "translationM": [0, 0, -0.5], "rotation": [0, 0, 0, 1], "friction": 0.8}
  ],
  "ragdolls": [
    {
      "name": "hero",
      "rig": "../../data/rigs/humanoid.rig.json",
      "profile": "../../data/profiles/humanoid-default.profile.json",
      "translationM": [0, 0, 0.02],
      "rotation": [0, 0, 0, 1],
      "rootMotion": "dynamic",
      "motors": "profile",
      "pose": "rest"
    }
  ],
  "impulses": [
    {"atS": 0.5, "ragdoll": "hero", "bone": "Chest", "impulseNs": [0, 60, 0], "offsetM": [0, 0, 0]}
  ]
}
)";

} // namespace opas::test
