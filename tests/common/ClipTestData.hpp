#pragma once

// Canonical pose clip and detection profile texts (docs/formats/poseclip.md and
// docs/formats/detection.md) and the values they describe, shared by the load and round-trip
// tests. The loader tests use hard-coded line numbers, so keep these layouts stable.

#include "opas/markers/ContactDetection.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"

#include <string_view>

namespace opas::test {

// Lines: 1 "{", 2 format, 3 version, 4 sampleRateHz, 5 "actors": [, then actor 0: 6 "{",
// 7 bodyMap, 8 bones, 9 "frames": [, 10-11 frames, 12 "],", 13 "},"; actor 1: 14 "{",
// 15 bodyMap, 16 bones, 17 "frames": [, 18-19 frames, 20 "]", 21 "}", 22 "]", 23 "}".
inline constexpr std::string_view kPoseClipText = R"json({
  "format": "opas.poseclip",
  "version": 1,
  "sampleRateHz": 30,
  "actors": [
    {
      "bodyMap": "humanoid",
      "bones": ["Pelvis", "RightHand"],
      "frames": [
        [[0, 0, 1, 0, 0, 0, 1], [0.3, -0.2, 1.1, 0, 0, 0, 1]],
        [[0, 0, 1, 0, 0, 0, 1], [0.31, -0.2, 1.1, 0, 0, 0, 1]]
      ]
    },
    {
      "bodyMap": "humanoid",
      "bones": ["Pelvis"],
      "frames": [
        [[0.6, 0, 1, 0, 0, 1, 0]],
        [[0.6, 0, 1, 0, 0, 1, 0]]
      ]
    }
  ]
}
)json";

inline markers::PoseClip poseClipOfText()
{
    using math::Quat;
    using math::Transform;
    using math::Vec3;
    markers::PoseClip clip;
    clip.sampleRateHz = 30.0;
    clip.actors[0].bodyMap = "humanoid";
    clip.actors[0].bones = {"Pelvis", "RightHand"};
    clip.actors[0].frames = {
        {Transform{Vec3{0.0, 0.0, 1.0}, Quat{}}, Transform{Vec3{0.3, -0.2, 1.1}, Quat{}}},
        {Transform{Vec3{0.0, 0.0, 1.0}, Quat{}}, Transform{Vec3{0.31, -0.2, 1.1}, Quat{}}},
    };
    clip.actors[1].bodyMap = "humanoid";
    clip.actors[1].bones = {"Pelvis"};
    clip.actors[1].frames = {
        {Transform{Vec3{0.6, 0.0, 1.0}, Quat{0.0, 0.0, 1.0, 0.0}}},
        {Transform{Vec3{0.6, 0.0, 1.0}, Quat{0.0, 0.0, 1.0, 0.0}}},
    };
    return clip;
}

// Lines: 1 "{", 2 format, 3 version, 4 distanceThresholdM, 5 minDurationS,
// 6 maxRelativeSpeedMps, 7 markerToleranceM, 8 includeLowConfidence, 9 "}".
inline constexpr std::string_view kDetectionText = R"json({
  "format": "opas.detection",
  "version": 1,
  "distanceThresholdM": 0.03,
  "minDurationS": 0.25,
  "maxRelativeSpeedMps": 0.2,
  "markerToleranceM": 0.02,
  "includeLowConfidence": false
}
)json";

// A second profile with every value different from the default.
inline constexpr std::string_view kOtherDetectionText = R"json({
  "format": "opas.detection",
  "version": 1,
  "distanceThresholdM": 0.05,
  "minDurationS": 0.5,
  "maxRelativeSpeedMps": 0.1,
  "markerToleranceM": 0.01,
  "includeLowConfidence": true
}
)json";

inline markers::DetectionSettings otherDetectionSettings()
{
    markers::DetectionSettings s;
    s.distanceThresholdM = 0.05;
    s.minDurationS = 0.5;
    s.maxRelativeSpeedMps = 0.1;
    s.markerToleranceM = 0.01;
    s.includeLowConfidence = true;
    return s;
}

} // namespace opas::test
