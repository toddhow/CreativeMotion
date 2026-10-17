#pragma once

// Synthetic marker files and the MarkerSets they describe, shared by the marker
// load tests and the round-trip tests. The texts are in canonical form (see
// docs/formats/markers.md), so writeMarkerSet must reproduce them exactly.
//
// The line numbers in the loader tests depend on this layout; keep it stable.

#include "opas/markers/InteractionMarker.hpp"

#include <string>
#include <string_view>

namespace opas::test {

// Lines: 1 "{", 2 format, 3 version, 4 "markers": [, 5 marker "{", 6 id, 7 sourceBone,
// 8 targetRegion, 9 offsetM, 10 contactDirection, 11 window, 12 priority, 13 toleranceM,
// 14 marker "}", 15 "]", 16 "}".
inline constexpr std::string_view kOneMarkerText = R"json({
  "format": "opas.markers",
  "version": 1,
  "markers": [
    {
      "id": "rightHandOnShoulder",
      "sourceBone": "RightHand",
      "targetRegion": "LeftUpperArm",
      "offsetM": [0.02, -0.01, 0.12],
      "contactDirection": [0, 0, -1],
      "window": {"startS": 1.25, "endS": 2},
      "priority": 1,
      "toleranceM": 0.02
    }
  ]
}
)json";

// Lines 1-13 as above, 14 "},", 15 second marker "{", 16 id, 17 sourceBone, 18 targetRegion,
// 19 offsetM, 20 contactDirection, 21 window, 22 priority, 23 toleranceM, 24 "}", 25 "]", 26 "}".
inline constexpr std::string_view kTwoMarkerText = R"json({
  "format": "opas.markers",
  "version": 1,
  "markers": [
    {
      "id": "rightHandOnShoulder",
      "sourceBone": "RightHand",
      "targetRegion": "LeftUpperArm",
      "offsetM": [0.02, -0.01, 0.12],
      "contactDirection": [0, 0, -1],
      "window": {"startS": 1.25, "endS": 2},
      "priority": 1,
      "toleranceM": 0.02
    },
    {
      "id": "leftFootOnFloor",
      "sourceBone": "LeftFoot",
      "targetRegion": "Floor",
      "offsetM": [0, 0, 0],
      "contactDirection": [1, 0, 0],
      "window": {"startS": 0, "endS": 0.5},
      "priority": -3,
      "toleranceM": 0.005
    }
  ]
}
)json";

inline constexpr std::string_view kEmptyMarkersText = R"json({
  "format": "opas.markers",
  "version": 1,
  "markers": []
}
)json";

inline markers::InteractionMarker rightHandOnShoulder()
{
    markers::InteractionMarker m;
    m.id = "rightHandOnShoulder";
    m.sourceBone = "RightHand";
    m.targetRegion = "LeftUpperArm";
    m.offsetM = math::Vec3{0.02, -0.01, 0.12};
    m.contactDirection = math::Vec3{0.0, 0.0, -1.0};
    m.window = markers::TimeWindow{1.25, 2.0};
    m.priority = 1;
    m.toleranceM = 0.02;
    return m;
}

inline markers::InteractionMarker leftFootOnFloor()
{
    markers::InteractionMarker m;
    m.id = "leftFootOnFloor";
    m.sourceBone = "LeftFoot";
    m.targetRegion = "Floor";
    m.offsetM = math::Vec3{0.0, 0.0, 0.0};
    m.contactDirection = math::Vec3{1.0, 0.0, 0.0};
    m.window = markers::TimeWindow{0.0, 0.5};
    m.priority = -3;
    m.toleranceM = 0.005;
    return m;
}

inline markers::MarkerSet oneMarkerSet()
{
    markers::MarkerSet set;
    set.markers.push_back(rightHandOnShoulder());
    return set;
}

inline markers::MarkerSet twoMarkerSet()
{
    markers::MarkerSet set;
    set.markers.push_back(rightHandOnShoulder());
    set.markers.push_back(leftFootOnFloor());
    return set;
}

} // namespace opas::test
