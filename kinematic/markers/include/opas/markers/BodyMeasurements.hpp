#pragma once

// Body measurements from a skeleton's rest pose: limb lengths and simple
// surface capsules. The alignment solver uses them to compare two actors of
// different sizes and to keep limbs from passing through bodies.
//
// Which bones form which limb is data (a BodyMapping), because skeletons name
// their bones differently.

#include "opas/math/Vec3.hpp"
#include "opas/rig/Skeleton.hpp"

#include <optional>
#include <string>
#include <vector>

namespace opas::markers {

// A limb is a chain of bones from proximal to distal, e.g.
// {"RightUpperArm", "RightForearm", "RightHand"}; its length is the sum of the
// distances between consecutive bones' world rest positions. Needs >= 2 bones.
struct LimbDefinition {
    std::string name;
    std::vector<std::string> bones;
};

// A capsule from `bone` to `endBone` (world rest positions), named as a body
// region, with radius = radiusRatio * its length. radiusRatio must be > 0.
struct CapsuleRule {
    std::string region; // e.g. "RightForearm"; what InteractionMarker::targetRegion names
    std::string bone;
    std::string endBone;
    double radiusRatio = 0.15;
};

struct BodyMapping {
    std::vector<LimbDefinition> limbs;
    std::vector<CapsuleRule> capsules;
};

struct LimbLength {
    std::string name;
    double lengthM = 0.0;
};

struct SurfaceCapsule {
    std::string region;
    math::Vec3 startM; // model space, rest pose
    math::Vec3 endM;
    double radiusM = 0.0;
};

struct BodyMeasurements {
    std::vector<LimbLength> limbs;        // same order as BodyMapping::limbs
    std::vector<SurfaceCapsule> capsules; // same order as BodyMapping::capsules
    double heightM = 0.0;                 // highest minus lowest bone world rest z
};

struct MeasureResult {
    std::optional<BodyMeasurements> value;
    std::string error; // names the bad limb, capsule or bone when value is empty
};

// The skeleton must pass rig::validateSkeleton (an invalid one is rejected).
MeasureResult measureBody(const rig::Skeleton& skeleton, const BodyMapping& mapping);

} // namespace opas::markers
