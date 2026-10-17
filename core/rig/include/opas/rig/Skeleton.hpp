#pragma once

// Minimal skeleton: bone names, parents and rest pose. M2 extends this into
// the full rig format (bodies, masses, joint limits) with a file loader.

#include "opas/math/Transform.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace opas::rig {

struct Bone {
    std::string name;
    int parent = -1;           // index into Skeleton::bones; -1 for a root
    math::Transform localRest; // rest pose relative to the parent (or model root)
};

// Bones are ordered so every parent comes before its children.
struct Skeleton {
    std::vector<Bone> bones;
};

// Empty string if valid; otherwise a message naming the bone and field, e.g.
// "bone 3 (\"LeftForearm\"): parent must be < 3".
std::string validateSkeleton(const Skeleton& skeleton);

std::optional<std::size_t> findBone(const Skeleton& skeleton, std::string_view name);

// World (model-space) rest transform of every bone, same order as bones.
// The skeleton must be valid.
std::vector<math::Transform> computeWorldRest(const Skeleton& skeleton);

// World transform of every bone for a posed skeleton: localPose holds one
// transform per bone (same order, relative to the parent, like localRest) and
// root places the whole model in the world. The skeleton must be valid and
// localPose.size() == bones.size().
std::vector<math::Transform> computeWorldPose(const Skeleton& skeleton,
                                              std::span<const math::Transform> localPose,
                                              const math::Transform& root);

} // namespace opas::rig
