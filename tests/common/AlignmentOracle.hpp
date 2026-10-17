#pragma once

// Independent checks for alignment results (A2). These recompute, from the scene and the
// solver's reported root shifts and bone rotations, where the hands ended up and how far limbs
// reach into the other actor's body capsules. They do not call the solver or any of its code,
// so a wrong residual or penetration in AlignmentResult is caught here.

#include "AlignmentScenes.hpp"
#include "SyntheticHumanoid.hpp"
#include "opas/markers/Alignment.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

namespace opas::test {

// A copy of the scene with the solver's root shifts added to the actor roots.
inline Scene shiftedScene(const Scene& scene, const markers::AlignmentResult& result)
{
    Scene shifted = scene;
    for (std::size_t i = 0; i < 2; ++i) {
        shifted.actors[i].root.translation =
            shifted.actors[i].root.translation + result.rootShiftM[i];
    }
    return shifted;
}

inline const markers::InteractionMarker* findMarker(const Scene& scene, const std::string& id)
{
    for (const markers::InteractionMarker& m : scene.markers.markers) {
        if (m.id == id) {
            return &m;
        }
    }
    return nullptr;
}

inline const markers::IkChain* findChain(const SceneActor& actor, const std::string& name)
{
    for (const markers::IkChain& c : actor.body.mapping.ikChains) {
        if (c.name == name) {
            return &c;
        }
    }
    return nullptr;
}

// The point a marker asks for, in the scene as given (use a shifted scene after alignment).
inline math::Vec3 markerTargetPoint(const Scene& scene, const markers::InteractionMarker& m)
{
    const SceneActor& target = scene.actors[static_cast<std::size_t>(m.targetActor)];
    return math::transformPoint(regionFrame(target, m.targetRegion), m.offsetM);
}

struct SolvedLimb {
    math::Vec3 root;
    math::Vec3 mid;
    math::Vec3 end;
};

// Joint positions after applying a limb correction to the posed (and shifted) actor: the
// upper bone vector is turned by rootBoneDelta, the lower one by midBoneDelta.
inline SolvedLimb solvedLimb(const Scene& shifted, const markers::LimbCorrection& limb)
{
    const SceneActor& actor = shifted.actors[static_cast<std::size_t>(limb.actor)];
    const markers::IkChain* chain = findChain(actor, limb.chain);
    SolvedLimb out;
    if (chain == nullptr) {
        return out;
    }
    const math::Vec3 root = bonePosition(actor, chain->rootBone);
    const math::Vec3 mid = bonePosition(actor, chain->midBone);
    const math::Vec3 end = bonePosition(actor, chain->endBone);
    out.root = root;
    out.mid = root + math::rotate(limb.rootBoneDelta, mid - root);
    out.end = out.mid + math::rotate(limb.midBoneDelta, end - mid);
    return out;
}

// Distance between two line segments (closest points).
inline double segmentDistance(math::Vec3 p1, math::Vec3 q1, math::Vec3 p2, math::Vec3 q2)
{
    const math::Vec3 d1 = q1 - p1;
    const math::Vec3 d2 = q2 - p2;
    const math::Vec3 r = p1 - p2;
    const double a = math::dot(d1, d1);
    const double e = math::dot(d2, d2);
    const double f = math::dot(d2, r);
    double s = 0.0;
    double t = 0.0;
    if (a <= 1e-18 && e <= 1e-18) {
        return math::length(r);
    }
    if (a <= 1e-18) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = math::dot(d1, r);
        if (e <= 1e-18) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = math::dot(d1, d2);
            const double denom = a * e - b * b;
            s = denom > 1e-18 ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return math::length((p1 + d1 * s) - (p2 + d2 * t));
}

// Summed penetration of the solved limb into the other actor's posed capsules, leaving out
// the capsule the driving marker touches: max(0, capsule radius + limb radius - distance) for
// each of the two bone segments and each capsule, as docs/tasks/0003 defines it.
inline double limbPenetrationM(const Scene& shifted, const markers::LimbCorrection& limb,
                               const std::string& touchedRegion)
{
    const SceneActor& owner = shifted.actors[static_cast<std::size_t>(limb.actor)];
    const SceneActor& other = shifted.actors[static_cast<std::size_t>(1 - limb.actor)];
    const markers::IkChain* chain = findChain(owner, limb.chain);
    if (chain == nullptr) {
        return -1.0;
    }
    const SolvedLimb solved = solvedLimb(shifted, limb);
    const std::vector<math::Transform> world = worldOf(other);
    double total = 0.0;
    for (const markers::CapsuleRule& c : other.body.mapping.capsules) {
        if (c.region == touchedRegion) {
            continue;
        }
        const math::Vec3 a = world[boneIndex(other, c.bone)].translation;
        const math::Vec3 b = world[boneIndex(other, c.endBone)].translation;
        const double radius = c.radiusRatio * math::length(b - a);
        const double upper = segmentDistance(solved.root, solved.mid, a, b);
        const double lower = segmentDistance(solved.mid, solved.end, a, b);
        total += std::max(0.0, radius + chain->limbRadiusM - upper);
        total += std::max(0.0, radius + chain->limbRadiusM - lower);
    }
    return total;
}

} // namespace opas::test
