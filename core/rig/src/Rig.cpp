#include "opas/rig/Rig.hpp"

#include "Problem.hpp"

#include <cmath>
#include <numbers>
#include <set>

namespace opas::rig {
namespace {

using detail::Problem;

// How far an authored rotation or axis may be from unit length, and two axes
// from perpendicular. Text round-off stays far below it; real mistakes do not.
constexpr double kUnitTolerance = 1e-6;

bool finite(math::Vec3 v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Written as !(... <= tol) so NaN fails too.
bool unitQuat(const math::Quat& q)
{
    return std::abs(math::length(q) - 1.0) <= kUnitTolerance;
}

bool unitVec(math::Vec3 v)
{
    return finite(v) && std::abs(math::length(v) - 1.0) <= kUnitTolerance;
}

bool positive(double v)
{
    return std::isfinite(v) && v > 0.0;
}

bool inRange(double v, double lo, double hi)
{
    return v >= lo && v <= hi; // false for NaN
}

std::string at(std::string_view array, std::size_t index, std::string_view field = {})
{
    std::string pointer = "/" + std::string(array) + "/" + std::to_string(index);
    if (!field.empty()) {
        pointer += "/" + std::string(field);
    }
    return pointer;
}

std::string inQuotes(std::string_view name)
{
    return "\"" + std::string(name) + "\"";
}

std::optional<Problem> findBoneProblem(const Skeleton& skeleton)
{
    std::set<std::string> seen;
    for (std::size_t i = 0; i < skeleton.bones.size(); ++i) {
        const Bone& bone = skeleton.bones[i];
        if (bone.name.empty()) {
            return Problem{at("bones", i, "name"), "must not be empty"};
        }
        if (!seen.insert(bone.name).second) {
            return Problem{at("bones", i, "name"), "duplicate bone " + inQuotes(bone.name)};
        }
        if (bone.parent < -1 || (bone.parent >= 0 && static_cast<std::size_t>(bone.parent) >= i)) {
            return Problem{at("bones", i, "parent"), "must name a bone listed before this one"};
        }
        if (!finite(bone.localRest.translation)) {
            return Problem{at("bones", i, "translationM"), "must be finite"};
        }
        if (!unitQuat(bone.localRest.rotation)) {
            return Problem{at("bones", i, "rotation"), "must be a unit quaternion"};
        }
    }
    return std::nullopt;
}

std::optional<Problem> findShapeProblem(const Shape& shape, const std::string& pointer)
{
    if (const auto* capsule = std::get_if<CapsuleShape>(&shape)) {
        if (!positive(capsule->halfHeightM)) {
            return Problem{pointer + "/halfHeightM", "must be > 0"};
        }
        if (!positive(capsule->radiusM)) {
            return Problem{pointer + "/radiusM", "must be > 0"};
        }
    } else if (const auto* box = std::get_if<BoxShape>(&shape)) {
        const math::Vec3 e = box->halfExtentsM;
        if (!positive(e.x) || !positive(e.y) || !positive(e.z)) {
            return Problem{pointer + "/halfExtentsM", "each half extent must be > 0"};
        }
    } else if (const auto* sphere = std::get_if<SphereShape>(&shape);
               sphere != nullptr && !positive(sphere->radiusM)) {
        return Problem{pointer + "/radiusM", "must be > 0"};
    }
    return std::nullopt;
}

std::optional<Problem> findBodyProblem(const Rig& rig)
{
    std::set<std::string> seen;
    for (std::size_t i = 0; i < rig.bodies.size(); ++i) {
        const Body& body = rig.bodies[i];
        if (!findBone(rig.skeleton, body.bone)) {
            return Problem{at("bodies", i, "bone"), "unknown bone " + inQuotes(body.bone)};
        }
        if (!seen.insert(body.bone).second) {
            return Problem{at("bodies", i, "bone"),
                           "bone " + inQuotes(body.bone) + " already has a body"};
        }
        if (auto problem = findShapeProblem(body.shape, at("bodies", i, "shape"))) {
            return problem;
        }
        if (!finite(body.local.translation)) {
            return Problem{at("bodies", i, "translationM"), "must be finite"};
        }
        if (!unitQuat(body.local.rotation)) {
            return Problem{at("bodies", i, "rotation"), "must be a unit quaternion"};
        }
        if (!positive(body.massKg)) {
            return Problem{at("bodies", i, "massKg"), "must be > 0"};
        }
    }
    return std::nullopt;
}

std::optional<Problem> findJointProblem(const Rig& rig)
{
    const double pi = std::numbers::pi;
    std::set<std::string> seen;
    for (std::size_t j = 0; j < rig.joints.size(); ++j) {
        const Joint& joint = rig.joints[j];
        if (!findBone(rig.skeleton, joint.bone)) {
            return Problem{at("joints", j, "bone"), "unknown bone " + inQuotes(joint.bone)};
        }
        if (!seen.insert(joint.bone).second) {
            return Problem{at("joints", j, "bone"),
                           "bone " + inQuotes(joint.bone) + " already has a joint"};
        }
        const std::optional<std::size_t> body = findBody(rig, joint.bone);
        if (!body) {
            return Problem{at("joints", j, "bone"), "bone " + inQuotes(joint.bone) +
                                                        " has no body (a joint connects bodies)"};
        }
        if (!parentBody(rig, *body)) {
            return Problem{at("joints", j, "bone"),
                           "bone " + inQuotes(joint.bone) +
                               " has the root body, which has no parent body to join"};
        }
        if (!unitVec(joint.twistAxis)) {
            return Problem{at("joints", j, "twistAxis"), "must be a unit vector"};
        }
        if (!unitVec(joint.planeAxis)) {
            return Problem{at("joints", j, "planeAxis"), "must be a unit vector"};
        }
        if (!(std::abs(math::dot(joint.twistAxis, joint.planeAxis)) <= kUnitTolerance)) {
            return Problem{at("joints", j, "planeAxis"), "must be perpendicular to twistAxis"};
        }
        if (!inRange(joint.normalHalfConeRad, 0.0, pi)) {
            return Problem{at("joints", j, "normalHalfConeRad"), "must be in [0, pi]"};
        }
        if (!inRange(joint.planeHalfConeRad, 0.0, pi)) {
            return Problem{at("joints", j, "planeHalfConeRad"), "must be in [0, pi]"};
        }
        if (!inRange(joint.twistMinRad, -pi, pi)) {
            return Problem{at("joints", j, "twistMinRad"), "must be in [-pi, pi]"};
        }
        if (!inRange(joint.twistMaxRad, -pi, pi)) {
            return Problem{at("joints", j, "twistMaxRad"), "must be in [-pi, pi]"};
        }
        if (joint.twistMinRad > joint.twistMaxRad) {
            return Problem{at("joints", j, "twistMinRad"), "must be <= twistMaxRad"};
        }
    }
    return std::nullopt;
}

// One root body; every other body has a joint.
std::optional<Problem> findStructureProblem(const Rig& rig)
{
    std::optional<std::size_t> root;
    for (std::size_t i = 0; i < rig.bodies.size(); ++i) {
        if (parentBody(rig, i)) {
            continue;
        }
        if (root) {
            return Problem{at("bodies", i, "bone"),
                           "no ancestor of " + inQuotes(rig.bodies[i].bone) +
                               " has a body, so it would be a second root body (the root is on " +
                               inQuotes(rig.bodies[*root].bone) + ")"};
        }
        root = i;
    }
    for (std::size_t i = 0; i < rig.bodies.size(); ++i) {
        if (i != root && !findJoint(rig, rig.bodies[i].bone)) {
            return Problem{at("bodies", i), "the body on " + inQuotes(rig.bodies[i].bone) +
                                                " needs a joint (only the root body has none)"};
        }
    }
    return std::nullopt;
}

} // namespace

namespace detail {

std::optional<Problem> findRigProblem(const Rig& rig)
{
    if (rig.name.empty()) {
        return Problem{"/name", "must not be empty"};
    }
    if (rig.skeleton.bones.empty()) {
        return Problem{"/bones", "needs at least one bone"};
    }
    if (auto problem = findBoneProblem(rig.skeleton)) {
        return problem;
    }
    if (rig.bodies.empty()) {
        return Problem{"/bodies", "needs at least one body"};
    }
    if (auto problem = findBodyProblem(rig)) {
        return problem;
    }
    if (auto problem = findJointProblem(rig)) {
        return problem;
    }
    return findStructureProblem(rig);
}

} // namespace detail

std::string validateRig(const Rig& rig)
{
    return detail::toString(detail::findRigProblem(rig));
}

std::optional<std::size_t> findBody(const Rig& rig, std::string_view bone)
{
    for (std::size_t i = 0; i < rig.bodies.size(); ++i) {
        if (rig.bodies[i].bone == bone) {
            return i;
        }
    }
    return std::nullopt;
}

std::optional<std::size_t> findJoint(const Rig& rig, std::string_view bone)
{
    for (std::size_t i = 0; i < rig.joints.size(); ++i) {
        if (rig.joints[i].bone == bone) {
            return i;
        }
    }
    return std::nullopt;
}

std::optional<std::size_t> parentBody(const Rig& rig, std::size_t bodyIndex)
{
    const std::optional<std::size_t> bone = findBone(rig.skeleton, rig.bodies[bodyIndex].bone);
    if (!bone) {
        return std::nullopt;
    }
    // Parents come before children, so this walk ends.
    int ancestor = rig.skeleton.bones[*bone].parent;
    while (ancestor >= 0) {
        const auto index = static_cast<std::size_t>(ancestor);
        if (const auto body = findBody(rig, rig.skeleton.bones[index].name)) {
            return body;
        }
        ancestor = rig.skeleton.bones[index].parent;
    }
    return std::nullopt;
}

} // namespace opas::rig
