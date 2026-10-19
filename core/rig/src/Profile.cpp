#include "opas/rig/Profile.hpp"

#include "Problem.hpp"

#include <cmath>
#include <set>

namespace opas::rig {
namespace {

using detail::Problem;

bool nonNegative(double v)
{
    return std::isfinite(v) && v >= 0.0;
}

std::string inQuotes(std::string_view name)
{
    return "\"" + std::string(name) + "\"";
}

std::optional<Problem> findMotorProblem(const MotorGains& motor, const std::string& pointer)
{
    if (!nonNegative(motor.stiffnessNmPerRad)) {
        return Problem{pointer + "/stiffnessNmPerRad", "must be >= 0"};
    }
    if (!nonNegative(motor.dampingNmsPerRad)) {
        return Problem{pointer + "/dampingNmsPerRad", "must be >= 0"};
    }
    if (!(std::isfinite(motor.maxTorqueNm) && motor.maxTorqueNm > 0.0)) {
        return Problem{pointer + "/maxTorqueNm", "must be > 0"};
    }
    return std::nullopt;
}

std::optional<Problem> findMaterialProblem(const Material& material, const std::string& pointer)
{
    if (!nonNegative(material.friction)) {
        return Problem{pointer + "/friction", "must be >= 0"};
    }
    if (!nonNegative(material.restitution)) {
        return Problem{pointer + "/restitution", "must be >= 0"};
    }
    if (!nonNegative(material.linearDamping)) {
        return Problem{pointer + "/linearDamping", "must be >= 0"};
    }
    if (!nonNegative(material.angularDamping)) {
        return Problem{pointer + "/angularDamping", "must be >= 0"};
    }
    return std::nullopt;
}

// Checks each override: a non-empty bone, used once, with valid values.
template <typename Override, typename CheckValue>
std::optional<Problem> findOverrideProblem(const std::vector<Override>& overrides,
                                           std::string_view array, std::string_view valueKey,
                                           CheckValue checkValue)
{
    std::set<std::string> seen;
    for (std::size_t i = 0; i < overrides.size(); ++i) {
        const std::string pointer = "/" + std::string(array) + "/" + std::to_string(i);
        const std::string& bone = overrides[i].bone;
        if (bone.empty()) {
            return Problem{pointer + "/bone", "must not be empty"};
        }
        if (!detail::isValidName(bone)) {
            return Problem{pointer + "/bone", detail::kInvalidNameMessage};
        }
        if (!seen.insert(bone).second) {
            return Problem{pointer + "/bone", "duplicate override for bone " + inQuotes(bone)};
        }
        if (auto problem = checkValue(overrides[i], pointer + "/" + std::string(valueKey))) {
            return problem;
        }
    }
    return std::nullopt;
}

} // namespace

namespace detail {

std::optional<Problem> findProfileProblem(const Profile& profile)
{
    if (profile.rig.empty()) {
        return Problem{"/rig", "must not be empty"};
    }
    if (!isValidName(profile.rig)) {
        return Problem{"/rig", kInvalidNameMessage};
    }
    if (auto problem = findMotorProblem(profile.defaultMotor, "/defaults/motor")) {
        return problem;
    }
    if (auto problem = findMaterialProblem(profile.defaultMaterial, "/defaults/material")) {
        return problem;
    }
    if (auto problem = findOverrideProblem(profile.joints, "joints", "motor",
                                           [](const JointOverride& o, const std::string& p) {
                                               return findMotorProblem(o.motor, p);
                                           })) {
        return problem;
    }
    return findOverrideProblem(profile.bodies, "bodies", "material",
                               [](const BodyOverride& o, const std::string& p) {
                                   return findMaterialProblem(o.material, p);
                               });
}

std::optional<Problem> findProfileRigProblem(const Profile& profile, const Rig& rig)
{
    if (profile.rig != rig.name) {
        return Problem{"/rig", "the profile is for rig " + inQuotes(profile.rig) + ", not " +
                                   inQuotes(rig.name)};
    }
    for (std::size_t i = 0; i < profile.joints.size(); ++i) {
        if (!findJoint(rig, profile.joints[i].bone)) {
            return Problem{"/joints/" + std::to_string(i) + "/bone",
                           "rig " + inQuotes(rig.name) + " has no joint on bone " +
                               inQuotes(profile.joints[i].bone)};
        }
    }
    for (std::size_t i = 0; i < profile.bodies.size(); ++i) {
        if (!findBody(rig, profile.bodies[i].bone)) {
            return Problem{"/bodies/" + std::to_string(i) + "/bone",
                           "rig " + inQuotes(rig.name) + " has no body on bone " +
                               inQuotes(profile.bodies[i].bone)};
        }
    }
    return std::nullopt;
}

} // namespace detail

std::string validateProfile(const Profile& profile, const Rig& rig)
{
    if (const auto problem = detail::findProfileProblem(profile)) {
        return detail::toString(problem);
    }
    return detail::toString(detail::findProfileRigProblem(profile, rig));
}

MotorGains motorFor(const Profile& profile, std::string_view bone)
{
    for (const JointOverride& o : profile.joints) {
        if (o.bone == bone) {
            return o.motor;
        }
    }
    return profile.defaultMotor;
}

Material materialFor(const Profile& profile, std::string_view bone)
{
    for (const BodyOverride& o : profile.bodies) {
        if (o.bone == bone) {
            return o.material;
        }
    }
    return profile.defaultMaterial;
}

} // namespace opas::rig
