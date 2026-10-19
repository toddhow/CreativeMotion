#pragma once

// The profile: how a rig's body behaves (motor gains, surface materials).
// One rig can have several profiles. File format: docs/formats/profile.md.
//
// Field meanings match opas::physics::JointMotor and BodyDesc; M3 applies them.

#include "opas/rig/Rig.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace opas::rig {

// Angular spring towards a joint's target: torque = stiffness * angle error -
// damping * relative angular velocity, clamped to maxTorqueNm.
struct MotorGains {
    double stiffnessNmPerRad = 0.0;
    double dampingNmsPerRad = 0.0;
    double maxTorqueNm = 0.0;
    friend bool operator==(const MotorGains&, const MotorGains&) = default;
};

struct Material {
    double friction = 0.0;
    double restitution = 0.0;
    double linearDamping = 0.0;
    double angularDamping = 0.0;
    friend bool operator==(const Material&, const Material&) = default;
};

struct JointOverride {
    std::string bone; // a bone with a joint in the rig
    MotorGains motor;
    friend bool operator==(const JointOverride&, const JointOverride&) = default;
};

struct BodyOverride {
    std::string bone; // a bone with a body in the rig
    Material material;
    friend bool operator==(const BodyOverride&, const BodyOverride&) = default;
};

// Defaults cover every joint and body; an override replaces the whole motor
// or material of one bone (no field-by-field merging).
struct Profile {
    std::string rig; // the rig name this profile was tuned for
    MotorGains defaultMotor;
    Material defaultMaterial;
    std::vector<JointOverride> joints;
    std::vector<BodyOverride> bodies;
    friend bool operator==(const Profile&, const Profile&) = default;
};

// Empty string if the profile is valid on its own and for `rig` (every rule
// in docs/formats/profile.md); otherwise "<JSON pointer>: <message>". The
// loader checks only the rules that need no rig, so call this once the rig
// is known.
std::string validateProfile(const Profile& profile, const Rig& rig);

// The gains and material that apply to a bone: its override, else the default.
MotorGains motorFor(const Profile& profile, std::string_view bone);
Material materialFor(const Profile& profile, std::string_view bone);

} // namespace opas::rig
