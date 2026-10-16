#pragma once

// Descriptions used to create bodies and joints, and motor settings that drive
// joints. All plain data; validated by the backend at create time.

#include "opas/physics/Types.hpp"

#include <cstdint>
#include <limits>
#include <variant>

namespace opas::physics {

struct BoxShape {
    Vec3 halfExtentsM{0.5f, 0.5f, 0.5f};
};

struct SphereShape {
    float radiusM = 0.5f;
};

// Capsule along the body's local Z axis (Z up). Total length is
// 2 * (halfHeightM + radiusM).
struct CapsuleShape {
    float halfHeightM = 0.5f;
    float radiusM = 0.25f;
};

using Shape = std::variant<BoxShape, SphereShape, CapsuleShape>;

enum class MotionType : std::uint8_t {
    Static,    // never moves
    Dynamic,   // simulated: gravity, contacts, joints, impulses
    Kinematic, // moved only by setKinematicTarget; pushes dynamic bodies
};

// Defaults for friction, restitution and damping match Jolt's
// BodyCreationSettings defaults at the pinned version.
struct BodyDesc {
    Shape shape = BoxShape{};
    MotionType motion = MotionType::Dynamic;
    Vec3 positionM;
    Quat rotation;
    float massKg = 1.0f; // used for Dynamic bodies only; must be > 0
    float friction = 0.2f;
    float restitution = 0.0f;
    float linearDamping = 0.05f;
    float angularDamping = 0.05f;
};

// Swing-twist joint between two bodies, defined in world space at creation.
// The twist axis is the joint's main axis (e.g. along a bone); swing is
// limited by an elliptical cone around it whose half-angles are measured in
// the plane containing planeAxis (planeHalfConeRad) and in the plane normal to
// it (normalHalfConeRad). A hinge is a swing-twist joint with zero cones.
struct JointDesc {
    BodyId bodyA;
    BodyId bodyB;
    Vec3 anchorM;                     // world space
    Vec3 twistAxis{1.0f, 0.0f, 0.0f}; // world space, unit length
    Vec3 planeAxis{0.0f, 1.0f, 0.0f}; // world space, unit length, perpendicular to twistAxis
    float normalHalfConeRad = 0.0f;
    float planeHalfConeRad = 0.0f;
    float twistMinRad = 0.0f;
    float twistMaxRad = 0.0f;
};

// Angular spring that pulls a joint towards its target rotation.
// Stiffness and damping both zero means the motor is off.
struct JointMotor {
    float stiffnessNmPerRad = 0.0f;
    float dampingNmsPerRad = 0.0f;
    float maxTorqueNm = std::numeric_limits<float>::max();
};

} // namespace opas::physics
