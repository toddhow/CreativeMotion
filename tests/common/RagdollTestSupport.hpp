#pragma once

// Shared helpers for the M3 ragdoll and motor-solver tests (docs/tasks/0013):
// the sample humanoid, worlds at a chosen step rate, a floor, the arm-wave
// target, and joint error and limit measurements.

#include "opas/math/Quat.hpp"
#include "opas/math/Transform.hpp"
#include "opas/motor/MotorSolver.hpp"
#include "opas/motor/Ragdoll.hpp"
#include "opas/physics/PhysicsWorld.hpp"
#include "opas/physics/jolt/JoltWorld.hpp"
#include "opas/rig/ProfileFile.hpp"
#include "opas/rig/RigFile.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <span>
#include <string_view>
#include <variant>
#include <vector>

namespace opas::test {

inline rig::Rig sampleRig()
{
    const auto result = rig::loadRigFile(OPAS_REPO_DATA_DIR "/rigs/humanoid.rig.json");
    EXPECT_TRUE(result.ok()) << result.error.toString();
    return result.value.value_or(rig::Rig{});
}

inline rig::Profile sampleProfile()
{
    const auto result =
        rig::loadProfileFile(OPAS_REPO_DATA_DIR "/profiles/humanoid-default.profile.json");
    EXPECT_TRUE(result.ok()) << result.error.toString();
    return result.value.value_or(rig::Profile{});
}

inline std::unique_ptr<physics::PhysicsWorld> makeWorldAtHz(int stepsPerSecond, bool gravity = true)
{
    physics::WorldConfig config;
    config.fixedStepSeconds = 1.0f / static_cast<float>(stepsPerSecond);
    if (!gravity) {
        config.gravityMps2 = physics::Vec3{};
    }
    return physics::jolt::createJoltWorld(config);
}

// Static floor whose top is z = 0.
inline physics::BodyId addFloor(physics::PhysicsWorld& world)
{
    physics::BodyDesc floor;
    floor.shape = physics::BoxShape{physics::Vec3{50.0f, 50.0f, 0.5f}};
    floor.motion = physics::MotionType::Static;
    floor.positionM = physics::Vec3{0.0f, 0.0f, -0.5f};
    floor.friction = 0.8f;
    return world.createBody(floor).id;
}

inline motor::Ragdoll buildOrFail(physics::PhysicsWorld& world, const rig::Rig& rig,
                                  const rig::Profile& profile, const math::Transform& rootWorld,
                                  motor::RootMotion rootMotion = motor::RootMotion::Dynamic)
{
    motor::BuildResult result =
        motor::buildRagdoll(world, rig, profile, rootWorld, motor::RagdollOptions{rootMotion});
    EXPECT_TRUE(result.ok()) << result.message;
    return result.ragdoll ? std::move(*result.ragdoll) : motor::Ragdoll{};
}

// Turns off every collision between the ragdoll's own bodies, so a test of
// joint targets is not disturbed by limbs touching the trunk or each other.
inline void disableSelfCollision(physics::PhysicsWorld& world, const motor::Ragdoll& ragdoll)
{
    const auto n = static_cast<std::uint32_t>(ragdoll.bodies.size());
    for (std::uint32_t a = 0; a < n; ++a) {
        for (std::uint32_t b = a + 1; b < n; ++b) {
            EXPECT_TRUE(world.setSubGroupsCollide(ragdoll.collisionGroup, a, b, false));
        }
    }
}

inline std::size_t boneIndex(const rig::Rig& rig, std::string_view name)
{
    const auto bone = rig::findBone(rig.skeleton, name);
    EXPECT_TRUE(bone.has_value()) << name;
    return bone.value_or(0);
}

// The spec's arm wave: the right shoulder swings 0.6 rad about its plane axis
// (model +X, forward and back) and the right elbow bends between 0.3 and
// 1.2 rad about +X, both at 0.5 Hz; every other bone holds its rest pose.
// The sample's rest rotations are all identity, so a local rotation about +X
// is a rotation about the joint's axis.
class ArmWave {
public:
    explicit ArmWave(const rig::Rig& rig)
        : shoulder_(boneIndex(rig, "RightUpperArm")), elbow_(boneIndex(rig, "RightForearm")),
          rest_(rig.skeleton.bones.size())
    {
        for (std::size_t i = 0; i < rest_.size(); ++i) {
            rest_[i] = rig.skeleton.bones[i].localRest;
        }
    }

    void operator()(double timeSeconds, std::span<math::Transform> pose) const
    {
        std::copy(rest_.begin(), rest_.end(), pose.begin());
        const double phase = 2.0 * std::numbers::pi * 0.5 * timeSeconds;
        const math::Vec3 x{1.0, 0.0, 0.0};
        pose[shoulder_].rotation = math::fromAxisAngle(x, 0.6 * std::sin(phase));
        pose[elbow_].rotation = math::fromAxisAngle(x, 0.75 + 0.45 * std::sin(phase));
    }

private:
    std::size_t shoulder_;
    std::size_t elbow_;
    std::vector<math::Transform> rest_;
};

inline math::Quat bodyRotation(const physics::PhysicsWorld& world, physics::BodyId body)
{
    physics::BodyState state;
    EXPECT_TRUE(world.readBody(body, state));
    return math::normalized(
        math::Quat{state.rotation.x, state.rotation.y, state.rotation.z, state.rotation.w});
}

// The joint's actual rotation, from the bodies as simulated.
inline math::Quat actualJointRotation(const physics::PhysicsWorld& world,
                                      const motor::Ragdoll& ragdoll, std::size_t j)
{
    const motor::RagdollJoint& joint = ragdoll.joints[j];
    return motor::jointRotation(ragdoll, j,
                                bodyRotation(world, ragdoll.bodies[joint.parentBody].id),
                                bodyRotation(world, ragdoll.bodies[joint.childBody].id));
}

// Every joint's target rotation for a local pose (what driveTowardsPose sets).
inline std::vector<math::Quat> targetJointRotations(const motor::Ragdoll& ragdoll,
                                                    std::span<const math::Transform> pose)
{
    const std::vector<math::Transform> model =
        rig::computeWorldPose(ragdoll.skeleton, pose, math::Transform{});
    std::vector<math::Quat> targets;
    for (std::size_t j = 0; j < ragdoll.joints.size(); ++j) {
        const motor::RagdollJoint& joint = ragdoll.joints[j];
        const auto bodyModel = [&](std::size_t b) {
            return (model[ragdoll.bodies[b].bone] * ragdoll.bodies[b].local).rotation;
        };
        targets.push_back(motor::jointRotation(ragdoll, j, bodyModel(joint.parentBody),
                                               bodyModel(joint.childBody)));
    }
    return targets;
}

// Swing-twist split of a joint-frame rotation q = swing * twist, twist about
// X (as Jolt's SwingTwistConstraintPart.h describes): the twist angle, and
// the swing's angles about Y and Z.
struct SwingTwist {
    double twistRad = 0.0;
    double swingYRad = 0.0;
    double swingZRad = 0.0;
};

inline SwingTwist splitSwingTwist(math::Quat q)
{
    q = math::normalized(q);
    if (q.w < 0.0) {
        q = math::Quat{-q.x, -q.y, -q.z, -q.w};
    }
    const double twistLen = std::hypot(q.x, q.w);
    const math::Quat twist =
        twistLen > 1e-12 ? math::Quat{q.x / twistLen, 0.0, 0.0, q.w / twistLen} : math::Quat{};
    const math::Quat swing = q * math::conjugate(twist);
    SwingTwist out;
    out.twistRad = 2.0 * std::atan2(twist.x, twist.w);
    const double vectorLen = std::hypot(swing.y, swing.z);
    const double swingAngle = 2.0 * std::atan2(vectorLen, std::abs(swing.w));
    if (vectorLen > 1e-12) {
        const double sign = swing.w < 0.0 ? -1.0 : 1.0;
        out.swingYRad = sign * swingAngle * swing.y / vectorLen;
        out.swingZRad = sign * swingAngle * swing.z / vectorLen;
    }
    return out;
}

// How far a joint-frame rotation is outside the rig joint's limits (0 inside).
// Swing about Y (plane x twist) is measured against planeHalfConeRad and
// about Z (the plane axis) against normalHalfConeRad, as Jolt does
// (SwingTwistConstraint::UpdateLimits), each on its own: a box around Jolt's
// elliptical cone,
// so it is a looser check than Jolt's own limit.
inline double limitExcessRad(const rig::Joint& joint, math::Quat q)
{
    const SwingTwist st = splitSwingTwist(q);
    const double twist =
        std::max({0.0, joint.twistMinRad - st.twistRad, st.twistRad - joint.twistMaxRad});
    const double swingY = std::max(0.0, std::abs(st.swingYRad) - joint.planeHalfConeRad);
    const double swingZ = std::max(0.0, std::abs(st.swingZRad) - joint.normalHalfConeRad);
    return std::max({twist, swingY, swingZ});
}

// Bit-for-bit equality of two body states (determinism checks): compares
// the float bit patterns, so -0 and 0 differ and NaNs compare by payload.
inline bool sameBits(const physics::BodyState& a, const physics::BodyState& b)
{
    const auto bits = [](const physics::BodyState& s) {
        return std::array<std::uint32_t, 13>{
            std::bit_cast<std::uint32_t>(s.positionM.x),
            std::bit_cast<std::uint32_t>(s.positionM.y),
            std::bit_cast<std::uint32_t>(s.positionM.z),
            std::bit_cast<std::uint32_t>(s.rotation.x),
            std::bit_cast<std::uint32_t>(s.rotation.y),
            std::bit_cast<std::uint32_t>(s.rotation.z),
            std::bit_cast<std::uint32_t>(s.rotation.w),
            std::bit_cast<std::uint32_t>(s.linearVelocityMps.x),
            std::bit_cast<std::uint32_t>(s.linearVelocityMps.y),
            std::bit_cast<std::uint32_t>(s.linearVelocityMps.z),
            std::bit_cast<std::uint32_t>(s.angularVelocityRadps.x),
            std::bit_cast<std::uint32_t>(s.angularVelocityRadps.y),
            std::bit_cast<std::uint32_t>(s.angularVelocityRadps.z)};
    };
    return bits(a) == bits(b) && a.awake == b.awake;
}

inline bool isFinite(const physics::BodyState& s)
{
    const float values[] = {s.positionM.x,
                            s.positionM.y,
                            s.positionM.z,
                            s.rotation.x,
                            s.rotation.y,
                            s.rotation.z,
                            s.rotation.w,
                            s.linearVelocityMps.x,
                            s.linearVelocityMps.y,
                            s.linearVelocityMps.z,
                            s.angularVelocityRadps.x,
                            s.angularVelocityRadps.y,
                            s.angularVelocityRadps.z};
    return std::all_of(std::begin(values), std::end(values),
                       [](float v) { return std::isfinite(v); });
}

// Height of a body's lowest point, from its rig shape and simulated pose.
inline double lowestPointZ(const rig::Shape& shape, const physics::BodyState& state)
{
    const math::Quat q = math::normalized(
        math::Quat{state.rotation.x, state.rotation.y, state.rotation.z, state.rotation.w});
    // World Z component of each local axis.
    const double ax = math::rotate(q, math::Vec3{1.0, 0.0, 0.0}).z;
    const double ay = math::rotate(q, math::Vec3{0.0, 1.0, 0.0}).z;
    const double az = math::rotate(q, math::Vec3{0.0, 0.0, 1.0}).z;
    double reach = 0.0;
    if (const auto* capsule = std::get_if<rig::CapsuleShape>(&shape)) {
        reach = std::abs(az) * capsule->halfHeightM + capsule->radiusM;
    } else if (const auto* box = std::get_if<rig::BoxShape>(&shape)) {
        reach = std::abs(ax) * box->halfExtentsM.x + std::abs(ay) * box->halfExtentsM.y +
                std::abs(az) * box->halfExtentsM.z;
    } else {
        reach = std::get<rig::SphereShape>(shape).radiusM;
    }
    return static_cast<double>(state.positionM.z) - reach;
}

inline double length(physics::Vec3 v)
{
    return std::sqrt(static_cast<double>(v.x) * v.x + static_cast<double>(v.y) * v.y +
                     static_cast<double>(v.z) * v.z);
}

} // namespace opas::test
