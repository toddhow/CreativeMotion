#include "opas/motor/MotorSolver.hpp"

#include "Conversions.hpp"

#include <utility>

namespace opas::motor {

bool driveTowardsPose(physics::PhysicsWorld& world, Ragdoll& ragdoll,
                      std::span<const math::Transform> localPose)
{
    if (localPose.size() != ragdoll.skeleton.bones.size() ||
        ragdoll.poseModel.size() != ragdoll.skeleton.bones.size()) {
        return false;
    }
    // Model-space pose (root = identity): joint targets depend only on
    // relative rotations, so the root's placement cancels out.
    rig::computeWorldPoseInto(ragdoll.skeleton, localPose, math::Transform{}, ragdoll.poseModel);

    const auto bodyModel = [&ragdoll](std::size_t bodyIndex) {
        const RagdollBody& body = ragdoll.bodies[bodyIndex];
        return ragdoll.poseModel[body.bone] * body.local;
    };

    for (std::size_t j = 0; j < ragdoll.joints.size(); ++j) {
        const RagdollJoint& joint = ragdoll.joints[j];
        const math::Quat target = jointRotation(ragdoll, j, bodyModel(joint.parentBody).rotation,
                                                bodyModel(joint.childBody).rotation);
        world.setJointTarget(joint.id, toPhysics(target));
    }

    if (ragdoll.kinematicRoot) {
        const math::Transform rootTarget = ragdoll.rootWorld * bodyModel(ragdoll.rootBody);
        world.setKinematicTarget(ragdoll.bodies[ragdoll.rootBody].id,
                                 toPhysics(rootTarget.translation), toPhysics(rootTarget.rotation));
    }
    return true;
}

RagdollDriver::RagdollDriver(Ragdoll& ragdoll, PoseSource source)
    : ragdoll_(&ragdoll), source_(std::move(source)), localPose_(ragdoll.skeleton.bones.size())
{
    // Start from the rest pose, so a source that fills only some bones still
    // gives a complete pose.
    for (std::size_t i = 0; i < localPose_.size(); ++i) {
        localPose_[i] = ragdoll.skeleton.bones[i].localRest;
    }
}

void RagdollDriver::onPreStep(physics::PhysicsWorld& world, float dtSeconds)
{
    const double timeSeconds = static_cast<double>(steps_ + 1) * static_cast<double>(dtSeconds);
    source_(timeSeconds, localPose_);
    driveTowardsPose(world, *ragdoll_, localPose_);
    ++steps_;
}

void RagdollDriver::onPostStep(physics::PhysicsWorld& /*world*/, float /*dtSeconds*/)
{}

} // namespace opas::motor
