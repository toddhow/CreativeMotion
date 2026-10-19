#include "RagdollTestSupport.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

// The ragdoll in motion (M3, docs/tasks/0013): falling and settling, tracking
// the arm wave at 30, 60 and 120 Hz, stability, determinism, hinge signs.

namespace {

using namespace opas;
using opas::test::addFloor;
using opas::test::ArmWave;
using opas::test::buildOrFail;
using opas::test::makeWorldAtHz;
using opas::test::sampleProfile;
using opas::test::sampleRig;

// The profile with every motor turned off except damping, for falling limp.
rig::Profile limpProfile()
{
    rig::Profile profile = sampleProfile();
    profile.defaultMotor.stiffnessNmPerRad = 0.0;
    for (auto& joint : profile.joints) {
        joint.motor.stiffnessNmPerRad = 0.0;
    }
    return profile;
}

TEST(RagdollBehaviour, FallsAndSettlesOnTheFloor)
{
    const auto world = makeWorldAtHz(60);
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(addFloor(*world).isValid());
    const rig::Rig rig = sampleRig();
    // Released standing, soles 5 cm above the floor.
    const motor::Ragdoll ragdoll =
        buildOrFail(*world, rig, limpProfile(), math::Transform{math::Vec3{0.0, 0.0, 0.05}, {}});
    for (int i = 0; i < 5 * 60; ++i) {
        world->step();
    }
    for (std::size_t i = 0; i < ragdoll.bodies.size(); ++i) {
        physics::BodyState state;
        ASSERT_TRUE(world->readBody(ragdoll.bodies[i].id, state));
        ASSERT_TRUE(test::isFinite(state)) << rig.bodies[i].bone;
        if (state.awake) {
            EXPECT_LT(test::length(state.linearVelocityMps), 0.05) << rig.bodies[i].bone;
            EXPECT_LT(test::length(state.angularVelocityRadps), 0.05) << rig.bodies[i].bone;
        }
        // At most 2 cm into the floor (Jolt's penetration slop, as in M1).
        EXPECT_GE(test::lowestPointZ(rig.bodies[i].shape, state), -0.02) << rig.bodies[i].bone;
    }
    for (std::size_t j = 0; j < ragdoll.joints.size(); ++j) {
        const double excess =
            test::limitExcessRad(rig.joints[j], test::actualJointRotation(*world, ragdoll, j));
        EXPECT_LE(excess, 0.05) << rig.joints[j].bone;
    }
}

// Per joint RMS and peak tracking error of the arm wave after `settleS`,
// over `measureS`, at `hz`.
struct TrackingErrors {
    std::vector<double> rmsRad;
    std::vector<double> peakRad;
    bool finite = true;
    double fastestLinearMps = 0.0;
    double fastestAngularRadps = 0.0;
};

TrackingErrors trackArmWave(int hz, double settleS = 1.0, double measureS = 4.0)
{
    TrackingErrors out;
    const auto world = makeWorldAtHz(hz);
    EXPECT_NE(world, nullptr);
    if (world == nullptr) {
        return out;
    }
    const rig::Rig rig = sampleRig();
    motor::Ragdoll ragdoll =
        buildOrFail(*world, rig, sampleProfile(), math::Transform{}, motor::RootMotion::Kinematic);
    const ArmWave wave(rig);
    motor::RagdollDriver driver(ragdoll, wave);
    const auto listener = world->addStepListener(driver);

    const double dt = static_cast<double>(world->fixedStepSeconds());
    const int settleSteps = static_cast<int>(std::lround(settleS / dt));
    const int measureSteps = static_cast<int>(std::lround(measureS / dt));
    out.rmsRad.assign(ragdoll.joints.size(), 0.0);
    out.peakRad.assign(ragdoll.joints.size(), 0.0);
    std::vector<math::Transform> pose(rig.skeleton.bones.size());
    for (int step = 1; step <= settleSteps + measureSteps; ++step) {
        world->step();
        for (const auto& body : ragdoll.bodies) {
            physics::BodyState state;
            EXPECT_TRUE(world->readBody(body.id, state));
            out.finite = out.finite && test::isFinite(state);
            out.fastestLinearMps =
                std::max(out.fastestLinearMps, test::length(state.linearVelocityMps));
            out.fastestAngularRadps =
                std::max(out.fastestAngularRadps, test::length(state.angularVelocityRadps));
        }
        if (step <= settleSteps) {
            continue;
        }
        // The driver aimed this step at the pose for its end time, step * dt.
        wave(static_cast<double>(step) * dt, pose);
        const std::vector<math::Quat> targets = test::targetJointRotations(ragdoll, pose);
        for (std::size_t j = 0; j < ragdoll.joints.size(); ++j) {
            const double error =
                math::angleBetween(test::actualJointRotation(*world, ragdoll, j), targets[j]);
            out.rmsRad[j] += error * error;
            out.peakRad[j] = std::max(out.peakRad[j], error);
        }
    }
    for (double& rms : out.rmsRad) {
        rms = std::sqrt(rms / static_cast<double>(measureSteps));
    }
    world->removeStepListener(listener);
    return out;
}

class RagdollTracking : public ::testing::TestWithParam<int> {};

TEST_P(RagdollTracking, ArmWaveIsTrackedWithinTolerance)
{
    const int hz = GetParam();
    const rig::Rig rig = sampleRig();
    const TrackingErrors errors = trackArmWave(hz);
    ASSERT_TRUE(errors.finite);
    ASSERT_EQ(errors.rmsRad.size(), rig.joints.size());
    for (std::size_t j = 0; j < rig.joints.size(); ++j) {
        EXPECT_LE(errors.rmsRad[j], 0.10) << rig.joints[j].bone << " at " << hz << " Hz";
        EXPECT_LE(errors.peakRad[j], 0.20) << rig.joints[j].bone << " at " << hz << " Hz";
    }
}

INSTANTIATE_TEST_SUITE_P(StepRates, RagdollTracking, ::testing::Values(30, 60, 120),
                         [](const ::testing::TestParamInfo<int>& info) {
                             return std::to_string(info.param) + "Hz";
                         });

class RagdollStability : public ::testing::TestWithParam<int> {};

TEST_P(RagdollStability, FallingWithMotorsOnStaysFiniteAndBounded)
{
    const int hz = GetParam();
    const auto world = makeWorldAtHz(hz);
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(addFloor(*world).isValid());
    const rig::Rig rig = sampleRig();
    motor::Ragdoll ragdoll =
        buildOrFail(*world, rig, sampleProfile(), math::Transform{math::Vec3{0.0, 0.0, 0.05}, {}});
    const ArmWave wave(rig);
    motor::RagdollDriver driver(ragdoll, wave);
    const auto listener = world->addStepListener(driver);
    for (int i = 0; i < 10 * hz; ++i) {
        world->step();
        for (std::size_t b = 0; b < ragdoll.bodies.size(); ++b) {
            physics::BodyState state;
            ASSERT_TRUE(world->readBody(ragdoll.bodies[b].id, state));
            ASSERT_TRUE(test::isFinite(state)) << rig.bodies[b].bone << " step " << i;
            ASSERT_LT(test::length(state.linearVelocityMps), 20.0) << rig.bodies[b].bone;
            ASSERT_LT(test::length(state.angularVelocityRadps), 50.0) << rig.bodies[b].bone;
        }
    }
    world->removeStepListener(listener);

    const TrackingErrors tracking = trackArmWave(hz);
    EXPECT_TRUE(tracking.finite);
    EXPECT_LT(tracking.fastestLinearMps, 20.0);
    EXPECT_LT(tracking.fastestAngularRadps, 50.0);
}

INSTANTIATE_TEST_SUITE_P(StepRates, RagdollStability, ::testing::Values(30, 60, 120),
                         [](const ::testing::TestParamInfo<int>& info) {
                             return std::to_string(info.param) + "Hz";
                         });

TEST(RagdollStability, TrackingAt30HzIsAtMostTwiceAsBadAsAt120Hz)
{
    const TrackingErrors slow = trackArmWave(30);
    const TrackingErrors fast = trackArmWave(120);
    ASSERT_EQ(slow.rmsRad.size(), fast.rmsRad.size());
    const rig::Rig rig = sampleRig();
    // The whole body's error (RMS over every joint) at 30 Hz is at most twice
    // that at 120 Hz.
    const auto overall = [](const std::vector<double>& rms) {
        double sum = 0.0;
        for (const double r : rms) {
            sum += r * r;
        }
        return std::sqrt(sum / static_cast<double>(rms.size()));
    };
    EXPECT_LE(overall(slow.rmsRad), 2.0 * overall(fast.rmsRad));
    // Per joint, only where the 120 Hz error is meaningful (10% of the 0.10 rad
    // tolerance): a ratio against a near-zero error measures noise. Measured
    // when this was written: the soft hand (50 N m/rad) flops 0.024 rad RMS on
    // the waving forearm at 30 Hz against under 0.005 at 120 Hz, both far
    // inside the tolerance.
    for (std::size_t j = 0; j < slow.rmsRad.size(); ++j) {
        if (fast.rmsRad[j] >= 0.01) {
            EXPECT_LE(slow.rmsRad[j], 2.0 * fast.rmsRad[j]) << rig.joints[j].bone;
        }
    }
}

TEST(RagdollDeterminism, TwoWorldsTrackingTheSameWaveAreBitIdentical)
{
    const rig::Rig rig = sampleRig();
    const ArmWave wave(rig);
    const auto worldA = makeWorldAtHz(60);
    const auto worldB = makeWorldAtHz(60);
    ASSERT_NE(worldA, nullptr);
    ASSERT_NE(worldB, nullptr);
    ASSERT_TRUE(addFloor(*worldA).isValid());
    ASSERT_TRUE(addFloor(*worldB).isValid());
    const math::Transform root{math::Vec3{0.0, 0.0, 0.05}, {}};
    motor::Ragdoll ragdollA = buildOrFail(*worldA, rig, sampleProfile(), root);
    motor::Ragdoll ragdollB = buildOrFail(*worldB, rig, sampleProfile(), root);
    motor::RagdollDriver driverA(ragdollA, wave);
    motor::RagdollDriver driverB(ragdollB, wave);
    const auto listenerA = worldA->addStepListener(driverA);
    const auto listenerB = worldB->addStepListener(driverB);
    for (int step = 0; step < 300; ++step) {
        worldA->step();
        worldB->step();
        for (std::size_t b = 0; b < ragdollA.bodies.size(); ++b) {
            physics::BodyState a;
            physics::BodyState s;
            ASSERT_TRUE(worldA->readBody(ragdollA.bodies[b].id, a));
            ASSERT_TRUE(worldB->readBody(ragdollB.bodies[b].id, s));
            ASSERT_TRUE(test::sameBits(a, s)) << "body " << b << " step " << step;
        }
    }
    worldA->removeStepListener(listenerA);
    worldB->removeStepListener(listenerB);
}

// data/rigs/README.md: with the shin hanging down, a negative rotation about
// +X swings the foot backwards (knee flexion, allowed down to -2.356 rad), and
// a positive one is blocked at 0. M3 confirms the sign against Jolt.
TEST(RagdollHinges, KneeBendsBackwardsWithNegativeTwist)
{
    const auto world = makeWorldAtHz(60, false);
    ASSERT_NE(world, nullptr);
    const rig::Rig rig = sampleRig();
    motor::Ragdoll ragdoll =
        buildOrFail(*world, rig, sampleProfile(), math::Transform{}, motor::RootMotion::Kinematic);
    const std::size_t shin = test::boneIndex(rig, "LeftShin");
    const std::size_t foot = rig::findBody(rig, "LeftFoot").value_or(0);
    const std::size_t knee = rig::findJoint(rig, "LeftShin").value_or(0);

    std::vector<math::Transform> pose(rig.skeleton.bones.size());
    for (std::size_t i = 0; i < pose.size(); ++i) {
        pose[i] = rig.skeleton.bones[i].localRest;
    }
    const auto holdFor = [&](double angleRad) {
        pose[shin].rotation = math::fromAxisAngle(math::Vec3{1.0, 0.0, 0.0}, angleRad);
        for (int i = 0; i < 120; ++i) {
            motor::driveTowardsPose(*world, ragdoll, pose);
            world->step();
        }
    };

    holdFor(-1.0);
    physics::BodyState footState;
    ASSERT_TRUE(world->readBody(ragdoll.bodies[foot].id, footState));
    EXPECT_LT(footState.positionM.y, -0.2f); // the foot swung backwards (-Y)
    EXPECT_NEAR(test::splitSwingTwist(test::actualJointRotation(*world, ragdoll, knee)).twistRad,
                -1.0, 0.05);

    holdFor(0.5); // beyond the limit at 0
    EXPECT_LE(test::splitSwingTwist(test::actualJointRotation(*world, ragdoll, knee)).twistRad,
              0.03);
}

} // namespace
