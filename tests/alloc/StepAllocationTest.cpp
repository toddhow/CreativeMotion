#include "CountingNew.hpp"
#include "RagdollTestSupport.hpp"

#include <gtest/gtest.h>

// M3 (docs/tasks/0013): the per-step update allocates nothing. Counts every
// global operator new while a driven ragdoll steps: the motor solver, the
// backend calls it makes, and the world's own step. Jolt allocates through
// its own hooks and our backend's preallocated TempAllocatorImpl, so those
// are not operator new calls.

namespace {

using namespace opas;

void expectNoAllocationsWhileStepping(motor::RootMotion rootMotion)
{
    const auto world = test::makeWorldAtHz(60);
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(test::addFloor(*world).isValid());
    const rig::Rig rig = test::sampleRig();
    motor::Ragdoll ragdoll =
        test::buildOrFail(*world, rig, test::sampleProfile(),
                          math::Transform{math::Vec3{0.0, 0.0, 0.05}, {}}, rootMotion);
    const test::ArmWave wave(rig);
    motor::RagdollDriver driver(ragdoll, wave);
    const auto listener = world->addStepListener(driver);

    world->step(); // warm-up: first-use setup is allowed
    test::startCountingAllocations();
    for (int i = 0; i < 600; ++i) {
        world->step();
    }
    const std::uint64_t allocations = test::stopCountingAllocations();
    EXPECT_EQ(allocations, 0U);
    world->removeStepListener(listener);
}

TEST(StepAllocations, DrivenRagdollWithKinematicRootAllocatesNothing)
{
    expectNoAllocationsWhileStepping(motor::RootMotion::Kinematic);
}

TEST(StepAllocations, DrivenFallingRagdollAllocatesNothing)
{
    expectNoAllocationsWhileStepping(motor::RootMotion::Dynamic);
}

TEST(StepAllocations, TheCounterSeesAllocations)
{
    test::startCountingAllocations();
    const auto* volatile p = new int(7); // NOLINT(cppcoreguidelines-owning-memory)
    delete p;                            // NOLINT(cppcoreguidelines-owning-memory)
    EXPECT_GE(test::stopCountingAllocations(), 1U);
}

} // namespace
