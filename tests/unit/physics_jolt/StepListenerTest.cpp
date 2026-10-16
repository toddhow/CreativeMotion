#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

enum class Phase : std::uint8_t { Pre, Post };

struct Call {
    int listener = 0;
    Phase phase = Phase::Pre;
    float dtSeconds = 0.0f;
    const PhysicsWorld* world = nullptr;
};

// Appends every callback to a log shared by several listeners, so the log
// shows the order across listeners.
class RecordingListener : public StepListener {
public:
    RecordingListener(int id, std::vector<Call>& log) : id_(id), log_(log) {}

    void onPreStep(PhysicsWorld& world, float dtSeconds) override
    {
        log_.push_back(Call{id_, Phase::Pre, dtSeconds, &world});
    }

    void onPostStep(PhysicsWorld& world, float dtSeconds) override
    {
        log_.push_back(Call{id_, Phase::Post, dtSeconds, &world});
    }

private:
    int id_;
    std::vector<Call>& log_;
};

TEST(JoltStepListeners, PreAndPostCalledOncePerStepInRegistrationOrderWithFixedDt)
{
    // Test choice: a non-default step, so a hard-coded 1/60 would be caught.
    WorldConfig config;
    config.fixedStepSeconds = 1.0f / 120.0f;
    const auto world = makeWorld(config);
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(world->createBody(BodyDesc{}).ok());

    std::vector<Call> log;
    RecordingListener first(0, log);
    RecordingListener second(1, log);
    RecordingListener third(2, log);
    const StepListenerId firstId = world->addStepListener(first);
    const StepListenerId secondId = world->addStepListener(second);
    const StepListenerId thirdId = world->addStepListener(third);
    EXPECT_TRUE(firstId.isValid());
    EXPECT_TRUE(secondId.isValid());
    EXPECT_TRUE(thirdId.isValid());
    EXPECT_NE(firstId, secondId);
    EXPECT_NE(secondId, thirdId);
    EXPECT_NE(firstId, thirdId);

    constexpr int kSteps = 3;
    for (int i = 0; i < kSteps; ++i) {
        world->step();
    }

    // Each step: every pre-step in registration order, then every post-step.
    const std::vector<std::pair<int, Phase>> perStep = {
        {0, Phase::Pre},  {1, Phase::Pre},  {2, Phase::Pre},
        {0, Phase::Post}, {1, Phase::Post}, {2, Phase::Post},
    };
    ASSERT_EQ(log.size(), perStep.size() * kSteps);
    for (std::size_t i = 0; i < log.size(); ++i) {
        const auto& expected = perStep[i % perStep.size()];
        EXPECT_EQ(log[i].listener, expected.first) << "call " << i;
        EXPECT_EQ(log[i].phase, expected.second) << "call " << i;
        EXPECT_TRUE(bitEqual(log[i].dtSeconds, config.fixedStepSeconds))
            << "call " << i << " dt " << log[i].dtSeconds;
        EXPECT_EQ(log[i].world, world.get()) << "call " << i;
    }

    world->removeStepListener(firstId);
    world->removeStepListener(secondId);
    world->removeStepListener(thirdId);
}

TEST(JoltStepListeners, RemovedListenerIsNotCalled)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    std::vector<Call> log;
    RecordingListener kept(0, log);
    RecordingListener removed(1, log);
    const StepListenerId keptId = world->addStepListener(kept);
    const StepListenerId removedId = world->addStepListener(removed);

    world->step();
    ASSERT_EQ(log.size(), 4U);
    log.clear();

    EXPECT_TRUE(world->removeStepListener(removedId));
    EXPECT_FALSE(world->removeStepListener(removedId));
    world->step();

    ASSERT_EQ(log.size(), 2U);
    EXPECT_EQ(log[0].listener, 0);
    EXPECT_EQ(log[0].phase, Phase::Pre);
    EXPECT_EQ(log[1].listener, 0);
    EXPECT_EQ(log[1].phase, Phase::Post);

    EXPECT_TRUE(world->removeStepListener(keptId));
    log.clear();
    world->step();
    EXPECT_TRUE(log.empty());
}

// From inside a callback, tries every structural change (all must fail) and
// some driving and reading (all must succeed).
class StructuralChangeListener : public StepListener {
public:
    struct Outcome {
        CreateResult<BodyId> createBody;
        bool destroyBody = true;
        CreateResult<JointId> createJoint;
        bool destroyJoint = true;
        bool readBody = false;
        bool applyImpulse = false;
        bool setJointMotor = false;
    };

    StructuralChangeListener(BodyId existingBody, JointId existingJoint, JointDesc newJointDesc)
        : existingBody_(existingBody), existingJoint_(existingJoint), newJointDesc_(newJointDesc)
    {
        // Reserved up front so recording does not allocate inside the step.
        pre.reserve(4);
        post.reserve(4);
    }

    void onPreStep(PhysicsWorld& world, float /*dtSeconds*/) override
    {
        pre.push_back(tryEverything(world));
    }

    void onPostStep(PhysicsWorld& world, float /*dtSeconds*/) override
    {
        post.push_back(tryEverything(world));
    }

    std::vector<Outcome> pre;
    std::vector<Outcome> post;

private:
    Outcome tryEverything(PhysicsWorld& world) const
    {
        Outcome outcome;
        outcome.createBody = world.createBody(BodyDesc{});
        outcome.destroyBody = world.destroyBody(existingBody_);
        outcome.createJoint = world.createJoint(newJointDesc_);
        outcome.destroyJoint = world.destroyJoint(existingJoint_);
        BodyState state;
        outcome.readBody = world.readBody(existingBody_, state);
        outcome.applyImpulse =
            world.applyImpulse(existingBody_, Vec3{0.0f, 0.0f, 0.01f}, state.positionM);
        outcome.setJointMotor = world.setJointMotor(existingJoint_, JointMotor{});
        return outcome;
    }

    BodyId existingBody_;
    JointId existingJoint_;
    JointDesc newJointDesc_;
};

void expectStructuralChangesFailed(const StructuralChangeListener::Outcome& outcome,
                                   const char* phase)
{
    SCOPED_TRACE(phase);
    EXPECT_EQ(outcome.createBody.error, ErrorCode::CalledDuringStep) << outcome.createBody.message;
    EXPECT_FALSE(outcome.createBody.id.isValid());
    EXPECT_FALSE(outcome.destroyBody);
    EXPECT_EQ(outcome.createJoint.error, ErrorCode::CalledDuringStep)
        << outcome.createJoint.message;
    EXPECT_FALSE(outcome.createJoint.id.isValid());
    EXPECT_FALSE(outcome.destroyJoint);
    // Listeners may still read and drive.
    EXPECT_TRUE(outcome.readBody);
    EXPECT_TRUE(outcome.applyImpulse);
    EXPECT_TRUE(outcome.setJointMotor);
}

TEST(JoltStepListeners, StructuralChangesDuringCallbacksFailWithCalledDuringStep)
{
    const auto world = makeWorld(zeroGravityConfig());
    ASSERT_NE(world, nullptr);
    const JointedPair pair = createJointedPair(*world, degToRad(-30.0f), degToRad(30.0f));
    const auto joint = world->createJoint(pair.jointDesc);
    ASSERT_TRUE(joint.ok()) << joint.message;

    StructuralChangeListener listener(pair.box, joint.id, pair.jointDesc);
    const StepListenerId listenerId = world->addStepListener(listener);
    world->step();
    ASSERT_TRUE(world->removeStepListener(listenerId));

    ASSERT_EQ(listener.pre.size(), 1U);
    ASSERT_EQ(listener.post.size(), 1U);
    expectStructuralChangesFailed(listener.pre[0], "pre-step");
    expectStructuralChangesFailed(listener.post[0], "post-step");

    // The failed destroys really did nothing.
    BodyState state;
    EXPECT_TRUE(world->readBody(pair.box, state));
    EXPECT_TRUE(world->setJointMotor(joint.id, JointMotor{}));

    // Outside a step, structural changes work again.
    EXPECT_TRUE(world->createBody(BodyDesc{}).ok());
    EXPECT_TRUE(world->createJoint(pair.jointDesc).ok());
    EXPECT_TRUE(world->destroyJoint(joint.id));
    EXPECT_TRUE(world->destroyBody(pair.box));
}

} // namespace
