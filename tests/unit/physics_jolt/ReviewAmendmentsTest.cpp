// Tests for the criteria added in "Amendments after the fresh review" of
// docs/tasks/0001-m1-physics-interface.md.

#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

// --- Kinematic hold --------------------------------------------------------------

TEST(JoltKinematic, KinematicBodyStaysAtTargetWithZeroVelocityAfterReachingIt)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    BodyDesc desc;
    desc.shape = BoxShape{Vec3{0.2f, 0.2f, 0.2f}};
    desc.motion = MotionType::Kinematic;
    desc.positionM = Vec3{0.0f, 0.0f, 1.0f};
    const auto body = world->createBody(desc);
    ASSERT_TRUE(body.ok()) << body.message;

    const Vec3 targetPositionM{0.5f, -0.25f, 1.3f};
    const Quat targetRotation = axisAngle(Vec3{0.0f, 0.6f, 0.8f}, degToRad(30.0f));
    ASSERT_TRUE(world->setKinematicTarget(body.id, targetPositionM, targetRotation));
    world->step();

    // Test choice: one second of further steps; "zero velocity" is checked
    // to 1e-6 because the spec gives no tolerance.
    for (int i = 0; i < kStepsPerSecond; ++i) {
        world->step();
        BodyState state;
        ASSERT_TRUE(world->readBody(body.id, state));
        EXPECT_LE(distance(state.positionM, targetPositionM), 1e-5) << "step " << i;
        EXPECT_LE(angleBetween(state.rotation, targetRotation), 1e-4) << "step " << i;
        EXPECT_LE(length(state.linearVelocityMps), 1e-6) << "step " << i;
        EXPECT_LE(length(state.angularVelocityRadps), 1e-6) << "step " << i;
    }
}

// --- Damping-only motor ------------------------------------------------------------

// Spins the jointed box at 2 rad/s about the twist axis under `motor` and
// returns its angular speed after 1 s.
double angularSpeedAfterOneSecond(const JointMotor& motor)
{
    const auto world = makeWorld(zeroGravityConfig());
    EXPECT_NE(world, nullptr);
    if (world == nullptr) {
        return 0.0;
    }
    // Test choice: twist limits of +-pi so the box can turn the ~2 rad it
    // covers in 1 s with the motor off without touching a limit.
    const JointedPair pair = createJointedPair(*world, -kPi, kPi);
    const auto joint = world->createJoint(pair.jointDesc);
    EXPECT_TRUE(joint.ok()) << joint.message;
    EXPECT_TRUE(world->setJointMotor(joint.id, motor));
    EXPECT_TRUE(world->setVelocity(pair.box, Vec3{}, Vec3{2.0f, 0.0f, 0.0f}));

    for (int i = 0; i < kStepsPerSecond; ++i) {
        world->step();
    }
    BodyState state;
    EXPECT_TRUE(world->readBody(pair.box, state));
    return length(state.angularVelocityRadps);
}

TEST(JoltJointMotor, DampingOnlyMotorSlowsSpinningBoxWithinOneSecond)
{
    JointMotor motor;
    motor.stiffnessNmPerRad = 0.0f;
    motor.dampingNmsPerRad = 5.0f;
    EXPECT_LT(angularSpeedAfterOneSecond(motor), 0.5);
}

TEST(JoltJointMotor, MotorOffLetsSpinningBoxKeepSpinning)
{
    // Test choice for "keeps spinning": only the body's default angular
    // damping (0.05 /s) acts, so about 1.9 rad/s should remain; 1.5 leaves
    // margin while staying far above the damped case's 0.5.
    EXPECT_GT(angularSpeedAfterOneSecond(JointMotor{}), 1.5);
}

// --- Motor validation --------------------------------------------------------------

class JoltMotorValidation : public ::testing::Test {
protected:
    void SetUp() override
    {
        world_ = makeWorld(zeroGravityConfig());
        ASSERT_NE(world_, nullptr);
        const JointedPair pair = createJointedPair(*world_, degToRad(-30.0f), degToRad(30.0f));
        const auto joint = world_->createJoint(pair.jointDesc);
        ASSERT_TRUE(joint.ok()) << joint.message;
        joint_ = joint.id;
    }

    static JointMotor validMotor()
    {
        JointMotor motor;
        motor.stiffnessNmPerRad = 50.0f;
        motor.dampingNmsPerRad = 5.0f;
        motor.maxTorqueNm = 100.0f;
        return motor;
    }

    std::unique_ptr<PhysicsWorld> world_;
    JointId joint_;
};

// Negative and NaN come from the spec; infinity from the header's "rejects
// negative or non-finite values".
constexpr float kBadValues[] = {-1.0f, std::numeric_limits<float>::quiet_NaN(),
                                std::numeric_limits<float>::infinity()};

TEST_F(JoltMotorValidation, ValidMotorIsAccepted)
{
    EXPECT_TRUE(world_->setJointMotor(joint_, validMotor()));
}

TEST_F(JoltMotorValidation, NegativeOrNonFiniteStiffnessIsRejected)
{
    for (const float bad : kBadValues) {
        JointMotor motor = validMotor();
        motor.stiffnessNmPerRad = bad;
        EXPECT_FALSE(world_->setJointMotor(joint_, motor)) << "stiffness " << bad;
    }
}

TEST_F(JoltMotorValidation, NegativeOrNonFiniteDampingIsRejected)
{
    for (const float bad : kBadValues) {
        JointMotor motor = validMotor();
        motor.dampingNmsPerRad = bad;
        EXPECT_FALSE(world_->setJointMotor(joint_, motor)) << "damping " << bad;
    }
}

TEST_F(JoltMotorValidation, NegativeOrNonFiniteMaxTorqueIsRejected)
{
    for (const float bad : kBadValues) {
        JointMotor motor = validMotor();
        motor.maxTorqueNm = bad;
        EXPECT_FALSE(world_->setJointMotor(joint_, motor)) << "maxTorqueNm " << bad;
    }
}

// --- Listener changes during a step ------------------------------------------------

enum class Phase : std::uint8_t { Pre, Post };

struct Call {
    int listener = 0;
    Phase phase = Phase::Pre;
};

// Logs its callbacks and optionally runs an action from one of them.
class ScriptedListener : public StepListener {
public:
    enum class Action : std::uint8_t { None, RemoveSelfInPre, AddOtherInPre, AddOtherInPost };

    ScriptedListener(int id, std::vector<Call>& log) : id_(id), log_(log) {}

    void script(Action action, StepListener* other = nullptr)
    {
        action_ = action;
        other_ = other;
    }

    void onPreStep(PhysicsWorld& world, float /*dtSeconds*/) override
    {
        log_.push_back(Call{id_, Phase::Pre});
        if (action_ == Action::RemoveSelfInPre) {
            actionResult = world.removeStepListener(selfId);
            action_ = Action::None;
        } else if (action_ == Action::AddOtherInPre) {
            addedId = world.addStepListener(*other_);
            action_ = Action::None;
        }
    }

    void onPostStep(PhysicsWorld& world, float /*dtSeconds*/) override
    {
        log_.push_back(Call{id_, Phase::Post});
        if (action_ == Action::AddOtherInPost) {
            addedId = world.addStepListener(*other_);
            action_ = Action::None;
        }
    }

    StepListenerId selfId;
    StepListenerId addedId;
    bool actionResult = false;

private:
    int id_;
    std::vector<Call>& log_;
    Action action_ = Action::None;
    StepListener* other_ = nullptr;
};

::testing::AssertionResult logIs(const std::vector<Call>& log, const std::vector<Call>& expected)
{
    bool same = log.size() == expected.size();
    for (std::size_t i = 0; same && i < log.size(); ++i) {
        same = log[i].listener == expected[i].listener && log[i].phase == expected[i].phase;
    }
    if (same) {
        return ::testing::AssertionSuccess();
    }
    auto failure = ::testing::AssertionFailure() << "log was:";
    for (const Call& call : log) {
        failure << " " << call.listener << (call.phase == Phase::Pre ? "pre" : "post");
    }
    return failure;
}

TEST(JoltStepListeners, ListenerRemovingItselfInPreStepDoesNotStopNextListener)
{
    // Listeners are declared before the world so they outlive it.
    std::vector<Call> log;
    ScriptedListener remover(0, log);
    ScriptedListener next(1, log);
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    remover.selfId = world->addStepListener(remover);
    const StepListenerId nextId = world->addStepListener(next);
    remover.script(ScriptedListener::Action::RemoveSelfInPre);

    world->step();
    EXPECT_TRUE(remover.actionResult);
    // The remover is not called again, not even for this step's post-step.
    EXPECT_TRUE(logIs(log, {{0, Phase::Pre}, {1, Phase::Pre}, {1, Phase::Post}}));

    log.clear();
    world->step();
    EXPECT_TRUE(logIs(log, {{1, Phase::Pre}, {1, Phase::Post}}));

    EXPECT_FALSE(world->removeStepListener(remover.selfId));
    EXPECT_TRUE(world->removeStepListener(nextId));
}

TEST(JoltStepListeners, ListenerAddedInPreStepIsFirstCalledOnNextStep)
{
    std::vector<Call> log;
    ScriptedListener adder(0, log);
    ScriptedListener added(1, log);
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    const StepListenerId adderId = world->addStepListener(adder);
    adder.script(ScriptedListener::Action::AddOtherInPre, &added);

    world->step();
    EXPECT_TRUE(adder.addedId.isValid());
    EXPECT_TRUE(logIs(log, {{0, Phase::Pre}, {0, Phase::Post}}));

    log.clear();
    world->step();
    EXPECT_TRUE(logIs(log, {{0, Phase::Pre}, {1, Phase::Pre}, {0, Phase::Post}, {1, Phase::Post}}));

    EXPECT_TRUE(world->removeStepListener(adderId));
    EXPECT_TRUE(world->removeStepListener(adder.addedId));
}

TEST(JoltStepListeners, ListenerAddedInPostStepIsFirstCalledOnNextStep)
{
    std::vector<Call> log;
    ScriptedListener adder(0, log);
    ScriptedListener added(1, log);
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    const StepListenerId adderId = world->addStepListener(adder);
    adder.script(ScriptedListener::Action::AddOtherInPost, &added);

    world->step();
    EXPECT_TRUE(adder.addedId.isValid());
    EXPECT_TRUE(logIs(log, {{0, Phase::Pre}, {0, Phase::Post}}));

    log.clear();
    world->step();
    EXPECT_TRUE(logIs(log, {{0, Phase::Pre}, {1, Phase::Pre}, {0, Phase::Post}, {1, Phase::Post}}));

    EXPECT_TRUE(world->removeStepListener(adderId));
    EXPECT_TRUE(world->removeStepListener(adder.addedId));
}

// --- Capsule axis ------------------------------------------------------------------

TEST(JoltCapsule, CapsuleWithIdentityRotationIsLongAlongZ)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);

    // Test choice: a static capsule centred at z = 1, so its top is at 1.75.
    const float centreZ = 1.0f;
    BodyDesc desc;
    desc.shape = CapsuleShape{0.5f, 0.25f};
    desc.motion = MotionType::Static;
    desc.positionM = Vec3{0.0f, 0.0f, centreZ};
    const auto capsule = world->createBody(desc);
    ASSERT_TRUE(capsule.ok()) << capsule.message;

    const auto down = world->castRay(Vec3{0.0f, 0.0f, 5.0f}, Vec3{0.0f, 0.0f, -10.0f});
    ASSERT_TRUE(down.has_value());
    EXPECT_EQ(down->body, capsule.id);
    // halfHeight + radius = 0.75 above the centre.
    EXPECT_NEAR(down->pointM.z, centreZ + 0.75f, 1e-3f);

    // Test choice: from the side it is only as wide as its radius.
    const auto side = world->castRay(Vec3{5.0f, 0.0f, centreZ}, Vec3{-10.0f, 0.0f, 0.0f});
    ASSERT_TRUE(side.has_value());
    EXPECT_NEAR(side->pointM.x, 0.25f, 1e-3f);
}

// --- Drive wakes sleeping bodies ---------------------------------------------------

// Creates a 1 kg box resting on the floor and steps until it sleeps.
BodyId createSleepingBox(PhysicsWorld& world)
{
    createFloor(world);
    const BodyId box =
        world.createBody(dynamicBox(Vec3{0.5f, 0.5f, 0.5f}, 1.0f, Vec3{0.0f, 0.0f, 0.5f})).id;
    // Jolt's default time before sleep is 0.5 s; 3 s leaves plenty of margin.
    for (int i = 0; i < 3 * kStepsPerSecond; ++i) {
        world.step();
        BodyState state;
        if (world.readBody(box, state) && !state.awake) {
            break;
        }
    }
    return box;
}

TEST(JoltSleep, ImpulseWakesSleepingBoxAndMovesIt)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    const BodyId box = createSleepingBox(*world);
    BodyState before;
    ASSERT_TRUE(world->readBody(box, before));
    ASSERT_FALSE(before.awake) << "box never fell asleep";

    ASSERT_TRUE(world->applyImpulse(box, Vec3{2.0f, 0.0f, 0.0f}, before.positionM));
    world->step();

    BodyState after;
    ASSERT_TRUE(world->readBody(box, after));
    EXPECT_TRUE(after.awake);
    // Test choice: 2 N s on 1 kg gives 2 m/s; friction removes only ~0.03 m/s
    // per step, so well over 1 m/s must remain.
    EXPECT_GT(after.linearVelocityMps.x, 1.0f);

    for (int i = 0; i < 10; ++i) {
        world->step();
    }
    ASSERT_TRUE(world->readBody(box, after));
    EXPECT_GT(after.positionM.x - before.positionM.x, 0.1f);
}

TEST(JoltSleep, SetVelocityWakesSleepingBoxAndMovesIt)
{
    // Test choice: the criterion says "drive", so setVelocity is checked too.
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    const BodyId box = createSleepingBox(*world);
    BodyState before;
    ASSERT_TRUE(world->readBody(box, before));
    ASSERT_FALSE(before.awake) << "box never fell asleep";

    ASSERT_TRUE(world->setVelocity(box, Vec3{2.0f, 0.0f, 0.0f}, Vec3{}));
    for (int i = 0; i < 10; ++i) {
        world->step();
    }

    BodyState after;
    ASSERT_TRUE(world->readBody(box, after));
    EXPECT_GT(after.positionM.x - before.positionM.x, 0.1f);
}

} // namespace
