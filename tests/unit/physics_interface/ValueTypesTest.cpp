#include "opas/physics/Descriptions.hpp"
#include "opas/physics/PhysicsWorld.hpp"
#include "opas/physics/Types.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>
#include <variant>

namespace {

using namespace opas::physics;

// Different tags make the handle types distinct, so a JointId can never be
// passed where a BodyId is expected.
static_assert(!std::is_same_v<BodyId, JointId>);
static_assert(!std::is_same_v<BodyId, StepListenerId>);
static_assert(!std::is_same_v<JointId, StepListenerId>);

TEST(PhysicsHandles, DefaultConstructedHandlesAreInvalid)
{
    EXPECT_FALSE(BodyId{}.isValid());
    EXPECT_FALSE(JointId{}.isValid());
    EXPECT_FALSE(StepListenerId{}.isValid());
}

TEST(PhysicsHandles, HandleWithIndexIsValid)
{
    BodyId id;
    id.index = 0;
    EXPECT_TRUE(id.isValid());
}

TEST(PhysicsHandles, DefaultConstructedHandlesCompareEqual)
{
    EXPECT_EQ(BodyId{}, BodyId{});
    EXPECT_EQ(JointId{}, JointId{});
    EXPECT_EQ(StepListenerId{}, StepListenerId{});
}

TEST(PhysicsHandles, HandlesDifferingInIndexOrGenerationAreNotEqual)
{
    BodyId a;
    a.index = 3;
    a.generation = 1;

    BodyId sameSlotNewGeneration = a;
    sameSlotNewGeneration.generation = 2;

    BodyId otherSlot = a;
    otherSlot.index = 4;

    const BodyId copy = a;

    EXPECT_EQ(a, copy);
    // A reused slot with a new generation must not alias the old handle.
    EXPECT_NE(a, sameSlotNewGeneration);
    EXPECT_NE(a, otherSlot);
    EXPECT_NE(a, BodyId{});
}

TEST(PhysicsValueTypes, Vec3DefaultsToZero)
{
    const Vec3 v;
    EXPECT_EQ(v.x, 0.0f);
    EXPECT_EQ(v.y, 0.0f);
    EXPECT_EQ(v.z, 0.0f);
}

TEST(PhysicsValueTypes, QuatDefaultsToIdentity)
{
    const Quat q;
    EXPECT_EQ(q.x, 0.0f);
    EXPECT_EQ(q.y, 0.0f);
    EXPECT_EQ(q.z, 0.0f);
    EXPECT_EQ(q.w, 1.0f);
    EXPECT_EQ(q, (Quat{0.0f, 0.0f, 0.0f, 1.0f}));
}

TEST(PhysicsValueTypes, Vec3AndQuatEqualityComparesAllComponents)
{
    EXPECT_EQ((Vec3{1.0f, 2.0f, 3.0f}), (Vec3{1.0f, 2.0f, 3.0f}));
    EXPECT_NE((Vec3{1.0f, 2.0f, 3.0f}), (Vec3{1.0f, 2.0f, 4.0f}));
    EXPECT_NE((Quat{0.0f, 0.0f, 0.0f, 1.0f}), (Quat{0.0f, 0.0f, 1.0f, 0.0f}));
}

TEST(PhysicsCreateResult, DefaultIsOkWithInvalidId)
{
    const CreateResult<BodyId> result;
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(result.error, ErrorCode::None);
    EXPECT_FALSE(result.id.isValid());
    EXPECT_TRUE(result.message.empty());
}

TEST(PhysicsCreateResult, AnyErrorCodeIsNotOk)
{
    for (const ErrorCode code : {ErrorCode::InvalidDescription, ErrorCode::InvalidHandle,
                                 ErrorCode::CapacityExceeded, ErrorCode::CalledDuringStep}) {
        CreateResult<JointId> result;
        result.error = code;
        EXPECT_FALSE(result.ok());
    }
}

TEST(PhysicsDefaults, WorldConfigMatchesDocumentedDefaults)
{
    const WorldConfig config;
    EXPECT_FLOAT_EQ(config.fixedStepSeconds, 1.0f / 60.0f);
    EXPECT_EQ(config.gravityMps2, (Vec3{0.0f, 0.0f, -9.81f}));
    EXPECT_EQ(config.maxBodies, 1024U);
    EXPECT_EQ(config.maxJoints, 1024U);
}

TEST(PhysicsDefaults, BodyDescMatchesDocumentedDefaults)
{
    const BodyDesc desc;
    ASSERT_TRUE(std::holds_alternative<BoxShape>(desc.shape));
    EXPECT_EQ(std::get<BoxShape>(desc.shape).halfExtentsM, (Vec3{0.5f, 0.5f, 0.5f}));
    EXPECT_EQ(desc.motion, MotionType::Dynamic);
    EXPECT_EQ(desc.positionM, Vec3{});
    EXPECT_EQ(desc.rotation, Quat{});
    EXPECT_EQ(desc.massKg, 1.0f);
    EXPECT_EQ(desc.friction, 0.2f);
    EXPECT_EQ(desc.restitution, 0.0f);
    EXPECT_EQ(desc.linearDamping, 0.05f);
    EXPECT_EQ(desc.angularDamping, 0.05f);
}

TEST(PhysicsDefaults, ShapesHavePositiveDefaultSizes)
{
    EXPECT_EQ(BoxShape{}.halfExtentsM, (Vec3{0.5f, 0.5f, 0.5f}));
    EXPECT_EQ(SphereShape{}.radiusM, 0.5f);
    EXPECT_EQ(CapsuleShape{}.halfHeightM, 0.5f);
    EXPECT_EQ(CapsuleShape{}.radiusM, 0.25f);
}

TEST(PhysicsDefaults, JointDescHasInvalidBodiesAndPerpendicularUnitAxes)
{
    const JointDesc desc;
    EXPECT_FALSE(desc.bodyA.isValid());
    EXPECT_FALSE(desc.bodyB.isValid());
    EXPECT_EQ(desc.twistAxis, (Vec3{1.0f, 0.0f, 0.0f}));
    EXPECT_EQ(desc.planeAxis, (Vec3{0.0f, 1.0f, 0.0f}));
    EXPECT_EQ(desc.normalHalfConeRad, 0.0f);
    EXPECT_EQ(desc.planeHalfConeRad, 0.0f);
    EXPECT_EQ(desc.twistMinRad, 0.0f);
    EXPECT_EQ(desc.twistMaxRad, 0.0f);
}

TEST(PhysicsDefaults, JointMotorIsOffWithUnlimitedTorque)
{
    const JointMotor motor;
    EXPECT_EQ(motor.stiffnessNmPerRad, 0.0f);
    EXPECT_EQ(motor.dampingNmsPerRad, 0.0f);
    EXPECT_EQ(motor.maxTorqueNm, std::numeric_limits<float>::max());
}

TEST(PhysicsDefaults, BodyStateAndRayHitStartZeroed)
{
    const BodyState state;
    EXPECT_EQ(state.positionM, Vec3{});
    EXPECT_EQ(state.rotation, Quat{});
    EXPECT_EQ(state.linearVelocityMps, Vec3{});
    EXPECT_EQ(state.angularVelocityRadps, Vec3{});
    EXPECT_FALSE(state.awake);

    const RayHit hit;
    EXPECT_FALSE(hit.body.isValid());
    EXPECT_EQ(hit.fraction, 0.0f);
}

} // namespace
