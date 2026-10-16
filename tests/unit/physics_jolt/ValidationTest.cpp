#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

#include <initializer_list>
#include <string>
#include <string_view>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

bool mentionsAny(const std::string& message, std::initializer_list<std::string_view> fields)
{
    for (const std::string_view field : fields) {
        if (message.find(field) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// A rejected create must fail with `expected`, return an invalid id and name
// one of `fields` in its message.
template <typename Id>
::testing::AssertionResult isRejected(const CreateResult<Id>& result, ErrorCode expected,
                                      std::initializer_list<std::string_view> fields)
{
    if (result.ok()) {
        return ::testing::AssertionFailure() << "create succeeded";
    }
    if (result.error != expected) {
        return ::testing::AssertionFailure()
               << "error code " << static_cast<int>(result.error) << ", expected "
               << static_cast<int>(expected) << "; message: " << result.message;
    }
    if (result.id.isValid()) {
        return ::testing::AssertionFailure() << "id is valid on failure";
    }
    if (!mentionsAny(result.message, fields)) {
        return ::testing::AssertionFailure()
               << "message does not name the field: \"" << result.message << "\"";
    }
    return ::testing::AssertionSuccess();
}

class JoltValidation : public ::testing::Test {
protected:
    void SetUp() override
    {
        world_ = makeWorld(zeroGravityConfig());
        ASSERT_NE(world_, nullptr);
        pair_ = createJointedPair(*world_, degToRad(-30.0f), degToRad(30.0f));
        ASSERT_TRUE(pair_.anchor.isValid());
        ASSERT_TRUE(pair_.box.isValid());
    }

    std::unique_ptr<PhysicsWorld> world_;
    JointedPair pair_;
};

// --- BodyDesc ------------------------------------------------------------------

TEST_F(JoltValidation, BaselineDescriptionsAreAccepted)
{
    // Proves that each rejection below is caused by the one field changed.
    EXPECT_TRUE(world_->createBody(BodyDesc{}).ok());
    EXPECT_TRUE(world_->createJoint(pair_.jointDesc).ok());
}

TEST_F(JoltValidation, DynamicBodyWithZeroMassIsRejected)
{
    BodyDesc desc;
    desc.massKg = 0.0f;
    EXPECT_TRUE(isRejected(world_->createBody(desc), ErrorCode::InvalidDescription, {"massKg"}));
}

TEST_F(JoltValidation, DynamicBodyWithNegativeMassIsRejected)
{
    BodyDesc desc;
    desc.massKg = -1.0f;
    EXPECT_TRUE(isRejected(world_->createBody(desc), ErrorCode::InvalidDescription, {"massKg"}));
}

TEST_F(JoltValidation, MassIsIgnoredForStaticAndKinematicBodies)
{
    // massKg is documented as "used for Dynamic bodies only".
    BodyDesc desc;
    desc.massKg = 0.0f;
    desc.motion = MotionType::Static;
    const auto staticBody = world_->createBody(desc);
    EXPECT_TRUE(staticBody.ok()) << staticBody.message;
    desc.motion = MotionType::Kinematic;
    const auto kinematicBody = world_->createBody(desc);
    EXPECT_TRUE(kinematicBody.ok()) << kinematicBody.message;
}

TEST_F(JoltValidation, BoxWithNonPositiveHalfExtentIsRejected)
{
    for (const Vec3 halfExtents :
         {Vec3{0.0f, 0.5f, 0.5f}, Vec3{0.5f, -0.1f, 0.5f}, Vec3{0.5f, 0.5f, 0.0f}}) {
        BodyDesc desc;
        desc.shape = BoxShape{halfExtents};
        EXPECT_TRUE(
            isRejected(world_->createBody(desc), ErrorCode::InvalidDescription, {"halfExtentsM"}))
            << "half extents " << halfExtents.x << ", " << halfExtents.y << ", " << halfExtents.z;
    }
}

TEST_F(JoltValidation, SphereWithNonPositiveRadiusIsRejected)
{
    for (const float radius : {0.0f, -0.5f}) {
        BodyDesc desc;
        desc.shape = SphereShape{radius};
        EXPECT_TRUE(
            isRejected(world_->createBody(desc), ErrorCode::InvalidDescription, {"radiusM"}))
            << "radius " << radius;
    }
}

TEST_F(JoltValidation, CapsuleWithNonPositiveRadiusIsRejected)
{
    for (const float radius : {0.0f, -0.25f}) {
        BodyDesc desc;
        desc.shape = CapsuleShape{0.5f, radius};
        EXPECT_TRUE(
            isRejected(world_->createBody(desc), ErrorCode::InvalidDescription, {"radiusM"}))
            << "radius " << radius;
    }
}

TEST_F(JoltValidation, CapsuleWithNonPositiveHalfHeightIsRejected)
{
    for (const float halfHeight : {0.0f, -0.5f}) {
        BodyDesc desc;
        desc.shape = CapsuleShape{halfHeight, 0.25f};
        EXPECT_TRUE(
            isRejected(world_->createBody(desc), ErrorCode::InvalidDescription, {"halfHeightM"}))
            << "half height " << halfHeight;
    }
}

// --- JointDesc -----------------------------------------------------------------

TEST_F(JoltValidation, JointWithNonUnitTwistAxisIsRejected)
{
    for (const Vec3 axis : {Vec3{2.0f, 0.0f, 0.0f}, Vec3{0.5f, 0.0f, 0.0f}, Vec3{}}) {
        JointDesc desc = pair_.jointDesc;
        desc.twistAxis = axis;
        EXPECT_TRUE(
            isRejected(world_->createJoint(desc), ErrorCode::InvalidDescription, {"twistAxis"}))
            << "twist axis " << axis.x << ", " << axis.y << ", " << axis.z;
    }
}

TEST_F(JoltValidation, JointWithNonUnitPlaneAxisIsRejected)
{
    for (const Vec3 axis : {Vec3{0.0f, 2.0f, 0.0f}, Vec3{0.0f, 0.5f, 0.0f}, Vec3{}}) {
        JointDesc desc = pair_.jointDesc;
        desc.planeAxis = axis;
        EXPECT_TRUE(
            isRejected(world_->createJoint(desc), ErrorCode::InvalidDescription, {"planeAxis"}))
            << "plane axis " << axis.x << ", " << axis.y << ", " << axis.z;
    }
}

TEST_F(JoltValidation, JointWithNonPerpendicularAxesIsRejected)
{
    // Both axes are unit length; only their 45 degree angle is wrong. Either
    // field name is acceptable since the condition involves both.
    const float invSqrt2 = 0.70710678f;
    JointDesc desc = pair_.jointDesc;
    desc.planeAxis = Vec3{invSqrt2, invSqrt2, 0.0f};
    EXPECT_TRUE(isRejected(world_->createJoint(desc), ErrorCode::InvalidDescription,
                           {"planeAxis", "twistAxis"}));
}

TEST_F(JoltValidation, JointWithInvalidBodyAHandleIsRejected)
{
    JointDesc desc = pair_.jointDesc;
    desc.bodyA = BodyId{};
    EXPECT_TRUE(isRejected(world_->createJoint(desc), ErrorCode::InvalidHandle, {"bodyA"}));
}

TEST_F(JoltValidation, JointWithDestroyedBodyBHandleIsRejected)
{
    const BodyId doomed = world_->createBody(BodyDesc{}).id;
    ASSERT_TRUE(world_->destroyBody(doomed));
    JointDesc desc = pair_.jointDesc;
    desc.bodyB = doomed;
    EXPECT_TRUE(isRejected(world_->createJoint(desc), ErrorCode::InvalidHandle, {"bodyB"}));
}

TEST_F(JoltValidation, JointWithTwistMinAboveTwistMaxIsRejected)
{
    JointDesc desc = pair_.jointDesc;
    desc.twistMinRad = degToRad(20.0f);
    desc.twistMaxRad = degToRad(-20.0f);
    EXPECT_TRUE(isRejected(world_->createJoint(desc), ErrorCode::InvalidDescription,
                           {"twistMinRad", "twistMaxRad"}));
}

// --- Capacity ------------------------------------------------------------------

TEST(JoltCapacity, CreatingMoreBodiesThanMaxBodiesFails)
{
    WorldConfig config;
    config.maxBodies = 2;
    const auto world = makeWorld(config);
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(world->createBody(BodyDesc{}).ok());
    ASSERT_TRUE(world->createBody(BodyDesc{}).ok());

    const auto third = world->createBody(BodyDesc{});
    EXPECT_EQ(third.error, ErrorCode::CapacityExceeded);
    EXPECT_FALSE(third.id.isValid());
}

// --- WorldConfig ---------------------------------------------------------------

void expectWorldRejected(const WorldConfig& config, std::string_view field)
{
    std::string error;
    EXPECT_EQ(opas::physics::jolt::createJoltWorld(config, &error), nullptr);
    EXPECT_NE(error.find(field), std::string::npos) << "message: \"" << error << "\"";
    // Passing no error string must also be safe.
    EXPECT_EQ(opas::physics::jolt::createJoltWorld(config), nullptr);
}

TEST(JoltWorldConfig, NonPositiveFixedStepIsRejected)
{
    for (const float stepSeconds : {0.0f, -1.0f / 60.0f}) {
        WorldConfig config;
        config.fixedStepSeconds = stepSeconds;
        expectWorldRejected(config, "fixedStepSeconds");
    }
}

TEST(JoltWorldConfig, ZeroMaxBodiesIsRejected)
{
    WorldConfig config;
    config.maxBodies = 0;
    expectWorldRejected(config, "maxBodies");
}

TEST(JoltWorldConfig, ZeroMaxJointsIsRejected)
{
    WorldConfig config;
    config.maxJoints = 0;
    expectWorldRejected(config, "maxJoints");
}

TEST(JoltWorldConfig, ValidConfigCreatesWorldWithoutError)
{
    std::string error;
    WorldConfig config;
    config.fixedStepSeconds = 1.0f / 120.0f;
    const auto world = opas::physics::jolt::createJoltWorld(config, &error);
    ASSERT_NE(world, nullptr) << error;
    EXPECT_EQ(world->fixedStepSeconds(), config.fixedStepSeconds);
}

} // namespace
