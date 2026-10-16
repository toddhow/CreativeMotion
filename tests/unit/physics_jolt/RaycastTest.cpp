#include "PhysicsTestHelpers.hpp"

#include <gtest/gtest.h>

namespace {

using namespace opas::physics;
using namespace opas::physics::test;

TEST(JoltRaycast, RayStraightDownHitsFloorAtOriginWithUpNormal)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    const BodyId floor = createFloor(*world);
    ASSERT_TRUE(floor.isValid());

    const auto hit = world->castRay(Vec3{0.0f, 0.0f, 5.0f}, Vec3{0.0f, 0.0f, -10.0f});
    ASSERT_TRUE(hit.has_value());
    EXPECT_EQ(hit->body, floor);
    EXPECT_LE(distance(hit->pointM, Vec3{0.0f, 0.0f, 0.0f}), 1e-4);
    EXPECT_LE(distance(hit->normal, Vec3{0.0f, 0.0f, 1.0f}), 1e-4);
    // The floor is 5 m along a 10 m ray.
    EXPECT_NEAR(hit->fraction, 0.5f, 1e-4f);
}

TEST(JoltRaycast, RayPointingAwayFromFloorReturnsNoHit)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(createFloor(*world).isValid());

    EXPECT_FALSE(world->castRay(Vec3{0.0f, 0.0f, 5.0f}, Vec3{0.0f, 0.0f, 10.0f}).has_value());
}

TEST(JoltRaycast, RayTooShortToReachFloorReturnsNoHit)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    ASSERT_TRUE(createFloor(*world).isValid());

    EXPECT_FALSE(world->castRay(Vec3{0.0f, 0.0f, 5.0f}, Vec3{0.0f, 0.0f, -4.0f}).has_value());
}

TEST(JoltRaycast, IgnoredBodyIsSkippedAndFloorBehindItIsHit)
{
    const auto world = makeWorld();
    ASSERT_NE(world, nullptr);
    const BodyId floor = createFloor(*world);
    ASSERT_TRUE(floor.isValid());

    // Static so it cannot fall; its top face is at z = 2.5.
    BodyDesc blockerDesc;
    blockerDesc.shape = BoxShape{Vec3{0.5f, 0.5f, 0.5f}};
    blockerDesc.motion = MotionType::Static;
    blockerDesc.positionM = Vec3{0.0f, 0.0f, 2.0f};
    const BodyId blocker = world->createBody(blockerDesc).id;
    ASSERT_TRUE(blocker.isValid());

    const Vec3 origin{0.0f, 0.0f, 5.0f};
    const Vec3 ray{0.0f, 0.0f, -10.0f};

    const auto blocked = world->castRay(origin, ray);
    ASSERT_TRUE(blocked.has_value());
    EXPECT_EQ(blocked->body, blocker);
    EXPECT_LE(distance(blocked->pointM, Vec3{0.0f, 0.0f, 2.5f}), 1e-4);

    const auto skipped = world->castRay(origin, ray, blocker);
    ASSERT_TRUE(skipped.has_value());
    EXPECT_EQ(skipped->body, floor);
    EXPECT_LE(distance(skipped->pointM, Vec3{0.0f, 0.0f, 0.0f}), 1e-4);
}

} // namespace
