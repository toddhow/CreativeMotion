#include "opas/math/Vec3.hpp"
#include "TestSupport.hpp"

#include <gtest/gtest.h>

namespace {

using opas::math::cross;
using opas::math::dot;
using opas::math::length;
using opas::math::normalized;
using opas::math::Vec3;
using opas::test::Vec3Near;

constexpr double kTol = 1e-12;

TEST(MathVec3, DotProductOfKnownVectors)
{
    // 1*4 + 2*(-5) + 3*6 = 12
    EXPECT_NEAR(dot(Vec3{1.0, 2.0, 3.0}, Vec3{4.0, -5.0, 6.0}), 12.0, kTol);
    EXPECT_NEAR(dot(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}), 0.0, kTol);
    EXPECT_NEAR(dot(Vec3{0.5, 0.5, 0.5}, Vec3{0.5, 0.5, 0.5}), 0.75, kTol);
}

TEST(MathVec3, CrossProductOfKnownVectors)
{
    EXPECT_PRED_FORMAT3(Vec3Near, cross(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}),
                        (Vec3{0.0, 0.0, 1.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, cross(Vec3{0.0, 1.0, 0.0}, Vec3{1.0, 0.0, 0.0}),
                        (Vec3{0.0, 0.0, -1.0}), kTol);
    // (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
    EXPECT_PRED_FORMAT3(Vec3Near, cross(Vec3{1.0, 2.0, 3.0}, Vec3{4.0, 5.0, 6.0}),
                        (Vec3{-3.0, 6.0, -3.0}), kTol);
}

TEST(MathVec3, CrossProductIsPerpendicularToBothInputs)
{
    const Vec3 a{1.5, -2.0, 0.25};
    const Vec3 b{-0.5, 3.0, 2.0};
    const Vec3 c = cross(a, b);
    EXPECT_NEAR(dot(c, a), 0.0, kTol);
    EXPECT_NEAR(dot(c, b), 0.0, kTol);
}

TEST(MathVec3, LengthOfKnownVectors)
{
    EXPECT_NEAR(length(Vec3{3.0, 4.0, 12.0}), 13.0, kTol);
    EXPECT_NEAR(length(Vec3{0.0, -2.0, 0.0}), 2.0, kTol);
    EXPECT_EQ(length(Vec3{}), 0.0);
}

TEST(MathVec3, NormalizedGivesUnitVectorInSameDirection)
{
    EXPECT_PRED_FORMAT3(Vec3Near, normalized(Vec3{0.0, 3.0, 4.0}), (Vec3{0.0, 0.6, 0.8}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, normalized(Vec3{-10.0, 0.0, 0.0}), (Vec3{-1.0, 0.0, 0.0}), kTol);
    EXPECT_NEAR(length(normalized(Vec3{1.0, 2.0, 3.0})), 1.0, kTol);
}

TEST(MathVec3, NormalizedZeroVectorStaysZero)
{
    const Vec3 n = normalized(Vec3{});
    EXPECT_EQ(n.x, 0.0);
    EXPECT_EQ(n.y, 0.0);
    EXPECT_EQ(n.z, 0.0);
}

} // namespace
