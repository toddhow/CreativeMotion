#include "opas/math/Transform.hpp"
#include "TestSupport.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using opas::math::fromAxisAngle;
using opas::math::inverse;
using opas::math::length;
using opas::math::Quat;
using opas::math::rotate;
using opas::math::Transform;
using opas::math::transformPoint;
using opas::math::Vec3;
using opas::test::kPi;
using opas::test::QuatSameRotation;
using opas::test::Vec3Near;

constexpr double kTol = 1e-12;
constexpr Vec3 kX{1.0, 0.0, 0.0};
constexpr Vec3 kZ{0.0, 0.0, 1.0};

TEST(MathTransform, DefaultIsIdentity)
{
    const Transform t;
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(t, Vec3{1.0, 2.0, 3.0}), (Vec3{1.0, 2.0, 3.0}),
                        kTol);
}

TEST(MathTransform, TransformPointRotatesThenTranslates)
{
    // Rz(90): (1,0,0) -> (0,1,0); then + (1,2,3) = (1,3,3)
    const Transform t{Vec3{1.0, 2.0, 3.0}, fromAxisAngle(kZ, kPi / 2.0)};
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(t, kX), (Vec3{1.0, 3.0, 3.0}), kTol);
    // The origin lands on the translation.
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(t, Vec3{}), (Vec3{1.0, 2.0, 3.0}), kTol);
}

TEST(MathTransform, ComposeMatchesHandComputedResult)
{
    const Transform parent{Vec3{1.0, 2.0, 3.0}, fromAxisAngle(kZ, kPi / 2.0)};
    const Transform child{Vec3{1.0, 0.0, 0.0}, fromAxisAngle(kX, kPi / 2.0)};
    const Transform world = parent * child;

    // Translation: Rz90 * (1,0,0) + (1,2,3) = (0,1,0) + (1,2,3) = (1,3,3)
    EXPECT_PRED_FORMAT3(Vec3Near, world.translation, (Vec3{1.0, 3.0, 3.0}), kTol);
    // Rotation is Rz90 * Rx90: x -> y, y -> z (see MathQuat.MultiplyAppliesTheRightFactorFirst).
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world.rotation, kX), (Vec3{0.0, 1.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(world.rotation, Vec3{0.0, 1.0, 0.0}), kZ, kTol);
    EXPECT_NEAR(length(world.rotation), 1.0, kTol);

    // Child point (0.5,-1,2): Rx90 -> (0.5,-2,-1), + (1,0,0) -> (1.5,-2,-1);
    // parent: Rz90 -> (2,1.5,-1), + (1,2,3) -> (3,3.5,2).
    const Vec3 p{0.5, -1.0, 2.0};
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(world, p), (Vec3{3.0, 3.5, 2.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(world, p),
                        transformPoint(parent, transformPoint(child, p)), kTol);
}

TEST(MathTransform, ComposeWithIdentityChangesNothing)
{
    const Transform t{Vec3{-1.0, 0.5, 2.0}, fromAxisAngle(Vec3{0.0, 1.0, 0.0}, 0.8)};
    const Vec3 p{0.3, 0.2, -0.9};
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(Transform{} * t, p), transformPoint(t, p), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(t * Transform{}, p), transformPoint(t, p), kTol);
}

TEST(MathTransform, InverseMatchesHandComputedResult)
{
    // t = Rz90 then +(1,2,3). Inverse translation = Rz(-90) * (-1,-2,-3) = (-2, 1, -3).
    const Transform t{Vec3{1.0, 2.0, 3.0}, fromAxisAngle(kZ, kPi / 2.0)};
    const Transform inv = inverse(t);
    EXPECT_PRED_FORMAT3(Vec3Near, inv.translation, (Vec3{-2.0, 1.0, -3.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(inv, Vec3{1.0, 3.0, 3.0}), kX, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(inv, Vec3{1.0, 2.0, 3.0}), Vec3{}, kTol);
}

TEST(MathTransform, InverseUndoesTransformPointAndComposesToIdentity)
{
    const Vec3 axis = Vec3{1.0, 1.0, 0.0} / std::sqrt(2.0);
    const Transform t{Vec3{0.7, -1.3, 2.2}, fromAxisAngle(axis, 2.1)};
    const Vec3 points[] = {Vec3{}, kX, Vec3{1.0, 2.0, 3.0}, Vec3{-4.0, 0.5, 0.25}};
    for (const Vec3& p : points) {
        SCOPED_TRACE(opas::test::str(p));
        EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(inverse(t), transformPoint(t, p)), p, kTol);
        EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(t * inverse(t), p), p, kTol);
        EXPECT_PRED_FORMAT3(Vec3Near, transformPoint(inverse(t) * t, p), p, kTol);
    }
    EXPECT_PRED_FORMAT3(QuatSameRotation, (t * inverse(t)).rotation, Quat{}, kTol);
}

} // namespace
