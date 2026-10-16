#include "opas/math/Quat.hpp"
#include "TestSupport.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using opas::math::angleBetween;
using opas::math::conjugate;
using opas::math::dot;
using opas::math::fromAxisAngle;
using opas::math::fromTo;
using opas::math::length;
using opas::math::normalized;
using opas::math::Quat;
using opas::math::rotate;
using opas::math::Vec3;
using opas::test::kPi;
using opas::test::QuatSameRotation;
using opas::test::Vec3Near;

constexpr double kTol = 1e-12;
constexpr Vec3 kX{1.0, 0.0, 0.0};
constexpr Vec3 kY{0.0, 1.0, 0.0};
constexpr Vec3 kZ{0.0, 0.0, 1.0};

TEST(MathQuat, DefaultConstructedIsIdentity)
{
    const Quat q;
    EXPECT_EQ(q.x, 0.0);
    EXPECT_EQ(q.y, 0.0);
    EXPECT_EQ(q.z, 0.0);
    EXPECT_EQ(q.w, 1.0);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, Vec3{1.0, 2.0, 3.0}), (Vec3{1.0, 2.0, 3.0}), kTol);
}

TEST(MathQuat, FromAxisAngleComponents)
{
    // 90 degrees about Z: (0, 0, sin 45, cos 45)
    const Quat q = fromAxisAngle(kZ, kPi / 2.0);
    const double s = std::sqrt(0.5);
    EXPECT_NEAR(q.x, 0.0, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
    EXPECT_NEAR(q.z, s, kTol);
    EXPECT_NEAR(q.w, s, kTol);

    // Half turn about X: (1, 0, 0, 0)
    const Quat half = fromAxisAngle(kX, kPi);
    EXPECT_NEAR(half.x, 1.0, kTol);
    EXPECT_NEAR(half.w, 0.0, kTol);

    // Zero angle is the identity.
    const Quat none = fromAxisAngle(kY, 0.0);
    EXPECT_NEAR(none.w, 1.0, kTol);
    EXPECT_NEAR(length(none), 1.0, kTol);
}

TEST(MathQuat, RotateFollowsRightHandRule)
{
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(fromAxisAngle(kZ, kPi / 2.0), kX), kY, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(fromAxisAngle(kX, kPi / 2.0), kY), kZ, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(fromAxisAngle(kY, kPi / 2.0), kZ), kX, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(fromAxisAngle(kZ, kPi), kX), (Vec3{-1.0, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(fromAxisAngle(kZ, -kPi / 2.0), kX), (Vec3{0.0, -1.0, 0.0}),
                        kTol);
}

TEST(MathQuat, RotateLeavesTheAxisAndLengthsUnchanged)
{
    const Quat q = fromAxisAngle(kZ, 1.234);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, Vec3{0.0, 0.0, 5.0}), (Vec3{0.0, 0.0, 5.0}), kTol);
    EXPECT_NEAR(length(rotate(q, Vec3{1.0, 2.0, 3.0})), std::sqrt(14.0), kTol);
}

TEST(MathQuat, RotationAboutDiagonalAxisByOneThirdTurnCyclesTheAxes)
{
    // 120 degrees about (1,1,1)/sqrt(3) sends x -> y -> z -> x.
    const Vec3 axis = normalized(Vec3{1.0, 1.0, 1.0});
    const Quat q = fromAxisAngle(axis, 2.0 * kPi / 3.0);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, kX), kY, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, kY), kZ, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, kZ), kX, kTol);
}

TEST(MathQuat, MultiplyHamiltonProductOfBasisQuaternions)
{
    const Quat i{1.0, 0.0, 0.0, 0.0};
    const Quat j{0.0, 1.0, 0.0, 0.0};
    const Quat ij = i * j; // i*j = k
    EXPECT_NEAR(ij.x, 0.0, kTol);
    EXPECT_NEAR(ij.y, 0.0, kTol);
    EXPECT_NEAR(ij.z, 1.0, kTol);
    EXPECT_NEAR(ij.w, 0.0, kTol);
    const Quat ji = j * i; // j*i = -k
    EXPECT_NEAR(ji.z, -1.0, kTol);
    const Quat ii = i * i; // i*i = -1
    EXPECT_NEAR(ii.w, -1.0, kTol);
    EXPECT_NEAR(ii.x, 0.0, kTol);
}

TEST(MathQuat, MultiplyByIdentityChangesNothing)
{
    const Quat q = normalized(Quat{0.1, -0.2, 0.3, 0.9});
    const Quat left = Quat{} * q;
    const Quat right = q * Quat{};
    EXPECT_PRED_FORMAT3(QuatSameRotation, left, q, kTol);
    EXPECT_PRED_FORMAT3(QuatSameRotation, right, q, kTol);
}

TEST(MathQuat, MultiplyAppliesTheRightFactorFirst)
{
    const Quat rz = fromAxisAngle(kZ, kPi / 2.0);
    const Quat rx = fromAxisAngle(kX, kPi / 2.0);

    // (rz * rx): rx first, then rz.
    //   x: rx leaves x, rz -> y;  y: rx -> z, rz leaves z.
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(rz * rx, kX), kY, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(rz * rx, kY), kZ, kTol);

    // (rx * rz): rz first, then rx.
    //   x: rz -> y, rx -> z;  y: rz -> -x, rx leaves x.
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(rx * rz, kX), kZ, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(rx * rz, kY), (Vec3{-1.0, 0.0, 0.0}), kTol);
}

TEST(MathQuat, MultiplyComposesRotationsLikeNestedRotate)
{
    const Quat a = fromAxisAngle(normalized(Vec3{1.0, 2.0, -1.0}), 0.7);
    const Quat b = fromAxisAngle(normalized(Vec3{-0.3, 0.5, 1.0}), -1.9);
    const Vec3 v{0.4, -1.2, 2.5};
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(a * b, v), rotate(a, rotate(b, v)), kTol);
}

TEST(MathQuat, ConjugateNegatesTheVectorPart)
{
    const Quat c = conjugate(Quat{1.0, 2.0, 3.0, 4.0});
    EXPECT_EQ(c.x, -1.0);
    EXPECT_EQ(c.y, -2.0);
    EXPECT_EQ(c.z, -3.0);
    EXPECT_EQ(c.w, 4.0);
}

TEST(MathQuat, ConjugateOfUnitQuaternionUndoesTheRotation)
{
    const Quat q = fromAxisAngle(normalized(Vec3{2.0, -1.0, 0.5}), 2.2);
    const Vec3 v{1.0, 2.0, 3.0};
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(conjugate(q), rotate(q, v)), v, kTol);
    EXPECT_PRED_FORMAT3(QuatSameRotation, conjugate(q) * q, Quat{}, kTol);
}

TEST(MathQuat, NormalizedGivesUnitLengthAndZeroBecomesIdentity)
{
    // (0, 0, 3, 4) has length 5.
    const Quat q = normalized(Quat{0.0, 0.0, 3.0, 4.0});
    EXPECT_NEAR(q.z, 0.6, kTol);
    EXPECT_NEAR(q.w, 0.8, kTol);
    EXPECT_NEAR(length(q), 1.0, kTol);

    const Quat zero = normalized(Quat{0.0, 0.0, 0.0, 0.0});
    EXPECT_EQ(zero.x, 0.0);
    EXPECT_EQ(zero.y, 0.0);
    EXPECT_EQ(zero.z, 0.0);
    EXPECT_EQ(zero.w, 1.0);
}

TEST(MathQuat, AngleBetweenKnownRotations)
{
    EXPECT_NEAR(angleBetween(Quat{}, Quat{}), 0.0, kTol);
    EXPECT_NEAR(angleBetween(Quat{}, fromAxisAngle(kZ, 1.0)), 1.0, kTol);
    EXPECT_NEAR(angleBetween(fromAxisAngle(kX, 0.3), fromAxisAngle(kX, 1.1)), 0.8, kTol);
    EXPECT_NEAR(angleBetween(Quat{}, fromAxisAngle(kY, kPi)), kPi, kTol);
    // Symmetric.
    EXPECT_NEAR(angleBetween(fromAxisAngle(kX, 1.1), fromAxisAngle(kX, 0.3)), 0.8, kTol);
}

TEST(MathQuat, AngleBetweenTakesTheShorterPath)
{
    // Three quarters of a turn is the same rotation as a quarter turn the other way.
    EXPECT_NEAR(angleBetween(Quat{}, fromAxisAngle(kZ, 1.5 * kPi)), kPi / 2.0, kTol);
    // q and -q are one rotation.
    const Quat q = fromAxisAngle(kY, 0.9);
    const Quat negated{-q.x, -q.y, -q.z, -q.w};
    EXPECT_NEAR(angleBetween(q, negated), 0.0, kTol);
    EXPECT_NEAR(angleBetween(Quat{}, Quat{0.0, 0.0, 0.0, -1.0}), 0.0, kTol);
    EXPECT_NEAR(angleBetween(Quat{}, negated), 0.9, kTol);
}

TEST(MathQuat, FromToRotatesFromOntoTo)
{
    const Quat q = fromTo(kX, kY);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, kX), kY, kTol);
    EXPECT_NEAR(length(q), 1.0, kTol);
    // Shortest arc: 90 degrees, so it matches the Z quarter turn.
    EXPECT_PRED_FORMAT3(QuatSameRotation, q, fromAxisAngle(kZ, kPi / 2.0), kTol);
}

TEST(MathQuat, FromToAcceptsNonUnitInputs)
{
    const Vec3 from{1.0, 2.0, 3.0};
    const Vec3 to{-2.0, 0.5, 1.0};
    const Quat q = fromTo(from, to);
    EXPECT_NEAR(length(q), 1.0, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, normalized(from)), normalized(to), kTol);

    EXPECT_PRED_FORMAT3(Vec3Near, rotate(fromTo(Vec3{0.0, 0.0, 4.0}, Vec3{0.0, 7.0, 0.0}), kZ), kY,
                        kTol);
}

TEST(MathQuat, FromToSameDirectionIsIdentity)
{
    EXPECT_PRED_FORMAT3(QuatSameRotation, fromTo(Vec3{0.0, 2.0, 0.0}, Vec3{0.0, 5.0, 0.0}), Quat{},
                        kTol);
}

TEST(MathQuat, FromToOppositeVectorsIsHalfTurnAboutAPerpendicularAxis)
{
    const Vec3 directions[] = {kX,
                               Vec3{0.0, -1.0, 0.0},
                               kZ,
                               Vec3{0.0, 0.0, -3.0},
                               Vec3{0.3, -0.4, 0.5},
                               Vec3{-1.0, 1.0, 1.0}};
    for (const Vec3& from : directions) {
        SCOPED_TRACE(opas::test::str(from));
        const Vec3 to = -from * 2.0; // opposite, different length
        const Quat q = fromTo(from, to);
        EXPECT_NEAR(length(q), 1.0, kTol);
        EXPECT_PRED_FORMAT3(Vec3Near, rotate(q, normalized(from)), normalized(to), kTol);
        // A half turn: the rotation angle is pi.
        EXPECT_NEAR(angleBetween(Quat{}, q), kPi, kTol);
        // The axis is perpendicular to `from`.
        const Vec3 axis{q.x, q.y, q.z};
        EXPECT_NEAR(dot(axis, normalized(from)), 0.0, kTol);
    }
}

TEST(MathQuat, FromToOppositeIsDeterministic)
{
    const Quat first = fromTo(Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0});
    const Quat second = fromTo(Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0});
    EXPECT_EQ(first, second);
}

TEST(MathQuat, FromToZeroInputGivesIdentity)
{
    // Documented in the header: no direction means no rotation.
    EXPECT_PRED_FORMAT3(QuatSameRotation, fromTo(Vec3{}, kX), Quat{}, kTol);
    EXPECT_PRED_FORMAT3(QuatSameRotation, fromTo(kX, Vec3{}), Quat{}, kTol);
}

} // namespace
