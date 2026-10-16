#include "opas/ik/TwoBoneIk.hpp"
#include "TestSupport.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using opas::ik::HingeLimits;
using opas::ik::solveTwoBone;
using opas::ik::TwoBoneChain;
using opas::ik::TwoBoneResult;
using opas::math::cross;
using opas::math::distance;
using opas::math::dot;
using opas::math::length;
using opas::math::normalized;
using opas::math::Vec3;
using opas::test::isFinite;
using opas::test::kPi;
using opas::test::Vec3Near;

constexpr double kTol = 1e-9;

// Angle between the upper and lower bone directions; 0 = straight. atan2 keeps
// full precision near 0 and pi, where acos would not.
double measuredBend(Vec3 root, Vec3 mid, Vec3 end)
{
    const Vec3 upper = mid - root;
    const Vec3 lower = end - mid;
    return std::atan2(length(cross(upper, lower)), dot(upper, lower));
}

void expectFinite(const TwoBoneResult& r)
{
    EXPECT_TRUE(isFinite(r.mid)) << opas::test::str(r.mid);
    EXPECT_TRUE(isFinite(r.end)) << opas::test::str(r.end);
    EXPECT_TRUE(std::isfinite(r.bendRad));
    EXPECT_TRUE(isFinite(r.rootRotation));
    EXPECT_TRUE(isFinite(r.midRotation));
}

// ---- Hand-computed cases -----------------------------------------------------

TEST(IkTwoBone, StraightArmReachWhenTargetIsAtFullExtension)
{
    // Two unit bones, bent now; the target is exactly 2 m away, so the arm must straighten.
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    const TwoBoneResult r = solveTwoBone(chain, Vec3{2.0, 0.0, 0.0}, Vec3{0.0, 0.0, 1.0}, {});
    ASSERT_TRUE(r.valid);
    EXPECT_TRUE(r.reached);
    EXPECT_PRED_FORMAT3(Vec3Near, r.end, (Vec3{2.0, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, r.mid, (Vec3{1.0, 0.0, 0.0}), kTol);
    EXPECT_NEAR(r.bendRad, 0.0, kTol);
}

TEST(IkTwoBone, TargetBeyondFullExtensionClampsToStraightArm)
{
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    const TwoBoneResult r = solveTwoBone(chain, Vec3{3.0, 0.0, 0.0}, Vec3{0.0, 0.0, 1.0}, {});
    ASSERT_TRUE(r.valid);
    EXPECT_FALSE(r.reached);
    EXPECT_PRED_FORMAT3(Vec3Near, r.end, (Vec3{2.0, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, r.mid, (Vec3{1.0, 0.0, 0.0}), kTol);
    EXPECT_NEAR(r.bendRad, 0.0, kTol);
}

TEST(IkTwoBone, NinetyDegreeBendWithEqualBones)
{
    // Unit bones, target at distance sqrt(2): 2 + 2cos(theta) = 2, so theta = 90 degrees.
    // The mid joint is 1 m from both root and target: (1,0,0) or (0,1,0). The pole picks.
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0}};
    const Vec3 target{1.0, 1.0, 0.0};

    const TwoBoneResult towardPlusX = solveTwoBone(chain, target, Vec3{1.0, -1.0, 0.0}, {});
    ASSERT_TRUE(towardPlusX.valid);
    EXPECT_TRUE(towardPlusX.reached);
    EXPECT_PRED_FORMAT3(Vec3Near, towardPlusX.end, target, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, towardPlusX.mid, (Vec3{1.0, 0.0, 0.0}), kTol);
    EXPECT_NEAR(towardPlusX.bendRad, kPi / 2.0, kTol);

    const TwoBoneResult towardPlusY = solveTwoBone(chain, target, Vec3{-1.0, 1.0, 0.0}, {});
    ASSERT_TRUE(towardPlusY.valid);
    EXPECT_PRED_FORMAT3(Vec3Near, towardPlusY.end, target, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, towardPlusY.mid, (Vec3{0.0, 1.0, 0.0}), kTol);
}

TEST(IkTwoBone, PoleIsOnlyUsedForItsSidePerpendicularToTheTargetDirection)
{
    // Same 90 degree case, pole tilted out of the XY plane. The bend plane contains root,
    // target and pole: the mid joint is the segment midpoint plus h along the pole's
    // perpendicular part, with h = sqrt(1 - 0.5) = sqrt(0.5).
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0}};
    const Vec3 target{1.0, 1.0, 0.0};
    const Vec3 pole{1.0, -1.0, 5.0}; // already perpendicular to the target direction (1,1,0)
    const Vec3 expectedMid = Vec3{0.5, 0.5, 0.0} + normalized(pole) * std::sqrt(0.5);
    const TwoBoneResult r = solveTwoBone(chain, target, pole, {});
    ASSERT_TRUE(r.valid);
    EXPECT_PRED_FORMAT3(Vec3Near, r.end, target, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, r.mid, expectedMid, kTol);
}

TEST(IkTwoBone, ThreeFourFiveTriangleGivesNinetyDegreeBendWithUnequalBones)
{
    // Bones 3 and 4, target 5 m away: 9 + 16 + 24cos(theta) = 25, so theta = 90 degrees.
    // Mid at x = (9 + 25 - 16) / 10 = 1.8, y = sqrt(9 - 3.24) = 2.4 on the pole (+Y) side.
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{3.0, 0.0, 0.0}, Vec3{7.0, 0.0, 0.0}};
    const TwoBoneResult r = solveTwoBone(chain, Vec3{5.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, {});
    ASSERT_TRUE(r.valid);
    EXPECT_TRUE(r.reached);
    EXPECT_PRED_FORMAT3(Vec3Near, r.end, (Vec3{5.0, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, r.mid, (Vec3{1.8, 2.4, 0.0}), kTol);
    EXPECT_NEAR(r.bendRad, kPi / 2.0, kTol);

    // The rotations take the original directions (3,0,0) and (4,0,0) to the solved ones.
    EXPECT_PRED_FORMAT3(Vec3Near, opas::math::rotate(r.rootRotation, Vec3{3.0, 0.0, 0.0}),
                        (Vec3{1.8, 2.4, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, opas::math::rotate(r.midRotation, Vec3{4.0, 0.0, 0.0}),
                        (Vec3{3.2, -2.4, 0.0}), kTol);
}

TEST(IkTwoBone, HingeLimitsRestrictTheBend)
{
    // Bones 3 and 4, but the elbow may only bend 0..45 degrees. The reachable distances are
    // then sqrt(25 + 24cos(45 deg)) = 6.48 .. 7 m, so a target 5 m away is too close: the end
    // stops at the nearest reachable distance, 6.48 m, along the ray to the target.
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{3.0, 0.0, 0.0}, Vec3{7.0, 0.0, 0.0}};
    const HingeLimits limits{0.0, kPi / 4.0};
    const double nearest = std::sqrt(25.0 + 24.0 * std::cos(kPi / 4.0));
    ASSERT_GT(nearest, 5.0);
    const TwoBoneResult r = solveTwoBone(chain, Vec3{0.0, 5.0, 0.0}, Vec3{1.0, 0.0, 0.0}, limits);
    ASSERT_TRUE(r.valid);
    EXPECT_FALSE(r.reached);
    EXPECT_PRED_FORMAT3(Vec3Near, r.end, (Vec3{0.0, nearest, 0.0}), kTol);
    EXPECT_NEAR(r.bendRad, kPi / 4.0, kTol);
    EXPECT_NEAR(distance(r.mid, Vec3{}), 3.0, kTol);
    EXPECT_NEAR(distance(r.end, r.mid), 4.0, kTol);
    // Pole side: +X.
    EXPECT_GT(r.mid.x, 0.0);
}

TEST(IkTwoBone, TargetAtTheRootUsesTheCurrentEndDirection)
{
    // The bend may be 0..90 degrees, so the end can never be closer than sqrt(2) to the
    // root. With the target on the root there is no direction to the target; the current
    // root -> end direction (+X) is used, so the end goes to (sqrt 2, 0, 0).
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0}};
    const HingeLimits limits{0.0, kPi / 2.0};
    const TwoBoneResult r = solveTwoBone(chain, Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, limits);
    ASSERT_TRUE(r.valid);
    expectFinite(r);
    EXPECT_FALSE(r.reached);
    const double root2 = std::sqrt(2.0);
    EXPECT_PRED_FORMAT3(Vec3Near, r.end, (Vec3{root2, 0.0, 0.0}), kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, r.mid, (Vec3{root2 / 2.0, root2 / 2.0, 0.0}), kTol);
    EXPECT_NEAR(r.bendRad, kPi / 2.0, kTol);
}

TEST(IkTwoBone, TargetAtTheRootWithFullFoldingStaysFinite)
{
    // Full range of bend and equal bones: the end can sit exactly on the root.
    const TwoBoneChain chain{Vec3{1.0, 2.0, 3.0}, Vec3{2.0, 2.0, 3.0}, Vec3{2.0, 3.0, 3.0}};
    const TwoBoneResult r = solveTwoBone(chain, Vec3{1.0, 2.0, 3.0}, Vec3{0.0, 0.0, 1.0}, {});
    ASSERT_TRUE(r.valid);
    expectFinite(r);
    EXPECT_NEAR(distance(r.mid, chain.root), 1.0, kTol);
    EXPECT_NEAR(distance(r.end, r.mid), 1.0, kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, r.end, chain.root, kTol);
}

TEST(IkTwoBone, PoleParallelToTargetDirectionFallsBackToTheCurrentBendPlane)
{
    // The chain bends in the XY plane, with the mid joint on the +Y side. The target is along
    // +X at sqrt(2) (a 90 degree bend), and the pole is exactly along the target direction, so
    // the pole gives no side. The plane of the current chain (XY) is used and the mid joint
    // stays on its current (+Y) side.
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    const Vec3 target{std::sqrt(2.0), 0.0, 0.0};
    const double half = std::sqrt(2.0) / 2.0;

    for (const Vec3& pole : {Vec3{1.0, 0.0, 0.0}, Vec3{-3.0, 0.0, 0.0}}) {
        SCOPED_TRACE(opas::test::str(pole));
        const TwoBoneResult r = solveTwoBone(chain, target, pole, {});
        ASSERT_TRUE(r.valid);
        expectFinite(r);
        EXPECT_TRUE(r.reached);
        EXPECT_PRED_FORMAT3(Vec3Near, r.end, target, kTol);
        EXPECT_PRED_FORMAT3(Vec3Near, r.mid, (Vec3{half, half, 0.0}), kTol);
        EXPECT_NEAR(r.bendRad, kPi / 2.0, kTol);
    }
}

TEST(IkTwoBone, PoleParallelFallbackIsDeterministic)
{
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    const Vec3 target{std::sqrt(2.0), 0.0, 0.0};
    const TwoBoneResult first = solveTwoBone(chain, target, Vec3{1.0, 0.0, 0.0}, {});
    const TwoBoneResult second = solveTwoBone(chain, target, Vec3{1.0, 0.0, 0.0}, {});
    EXPECT_EQ(first.mid, second.mid);
    EXPECT_EQ(first.end, second.end);
    EXPECT_EQ(first.rootRotation, second.rootRotation);
    EXPECT_EQ(first.midRotation, second.midRotation);
}

TEST(IkTwoBone, PoleParallelWithStraightCurrentChainUsesAFixedPerpendicular)
{
    // The current chain is a straight line, so it defines no plane either. Whatever fixed
    // perpendicular is chosen, the result must be finite, deterministic, bent (the target
    // needs a bend), and still reach the target.
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{2.0, 0.0, 0.0}};
    const Vec3 target{0.0, 0.0, std::sqrt(2.0)};
    const Vec3 pole{0.0, 0.0, 1.0};
    const TwoBoneResult first = solveTwoBone(chain, target, pole, {});
    const TwoBoneResult second = solveTwoBone(chain, target, pole, {});
    ASSERT_TRUE(first.valid);
    expectFinite(first);
    EXPECT_TRUE(first.reached);
    EXPECT_PRED_FORMAT3(Vec3Near, first.end, target, kTol);
    EXPECT_NEAR(distance(first.mid, chain.root), 1.0, kTol);
    EXPECT_NEAR(distance(first.end, first.mid), 1.0, kTol);
    EXPECT_NEAR(measuredBend(chain.root, first.mid, first.end), kPi / 2.0, kTol);
    // Perpendicular to the root -> target axis.
    EXPECT_NEAR(dot(first.mid - chain.root, normalized(target - chain.root)), std::sqrt(2.0) / 2.0,
                kTol);
    EXPECT_EQ(first.mid, second.mid);
    EXPECT_EQ(first.end, second.end);
}

// ---- Rotations -----------------------------------------------------------------

TEST(IkTwoBone, RotationsMapOriginalBoneDirectionsToSolvedOnes)
{
    const TwoBoneChain chain{Vec3{0.5, -0.25, 1.0}, Vec3{0.5, 0.35, 1.0}, Vec3{0.9, 0.35, 1.3}};
    const Vec3 target{0.1, 0.8, 1.9};
    const TwoBoneResult r = solveTwoBone(chain, target, Vec3{-1.0, 0.2, 0.3}, {});
    ASSERT_TRUE(r.valid);

    const Vec3 upperBefore = normalized(chain.mid - chain.root);
    const Vec3 lowerBefore = normalized(chain.end - chain.mid);
    const Vec3 upperAfter = normalized(r.mid - chain.root);
    const Vec3 lowerAfter = normalized(r.end - r.mid);
    EXPECT_PRED_FORMAT3(Vec3Near, opas::math::rotate(r.rootRotation, upperBefore), upperAfter,
                        kTol);
    EXPECT_PRED_FORMAT3(Vec3Near, opas::math::rotate(r.midRotation, lowerBefore), lowerAfter, kTol);
}

// ---- Invalid input -------------------------------------------------------------

void expectRejectedAndUnchanged(const TwoBoneChain& chain, const HingeLimits& limits)
{
    const TwoBoneResult r = solveTwoBone(chain, Vec3{0.5, 0.5, 0.0}, Vec3{0.0, 0.0, 1.0}, limits);
    EXPECT_FALSE(r.valid);
    EXPECT_EQ(r.mid, chain.mid);
    EXPECT_EQ(r.end, chain.end);
}

TEST(IkTwoBone, ZeroLengthUpperBoneIsInvalid)
{
    expectRejectedAndUnchanged(
        TwoBoneChain{Vec3{1.0, 1.0, 1.0}, Vec3{1.0, 1.0, 1.0}, Vec3{2.0, 1.0, 1.0}}, {});
}

TEST(IkTwoBone, ZeroLengthLowerBoneIsInvalid)
{
    expectRejectedAndUnchanged(
        TwoBoneChain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}}, {});
}

TEST(IkTwoBone, LimitsOutsideZeroToPiAreInvalid)
{
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    SCOPED_TRACE("min below 0");
    expectRejectedAndUnchanged(chain, HingeLimits{-0.1, 2.0});
    SCOPED_TRACE("max above pi");
    expectRejectedAndUnchanged(chain, HingeLimits{0.5, 3.2});
    SCOPED_TRACE("both outside");
    expectRejectedAndUnchanged(chain, HingeLimits{-1.0, 4.0});
}

TEST(IkTwoBone, MinGreaterThanMaxIsInvalid)
{
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    expectRejectedAndUnchanged(chain, HingeLimits{2.0, 1.0});
}

TEST(IkTwoBone, NonFiniteLimitsAreInvalid)
{
    // Test choice: NaN is not "within [0, pi]", so it is rejected like any other bad limit.
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    expectRejectedAndUnchanged(chain, HingeLimits{nan, 2.0});
    expectRejectedAndUnchanged(chain, HingeLimits{0.0, nan});
}

TEST(IkTwoBone, LimitsAtTheEdgesOfTheValidRangeAreAccepted)
{
    const TwoBoneChain chain{Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, Vec3{1.0, 1.0, 0.0}};
    EXPECT_TRUE(solveTwoBone(chain, Vec3{1.5, 0.5, 0.0}, Vec3{0.0, 0.0, 1.0}, {0.0, kPi}).valid);
    // min == max pins the bend to one angle and is allowed.
    EXPECT_TRUE(solveTwoBone(chain, Vec3{1.5, 0.5, 0.0}, Vec3{0.0, 0.0, 1.0}, {1.0, 1.0}).valid);
}

} // namespace
