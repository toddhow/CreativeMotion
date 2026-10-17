#include "TestSupport.hpp"
#include "opas/ik/TwoBoneIk.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>

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
using opas::math::rotate;
using opas::math::Vec3;
using opas::test::isFinite;
using opas::test::kPi;
using opas::test::SeededRandom;
using opas::test::Vec3Near;

constexpr std::uint32_t kSeed = 7001U;
constexpr int kCases = 1200;
constexpr double kTol = 1e-9;

enum class Kind : std::uint8_t {
    Reachable, // distance inside [minReach, maxReach]
    TooFar,    // beyond the straight-arm reach
    TooNear,   // closer than the fully-bent reach
};

// Distance from the root to the end when the middle joint is bent by bendRad
// (law of cosines; bend 0 is a straight arm).
double reachAt(double upper, double lower, double bendRad)
{
    return std::sqrt(
        std::max(0.0, upper * upper + lower * lower + 2.0 * upper * lower * std::cos(bendRad)));
}

struct IkCase {
    std::uint32_t seed = 0;
    int index = 0;
    Kind kind = Kind::Reachable;
    TwoBoneChain chain;
    double upperM = 0.0;
    double lowerM = 0.0;
    HingeLimits limits;
    double minReachM = 0.0; // from the largest allowed bend
    double maxReachM = 0.0; // from the smallest allowed bend
    Vec3 targetDirection;
    double targetDistanceM = 0.0;
    Vec3 target;
    Vec3 pole;
};

std::string describe(const IkCase& c)
{
    std::ostringstream out;
    out.precision(17);
    out << "seed=" << c.seed << " case=" << c.index << " root=" << opas::test::str(c.chain.root)
        << " mid=" << opas::test::str(c.chain.mid) << " end=" << opas::test::str(c.chain.end)
        << " target=" << opas::test::str(c.target) << " pole=" << opas::test::str(c.pole)
        << " limits=[" << c.limits.minBendRad << ", " << c.limits.maxBendRad << "] reach=["
        << c.minReachM << ", " << c.maxReachM << "] distance=" << c.targetDistanceM;
    return out.str();
}

// Each call draws everything from `rng`, so the (seed, index) pair reproduces a case.
IkCase makeCase(SeededRandom& rng, std::uint32_t seed, int index, Kind wanted)
{
    IkCase c;
    c.seed = seed;
    c.index = index;

    c.upperM = rng.uniform(0.1, 1.0);
    c.lowerM = rng.uniform(0.1, 1.0);

    // Random limits within [0, pi]; the edges and min == max are drawn often on purpose.
    double lo = rng.uniform(0.0, kPi);
    double hi = rng.uniform(0.0, kPi);
    if (lo > hi) {
        std::swap(lo, hi);
    }
    if (rng.chance(0.1)) {
        lo = 0.0;
    }
    if (rng.chance(0.1)) {
        hi = kPi;
    }
    if (rng.chance(0.05)) {
        hi = lo;
    }
    c.limits = HingeLimits{lo, hi};
    c.minReachM = reachAt(c.upperM, c.lowerM, hi);
    c.maxReachM = reachAt(c.upperM, c.lowerM, lo);

    // Any current pose; only the bone lengths matter to the solver.
    c.chain.root = Vec3{rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)};
    c.chain.mid = c.chain.root + rng.unitVector() * c.upperM;
    c.chain.end = c.chain.mid + rng.unitVector() * c.lowerM;

    // Test choice: a "too near" target needs a reach floor well above zero, otherwise the
    // case is drawn as "too far".
    c.kind = wanted;
    if (c.kind == Kind::TooNear && c.minReachM <= 0.01) {
        c.kind = Kind::TooFar;
    }
    switch (c.kind) {
    case Kind::Reachable:
        c.targetDistanceM = c.minReachM + rng.uniform01() * (c.maxReachM - c.minReachM);
        break;
    case Kind::TooFar:
        c.targetDistanceM = c.maxReachM + rng.uniform(0.01, 1.0);
        break;
    case Kind::TooNear:
        c.targetDistanceM = c.minReachM * rng.uniform(0.05, 0.9);
        break;
    }
    c.targetDirection = rng.unitVector();
    c.target = c.chain.root + c.targetDirection * c.targetDistanceM;
    c.pole = rng.unitVector();
    return c;
}

// Test choice: pole not parallel to the target direction by at least this much.
bool poleIsClearOfTargetDirection(const IkCase& c)
{
    return length(cross(c.pole, c.targetDirection)) >= 0.05;
}

Kind pickKind(SeededRandom& rng)
{
    const int k = rng.uniformInt(0, 2);
    return k == 0 ? Kind::Reachable : (k == 1 ? Kind::TooFar : Kind::TooNear);
}

// Angle between the upper and lower bone directions; 0 = straight. atan2 keeps
// full precision near 0 and pi, where acos would not.
double measuredBend(Vec3 root, Vec3 mid, Vec3 end)
{
    const Vec3 upper = mid - root;
    const Vec3 lower = end - mid;
    return std::atan2(length(cross(upper, lower)), dot(upper, lower));
}

TEST(IkTwoBoneProperty, ReachableTargetsAreReached)
{
    SeededRandom rng(kSeed);
    for (int i = 0; i < kCases; ++i) {
        const IkCase c = makeCase(rng, kSeed, i, Kind::Reachable);
        SCOPED_TRACE(describe(c));
        const TwoBoneResult r = solveTwoBone(c.chain, c.target, c.pole, c.limits);
        ASSERT_TRUE(r.valid);
        ASSERT_TRUE(isFinite(r.mid));
        ASSERT_TRUE(isFinite(r.end));
        EXPECT_LE(distance(r.end, c.target), kTol);
        // Within 1e-9 of a boundary "reached" is a judgement call, so it is only checked
        // when the target is clearly inside.
        const double margin =
            std::min(c.targetDistanceM - c.minReachM, c.maxReachM - c.targetDistanceM);
        if (margin > kTol) {
            EXPECT_TRUE(r.reached);
        }
        if (::testing::Test::HasFailure()) {
            break;
        }
    }
}

TEST(IkTwoBoneProperty, UnreachableTargetsClampToTheNearestReachableDistanceOnTheRay)
{
    SeededRandom rng(kSeed + 1U);
    for (int i = 0; i < kCases; ++i) {
        const IkCase c =
            makeCase(rng, kSeed + 1U, i, rng.chance(0.5) ? Kind::TooFar : Kind::TooNear);
        SCOPED_TRACE(describe(c));
        const TwoBoneResult r = solveTwoBone(c.chain, c.target, c.pole, c.limits);
        ASSERT_TRUE(r.valid);
        ASSERT_TRUE(isFinite(r.mid));
        ASSERT_TRUE(isFinite(r.end));
        ASSERT_TRUE(std::isfinite(r.bendRad));
        EXPECT_FALSE(r.reached);

        const double nearest = c.kind == Kind::TooFar ? c.maxReachM : c.minReachM;
        const Vec3 expectedEnd = c.chain.root + c.targetDirection * nearest;
        EXPECT_PRED_FORMAT3(Vec3Near, r.end, expectedEnd, kTol);
        // Same statement, as distance and direction.
        EXPECT_NEAR(distance(r.end, c.chain.root), nearest, kTol);
        EXPECT_LE(length(cross(normalized(r.end - c.chain.root), c.targetDirection)), 1e-8);
        if (::testing::Test::HasFailure()) {
            break;
        }
    }
}

TEST(IkTwoBoneProperty, BoneLengthsAreKeptAndBendStaysWithinLimitsInEveryCase)
{
    SeededRandom rng(kSeed + 2U);
    for (int i = 0; i < kCases; ++i) {
        const IkCase c = makeCase(rng, kSeed + 2U, i, pickKind(rng));
        SCOPED_TRACE(describe(c));
        const TwoBoneResult r = solveTwoBone(c.chain, c.target, c.pole, c.limits);
        ASSERT_TRUE(r.valid);
        ASSERT_TRUE(isFinite(r.mid));
        ASSERT_TRUE(isFinite(r.end));
        ASSERT_TRUE(std::isfinite(r.bendRad));

        EXPECT_NEAR(distance(c.chain.root, r.mid), c.upperM, kTol);
        EXPECT_NEAR(distance(r.mid, r.end), c.lowerM, kTol);

        const double bend = measuredBend(c.chain.root, r.mid, r.end);
        EXPECT_GE(bend, c.limits.minBendRad - kTol);
        EXPECT_LE(bend, c.limits.maxBendRad + kTol);
        EXPECT_GE(r.bendRad, c.limits.minBendRad - kTol);
        EXPECT_LE(r.bendRad, c.limits.maxBendRad + kTol);
        // Test choice: the reported bend agrees with the positions to 1e-7 (looser than 1e-9
        // because a bend computed with acos near 0 or pi is only good to about 1.5e-8).
        EXPECT_NEAR(r.bendRad, bend, 1e-7);
        if (::testing::Test::HasFailure()) {
            break;
        }
    }
}

TEST(IkTwoBoneProperty, MidJointLiesOnThePoleSideOfTheRootToEndLine)
{
    SeededRandom rng(kSeed + 3U);
    int checked = 0;
    for (int i = 0; i < kCases; ++i) {
        const IkCase c = makeCase(rng, kSeed + 3U, i, pickKind(rng));
        if (!poleIsClearOfTargetDirection(c)) {
            continue;
        }
        SCOPED_TRACE(describe(c));
        const TwoBoneResult r = solveTwoBone(c.chain, c.target, c.pole, c.limits);
        ASSERT_TRUE(r.valid);
        ASSERT_TRUE(isFinite(r.mid));
        ASSERT_TRUE(isFinite(r.end));

        // Components perpendicular to the root -> end line.
        const Vec3 axis = normalized(r.end - c.chain.root);
        const Vec3 midOffset = r.mid - c.chain.root;
        const Vec3 midPerp = midOffset - axis * dot(midOffset, axis);
        const Vec3 polePerp = c.pole - axis * dot(c.pole, axis);
        // A straight or fully folded arm has its mid joint on the line: nothing to check.
        // Test choice: "on the line" means closer than 1e-6 m.
        if (length(midPerp) > 1e-6) {
            EXPECT_GT(dot(midPerp, polePerp), 0.0);
            // Stronger: mid joint, root, end and pole direction are coplanar, mid on the pole side.
            EXPECT_NEAR(dot(cross(axis, midPerp), c.pole), 0.0, 1e-8);
        }
        ++checked;
        if (::testing::Test::HasFailure()) {
            break;
        }
    }
    // Rejection sampling must not have emptied the test.
    EXPECT_GE(checked, 1000);
}

TEST(IkTwoBoneProperty, RotationsMapOriginalBoneDirectionsToTheSolvedOnes)
{
    SeededRandom rng(kSeed + 4U);
    for (int i = 0; i < kCases; ++i) {
        const IkCase c = makeCase(rng, kSeed + 4U, i, pickKind(rng));
        SCOPED_TRACE(describe(c));
        const TwoBoneResult r = solveTwoBone(c.chain, c.target, c.pole, c.limits);
        ASSERT_TRUE(r.valid);
        ASSERT_TRUE(isFinite(r.mid));
        ASSERT_TRUE(isFinite(r.end));
        ASSERT_TRUE(isFinite(r.rootRotation));
        ASSERT_TRUE(isFinite(r.midRotation));

        // rotate() needs unit quaternions, so the results must be unit.
        EXPECT_NEAR(length(r.rootRotation), 1.0, kTol);
        EXPECT_NEAR(length(r.midRotation), 1.0, kTol);

        const Vec3 upperBefore = normalized(c.chain.mid - c.chain.root);
        const Vec3 lowerBefore = normalized(c.chain.end - c.chain.mid);
        const Vec3 upperAfter = normalized(r.mid - c.chain.root);
        const Vec3 lowerAfter = normalized(r.end - r.mid);
        EXPECT_PRED_FORMAT3(Vec3Near, rotate(r.rootRotation, upperBefore), upperAfter, kTol);
        EXPECT_PRED_FORMAT3(Vec3Near, rotate(r.midRotation, lowerBefore), lowerAfter, kTol);
        if (::testing::Test::HasFailure()) {
            break;
        }
    }
}

TEST(IkTwoBoneProperty, SameInputGivesBitIdenticalOutput)
{
    SeededRandom rng(kSeed + 5U);
    for (int i = 0; i < 200; ++i) {
        const IkCase c = makeCase(rng, kSeed + 5U, i, pickKind(rng));
        SCOPED_TRACE(describe(c));
        const TwoBoneResult a = solveTwoBone(c.chain, c.target, c.pole, c.limits);
        const TwoBoneResult b = solveTwoBone(c.chain, c.target, c.pole, c.limits);
        EXPECT_EQ(a.valid, b.valid);
        EXPECT_EQ(a.reached, b.reached);
        EXPECT_EQ(a.mid, b.mid);
        EXPECT_EQ(a.end, b.end);
        EXPECT_EQ(a.bendRad, b.bendRad);
        EXPECT_EQ(a.rootRotation, b.rootRotation);
        EXPECT_EQ(a.midRotation, b.midRotation);
        if (::testing::Test::HasFailure()) {
            break;
        }
    }
}

} // namespace
