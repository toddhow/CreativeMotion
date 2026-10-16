#include "TestSupport.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

namespace {

using opas::math::angleBetween;
using opas::math::fromAxisAngle;
using opas::math::length;
using opas::math::normalized;
using opas::math::Quat;
using opas::test::kPi;
using opas::test::SeededRandom;

constexpr std::uint32_t kSeed = 20260101U;
constexpr int kChains = 5;
constexpr int kRotationsPerChain = 1000;
constexpr int kCases = 1000;

Quat randomRotation(SeededRandom& rng)
{
    return fromAxisAngle(rng.unitVector(), rng.uniform(-2.0 * kPi, 2.0 * kPi));
}

TEST(MathQuatProperty, ComposingThousandsOfRotationsWithNormalizedKeepsUnitLength)
{
    for (int chain = 0; chain < kChains; ++chain) {
        const std::uint32_t seed = kSeed + static_cast<std::uint32_t>(chain);
        SeededRandom rng(seed);
        Quat accumulated;
        for (int step = 0; step < kRotationsPerChain; ++step) {
            accumulated = normalized(accumulated * randomRotation(rng));
            const double error = std::abs(length(accumulated) - 1.0);
            ASSERT_LE(error, 1e-12) << "seed=" << seed << " step=" << step
                                    << " q=" << opas::test::str(accumulated) << " |q|-1=" << error;
        }
    }
}

TEST(MathQuatProperty, ComposingInTheOtherOrderAlsoKeepsUnitLength)
{
    const std::uint32_t seed = kSeed + 100U;
    SeededRandom rng(seed);
    Quat accumulated;
    for (int step = 0; step < kRotationsPerChain; ++step) {
        accumulated = normalized(randomRotation(rng) * accumulated);
        const double error = std::abs(length(accumulated) - 1.0);
        ASSERT_LE(error, 1e-12) << "seed=" << seed << " step=" << step
                                << " q=" << opas::test::str(accumulated);
    }
}

TEST(MathQuatProperty, AngleBetweenQuaternionAndItsNegationIsZero)
{
    const std::uint32_t seed = kSeed + 200U;
    SeededRandom rng(seed);
    for (int i = 0; i < kCases; ++i) {
        const Quat q = randomRotation(rng);
        const Quat negated{-q.x, -q.y, -q.z, -q.w};
        // Spec: 0 by the shorter-path rule. The bound is 1e-12 rather than exact equality so the
        // test checks the rule, not the last bit of a dot product.
        const double angle = angleBetween(q, negated);
        ASSERT_LE(angle, 1e-12) << "seed=" << seed << " case=" << i << " q=" << opas::test::str(q)
                                << " angle=" << angle;
    }
}

TEST(MathQuatProperty, AngleBetweenIgnoresTheSignOfEitherQuaternion)
{
    const std::uint32_t seed = kSeed + 300U;
    SeededRandom rng(seed);
    for (int i = 0; i < kCases; ++i) {
        const Quat a = randomRotation(rng);
        const Quat b = randomRotation(rng);
        const Quat negB{-b.x, -b.y, -b.z, -b.w};
        const double direct = angleBetween(a, b);
        const double flipped = angleBetween(a, negB);
        ASSERT_GE(direct, 0.0) << "case=" << i;
        ASSERT_LE(direct, kPi + 1e-12) << "case=" << i;
        ASSERT_NEAR(direct, flipped, 1e-12)
            << "seed=" << seed << " case=" << i << " a=" << opas::test::str(a)
            << " b=" << opas::test::str(b);
    }
}

} // namespace
