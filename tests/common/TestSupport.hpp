#pragma once

// Helpers shared by the kinematic-foundation tests (A1): a deterministic
// random source and floating-point comparison assertions.

#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <string_view>

namespace opas::test {

inline constexpr double kPi = 3.14159265358979323846;

// Seeded random numbers with the same output on MSVC, GCC and Clang.
// std::mt19937 is fully specified by the standard, but
// std::uniform_real_distribution is not, so doubles are built here from the
// raw 32-bit outputs.
class SeededRandom {
public:
    explicit SeededRandom(std::uint32_t seed) : engine_(seed) {}

    // Uniform in [0, 1) with 53 random bits.
    double uniform01()
    {
        const std::uint32_t high = static_cast<std::uint32_t>(engine_()) >> 5U;
        const std::uint32_t low = static_cast<std::uint32_t>(engine_()) >> 6U;
        return (static_cast<double>(high) * 67108864.0 + static_cast<double>(low)) *
               (1.0 / 9007199254740992.0);
    }

    // Uniform in [lo, hi).
    double uniform(double lo, double hi) { return lo + (hi - lo) * uniform01(); }

    // Uniform integer in [lo, hi], both included.
    int uniformInt(int lo, int hi)
    {
        const double span = static_cast<double>(hi - lo + 1);
        const int offset = static_cast<int>(std::floor(uniform01() * span));
        return lo + std::min(offset, hi - lo);
    }

    // True with probability p.
    bool chance(double p) { return uniform01() < p; }

    // Unit vector, roughly uniform on the sphere (rejection from a cube).
    math::Vec3 unitVector()
    {
        for (;;) {
            const math::Vec3 v{uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0)};
            const double len = math::length(v);
            if (len > 0.1 && len <= 1.0) {
                return v / len;
            }
        }
    }

private:
    std::mt19937 engine_;
};

inline std::string str(math::Vec3 v)
{
    std::ostringstream out;
    out << std::setprecision(17) << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return out.str();
}

inline std::string str(math::Quat q)
{
    std::ostringstream out;
    out << std::setprecision(17) << "(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")";
    return out.str();
}

inline bool isFinite(math::Vec3 v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool isFinite(math::Quat q)
{
    return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

inline bool contains(std::string_view haystack, std::string_view needle)
{
    return haystack.find(needle) != std::string_view::npos;
}

// Use as EXPECT_PRED_FORMAT3(Vec3Near, actual, expected, tolerance). Fails on NaN.
inline ::testing::AssertionResult Vec3Near(const char* actualExpr, const char* expectedExpr,
                                           const char* /*tolExpr*/, math::Vec3 actual,
                                           math::Vec3 expected, double tolerance)
{
    const double error = math::distance(actual, expected);
    if (error <= tolerance) {
        return ::testing::AssertionSuccess();
    }
    return ::testing::AssertionFailure()
           << actualExpr << " = " << str(actual) << " differs from " << expectedExpr << " = "
           << str(expected) << " by " << std::setprecision(17) << error << " (tolerance "
           << tolerance << ")";
}

// Use as EXPECT_PRED_FORMAT3(QuatSameRotation, actual, expected, tolerance).
// q and -q are the same rotation, so the closer sign is compared.
inline ::testing::AssertionResult QuatSameRotation(const char* actualExpr, const char* expectedExpr,
                                                   const char* /*tolExpr*/, math::Quat actual,
                                                   math::Quat expected, double tolerance)
{
    const double same =
        std::max({std::abs(actual.x - expected.x), std::abs(actual.y - expected.y),
                  std::abs(actual.z - expected.z), std::abs(actual.w - expected.w)});
    const double flipped =
        std::max({std::abs(actual.x + expected.x), std::abs(actual.y + expected.y),
                  std::abs(actual.z + expected.z), std::abs(actual.w + expected.w)});
    const double error = std::min(same, flipped);
    if (error <= tolerance) {
        return ::testing::AssertionSuccess();
    }
    return ::testing::AssertionFailure()
           << actualExpr << " = " << str(actual) << " is not the rotation " << expectedExpr << " = "
           << str(expected) << " (component error " << std::setprecision(17) << error
           << ", tolerance " << tolerance << ")";
}

} // namespace opas::test
