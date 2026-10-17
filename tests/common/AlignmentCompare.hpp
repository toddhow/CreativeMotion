#pragma once

// Bit-for-bit comparison of solver results, for tests that run the same inputs through two
// routes (e.g. an original body mapping and one written to a file and loaded back).

#include "opas/markers/Alignment.hpp"
#include "opas/markers/BodyMeasurements.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <gtest/gtest.h>

#include <bit>
#include <cstddef>
#include <cstdint>

namespace opas::test {

inline bool bitsOf(double a, double b)
{
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

inline bool bitsOf(math::Vec3 a, math::Vec3 b)
{
    return bitsOf(a.x, b.x) && bitsOf(a.y, b.y) && bitsOf(a.z, b.z);
}

inline bool bitsOf(math::Quat a, math::Quat b)
{
    return bitsOf(a.x, b.x) && bitsOf(a.y, b.y) && bitsOf(a.z, b.z) && bitsOf(a.w, b.w);
}

inline void expectSameAlignment(const markers::AlignmentResult& a,
                                const markers::AlignmentResult& b)
{
    EXPECT_EQ(a.valid, b.valid);
    EXPECT_EQ(a.error, b.error);
    EXPECT_EQ(a.rootShiftClamped, b.rootShiftClamped);
    EXPECT_TRUE(bitsOf(a.rootShiftM[0], b.rootShiftM[0]));
    EXPECT_TRUE(bitsOf(a.rootShiftM[1], b.rootShiftM[1]));
    ASSERT_EQ(a.limbs.size(), b.limbs.size());
    for (std::size_t i = 0; i < a.limbs.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(a.limbs[i].actor, b.limbs[i].actor);
        EXPECT_EQ(a.limbs[i].chain, b.limbs[i].chain);
        EXPECT_EQ(a.limbs[i].markerId, b.limbs[i].markerId);
        EXPECT_EQ(a.limbs[i].reached, b.limbs[i].reached);
        EXPECT_TRUE(bitsOf(a.limbs[i].rootBoneDelta, b.limbs[i].rootBoneDelta));
        EXPECT_TRUE(bitsOf(a.limbs[i].midBoneDelta, b.limbs[i].midBoneDelta));
        EXPECT_TRUE(bitsOf(a.limbs[i].penetrationM, b.limbs[i].penetrationM));
    }
    ASSERT_EQ(a.markers.size(), b.markers.size());
    for (std::size_t i = 0; i < a.markers.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(a.markers[i].id, b.markers[i].id);
        EXPECT_EQ(a.markers[i].met, b.markers[i].met);
        EXPECT_TRUE(bitsOf(a.markers[i].distanceM, b.markers[i].distanceM));
    }
}

inline void expectSameMeasurements(const markers::MeasureResult& a, const markers::MeasureResult& b)
{
    ASSERT_EQ(a.value.has_value(), b.value.has_value());
    EXPECT_EQ(a.error, b.error);
    if (!a.value.has_value() || !b.value.has_value()) {
        return;
    }
    const markers::BodyMeasurements& x = *a.value;
    const markers::BodyMeasurements& y = *b.value;
    EXPECT_TRUE(bitsOf(x.heightM, y.heightM));
    ASSERT_EQ(x.limbs.size(), y.limbs.size());
    for (std::size_t i = 0; i < x.limbs.size(); ++i) {
        EXPECT_EQ(x.limbs[i].name, y.limbs[i].name);
        EXPECT_TRUE(bitsOf(x.limbs[i].lengthM, y.limbs[i].lengthM));
    }
    ASSERT_EQ(x.capsules.size(), y.capsules.size());
    for (std::size_t i = 0; i < x.capsules.size(); ++i) {
        EXPECT_EQ(x.capsules[i].region, y.capsules[i].region);
        EXPECT_TRUE(bitsOf(x.capsules[i].startM, y.capsules[i].startM));
        EXPECT_TRUE(bitsOf(x.capsules[i].endM, y.capsules[i].endM));
        EXPECT_TRUE(bitsOf(x.capsules[i].radiusM, y.capsules[i].radiusM));
    }
}

} // namespace opas::test
