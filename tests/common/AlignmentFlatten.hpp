#pragma once

// An AlignmentResult as a flat list of 64-bit words (doubles by their bit patterns), so that a
// result can be compared bit for bit with one recorded earlier (see AlignmentGoldens.hpp).
// `isDouble` marks which words hold doubles: the goldens were recorded with MSVC, and another
// C library may round sin/cos/acos differently in the last bit, so off MSVC those words are
// compared with a tolerance instead (the counts and flags must still match exactly).

#include "opas/markers/Alignment.hpp"

#include <bit>
#include <cstdint>
#include <vector>

namespace opas::test {

struct FlatResult {
    std::vector<std::uint64_t> words;
    std::vector<bool> isDouble; // same length as words

    void bits(double v)
    {
        words.push_back(std::bit_cast<std::uint64_t>(v));
        isDouble.push_back(true);
    }
    void word(std::uint64_t v)
    {
        words.push_back(v);
        isDouble.push_back(false);
    }
};

inline FlatResult flatten(const markers::AlignmentResult& r)
{
    FlatResult out;
    out.word(r.valid ? 1U : 0U);
    out.word(r.rootShiftClamped ? 1U : 0U);
    for (const math::Vec3& s : r.rootShiftM) {
        out.bits(s.x);
        out.bits(s.y);
        out.bits(s.z);
    }
    out.word(r.limbs.size());
    for (const markers::LimbCorrection& l : r.limbs) {
        out.word(static_cast<std::uint64_t>(l.actor));
        out.word(l.reached ? 1U : 0U);
        for (const math::Quat& q : {l.rootBoneDelta, l.midBoneDelta}) {
            out.bits(q.x);
            out.bits(q.y);
            out.bits(q.z);
            out.bits(q.w);
        }
        out.bits(l.penetrationM);
    }
    out.word(r.markers.size());
    for (const markers::MarkerResidual& m : r.markers) {
        out.bits(m.distanceM);
        out.word(m.met ? 1U : 0U);
    }
    return out;
}

} // namespace opas::test
