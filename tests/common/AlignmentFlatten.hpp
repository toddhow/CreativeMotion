#pragma once

// An AlignmentResult as a flat list of 64-bit words (doubles by their bit patterns), so that a
// result can be compared bit for bit with one recorded earlier (see AlignmentGoldens.hpp).

#include "opas/markers/Alignment.hpp"

#include <bit>
#include <cstdint>
#include <vector>

namespace opas::test {

inline void pushBits(std::vector<std::uint64_t>& out, double v)
{
    out.push_back(std::bit_cast<std::uint64_t>(v));
}

inline std::vector<std::uint64_t> flatten(const markers::AlignmentResult& r)
{
    std::vector<std::uint64_t> out;
    out.push_back(r.valid ? 1U : 0U);
    out.push_back(r.rootShiftClamped ? 1U : 0U);
    for (const math::Vec3& s : r.rootShiftM) {
        pushBits(out, s.x);
        pushBits(out, s.y);
        pushBits(out, s.z);
    }
    out.push_back(r.limbs.size());
    for (const markers::LimbCorrection& l : r.limbs) {
        out.push_back(static_cast<std::uint64_t>(l.actor));
        out.push_back(l.reached ? 1U : 0U);
        for (const math::Quat& q : {l.rootBoneDelta, l.midBoneDelta}) {
            pushBits(out, q.x);
            pushBits(out, q.y);
            pushBits(out, q.z);
            pushBits(out, q.w);
        }
        pushBits(out, l.penetrationM);
    }
    out.push_back(r.markers.size());
    for (const markers::MarkerResidual& m : r.markers) {
        pushBits(out, m.distanceM);
        out.push_back(m.met ? 1U : 0U);
    }
    return out;
}

} // namespace opas::test
