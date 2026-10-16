#pragma once

// Bootstrap (M0) checks that the Jolt backend is built and linked correctly.
// Deliberately free of Jolt types: callers never see Jolt headers.
// M1 replaces these with the real backend behind physics/interface.

#include <cstddef>

namespace opas::physics::jolt {

struct JoltVersion {
    int major = 0;
    int minor = 0;
    int patch = 0;
};

// The Jolt version this library was compiled against.
JoltVersion linkedJoltVersion();

// Registers Jolt's types, counts them, then unregisters them again.
// A non-zero result proves Jolt's compiled code (not just its headers) is
// linked in. Not thread-safe: it touches Jolt's global factory.
std::size_t countRegisteredJoltTypes();

} // namespace opas::physics::jolt
