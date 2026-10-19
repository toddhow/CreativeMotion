#pragma once

// Counting replacements for the global operator new/delete (CountingNew.cpp),
// linked only into opas_alloc_tests. While counting is on, every allocation
// through operator new is counted, from any code in the process.

#include <cstdint>

namespace opas::test {

void startCountingAllocations();
// Stops counting and returns how many allocations happened since the start.
std::uint64_t stopCountingAllocations();

} // namespace opas::test
