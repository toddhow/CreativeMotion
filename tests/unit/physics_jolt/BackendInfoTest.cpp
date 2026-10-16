#include "opas/physics/jolt/BackendInfo.hpp"

#include <gtest/gtest.h>

namespace {

using opas::physics::jolt::countRegisteredJoltTypes;
using opas::physics::jolt::linkedJoltVersion;

// Must match the pin in cmake/OpasDependencies.cmake. Changing the pin without
// updating this test (and docs/dependencies.md) fails the build on purpose.
TEST(JoltBackendInfo, LinksPinnedJoltVersion)
{
    const auto version = linkedJoltVersion();
    EXPECT_EQ(version.major, 5);
    EXPECT_EQ(version.minor, 6);
    EXPECT_EQ(version.patch, 0);
}

TEST(JoltBackendInfo, RegistersJoltTypes)
{
    EXPECT_GT(countRegisteredJoltTypes(), 0U);
}

TEST(JoltBackendInfo, RegistrationCanBeRepeated)
{
    // Register/unregister must leave Jolt's globals clean for the next user.
    const auto first = countRegisteredJoltTypes();
    const auto second = countRegisteredJoltTypes();
    EXPECT_EQ(first, second);
}

} // namespace
