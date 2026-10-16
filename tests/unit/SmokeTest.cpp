#include <gtest/gtest.h>

// Proves the test runner itself is wired up. If this fails, nothing else
// in the suite can be trusted.
TEST(Smoke, TestRunnerWorks)
{
    EXPECT_EQ(1 + 1, 2);
}
