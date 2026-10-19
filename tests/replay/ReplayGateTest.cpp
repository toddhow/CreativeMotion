#include "opas/sandbox/Recording.hpp"
#include "opas/sandbox/Runner.hpp"
#include "opas/sandbox/Scenario.hpp"

#include <gtest/gtest.h>

#include <string>

// Replay and Determinism gates (M4, docs/tasks/0015). The CI replay gate also
// runs opas_run --compare per scenario (tests/CMakeLists.txt); these tests
// check the same in-process, plus determinism and that a change is caught.

namespace {

using namespace opas;

std::string scenarioPath(const std::string& name)
{
    return std::string(OPAS_SANDBOX_SCENARIO_DIR) + "/" + name + ".scenario.json";
}

std::string goldenPath(const std::string& name)
{
    return std::string(OPAS_REPO_DATA_DIR) + "/recordings/" + name + ".recording.json";
}

class ShippedScenarioRun : public ::testing::TestWithParam<const char*> {};

TEST_P(ShippedScenarioRun, MatchesItsGoldenRecording)
{
    const sandbox::RunResult run = sandbox::runFile(scenarioPath(GetParam()));
    ASSERT_TRUE(run.ok()) << run.error.toString();
    const auto golden = sandbox::loadRecordingFile(goldenPath(GetParam()));
    ASSERT_TRUE(golden.ok()) << golden.error.toString();
    const auto difference = sandbox::compareRecordings(run.recording.value_or(sandbox::Recording{}),
                                                       golden.value.value_or(sandbox::Recording{}));
    EXPECT_FALSE(difference.has_value()) << (difference ? difference->toString() : "");
}

TEST_P(ShippedScenarioRun, TwoRunsAreBitIdentical)
{
    const sandbox::RunResult first = sandbox::runFile(scenarioPath(GetParam()));
    const sandbox::RunResult second = sandbox::runFile(scenarioPath(GetParam()));
    ASSERT_TRUE(first.ok()) << first.error.toString();
    ASSERT_TRUE(second.ok()) << second.error.toString();
    EXPECT_TRUE(sandbox::sameBits(first.recording.value_or(sandbox::Recording{}),
                                  second.recording.value_or(sandbox::Recording{})));
}

INSTANTIATE_TEST_SUITE_P(Gates, ShippedScenarioRun,
                         ::testing::Values("floor", "stairs", "slope", "push"));

// The gate notices a change in behaviour: a 61 N s push instead of 60.
TEST(ReplayGate, ASlightlyHarderPushIsCaught)
{
    const auto loaded = sandbox::loadScenarioFile(scenarioPath("push"));
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    sandbox::Scenario scenario = loaded.value.value_or(sandbox::Scenario{});
    ASSERT_EQ(scenario.impulses.size(), 1U);
    scenario.impulses[0].impulseNs.y = 61.0;
    const sandbox::RunResult run = sandbox::run(
        scenario, sandbox::RunOptions{OPAS_SANDBOX_SCENARIO_DIR, "push.scenario.json"});
    ASSERT_TRUE(run.ok()) << run.error.toString();
    const auto golden = sandbox::loadRecordingFile(goldenPath("push"));
    ASSERT_TRUE(golden.ok()) << golden.error.toString();
    EXPECT_TRUE(sandbox::compareRecordings(run.recording.value_or(sandbox::Recording{}),
                                           golden.value.value_or(sandbox::Recording{}))
                    .has_value());
}

} // namespace
