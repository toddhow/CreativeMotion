#include "TestSupport.hpp"
#include "opas/sandbox/Recording.hpp"
#include "opas/sandbox/Scenario.hpp"

#include <gtest/gtest.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

// Round trips for scenario and recording files (M4, docs/tasks/0015).

namespace {

using namespace opas;

std::string fileText(const std::string& path)
{
    const std::ifstream in(path, std::ios::binary);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

class ShippedScenario : public ::testing::TestWithParam<const char*> {};

TEST_P(ShippedScenario, IsCanonical)
{
    const std::string path =
        std::string(OPAS_SANDBOX_SCENARIO_DIR) + "/" + GetParam() + ".scenario.json";
    const auto loaded = sandbox::loadScenarioFile(path);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_EQ(sandbox::writeScenario(loaded.value.value_or(sandbox::Scenario{})), fileText(path));
}

TEST_P(ShippedScenario, GoldenRecordingIsCanonicalAndSmall)
{
    const std::string path =
        std::string(OPAS_REPO_DATA_DIR) + "/recordings/" + GetParam() + ".recording.json";
    const std::string text = fileText(path);
    EXPECT_LT(text.size(), 2U * 1024U * 1024U); // under 2 MB (spec)
    const auto loaded = sandbox::loadRecording(text, path);
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    const sandbox::Recording golden = loaded.value.value_or(sandbox::Recording{});
    EXPECT_EQ(golden.scenario, GetParam());
    EXPECT_EQ(sandbox::writeRecording(loaded.value.value_or(sandbox::Recording{})), text);
}

INSTANTIATE_TEST_SUITE_P(Sandbox, ShippedScenario,
                         ::testing::Values("floor", "stairs", "slope", "push"));

sandbox::Recording oneBody(const sandbox::BodySample& sample)
{
    sandbox::Recording r;
    r.scenario = "edge";
    r.bodies = {"x/Bone"};
    r.steps = {{sample}};
    return r;
}

TEST(RecordingRoundTrip, EdgeFloatsSurviveExactly)
{
    const float denormal = std::numeric_limits<float>::denorm_min();
    const sandbox::BodySample sample{std::numeric_limits<float>::max(),
                                     -std::numeric_limits<float>::max(),
                                     denormal,
                                     -denormal,
                                     std::numeric_limits<float>::min(),
                                     0.1f,
                                     1.0f / 3.0f,
                                     16777216.0f,
                                     -0.0f,
                                     1e-30f,
                                     3.0e38f,
                                     std::nextafter(1.0f, 2.0f),
                                     std::nextafter(0.7071068f, 0.0f)};
    const sandbox::Recording recording = oneBody(sample);
    const std::string text = sandbox::writeRecording(recording);
    const auto loaded = sandbox::loadRecording(text, "edge.recording.json");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    for (std::size_t i = 0; i < sandbox::kBodySampleSize; ++i) {
        const float expected = sample[i] == 0.0f ? 0.0f : sample[i]; // -0 is written 0
        EXPECT_EQ(std::bit_cast<std::uint32_t>(
                      loaded.value.value_or(sandbox::Recording{}).steps.at(0).at(0)[i]),
                  std::bit_cast<std::uint32_t>(expected))
            << "value " << i << " = " << sample[i];
    }
    EXPECT_EQ(sandbox::writeRecording(loaded.value.value_or(sandbox::Recording{})), text);
}

// Shortest float text read back through the parser's double: every one of
// about 20000 seeded random finite bit patterns survives.
TEST(RecordingRoundTrip, RandomFloatBitPatternsSurvive)
{
    test::SeededRandom random(15U);
    sandbox::Recording recording;
    recording.scenario = "random";
    recording.bodies = {"x/Bone"};
    for (int step = 0; step < 20000 / 13 + 1; ++step) {
        sandbox::BodySample sample{};
        for (float& value : sample) {
            for (;;) {
                const auto high = static_cast<std::uint32_t>(random.uniformInt(0, 65535));
                const auto low = static_cast<std::uint32_t>(random.uniformInt(0, 65535));
                value = std::bit_cast<float>((high << 16U) | low);
                if (std::isfinite(value) && value != 0.0f) {
                    break;
                }
            }
        }
        recording.steps.push_back({sample});
    }
    const auto loaded = sandbox::loadRecording(sandbox::writeRecording(recording), "random");
    ASSERT_TRUE(loaded.ok()) << loaded.error.toString();
    EXPECT_TRUE(sandbox::sameBits(loaded.value.value_or(sandbox::Recording{}), recording));
}

TEST(RecordingLoad, RejectsBadSamplesWithTheirPointer)
{
    const sandbox::Recording recording = oneBody(sandbox::BodySample{});
    const std::string text = sandbox::writeRecording(recording);
    const std::string zeros = "[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]";
    ASSERT_NE(text.find(zeros), std::string::npos);

    std::string bad = text;
    bad.replace(bad.find(zeros), zeros.size(), "[0, 0, 0, 0, 0, 0, 0, 1e39, 0, 0, 0, 0, 0]");
    auto loaded = sandbox::loadRecording(bad, "bad");
    EXPECT_FALSE(loaded.ok());
    EXPECT_EQ(loaded.error.field, "/steps/0/0/7");

    bad = text;
    bad.replace(bad.find(zeros), zeros.size(), "[0, 0, 0]");
    loaded = sandbox::loadRecording(bad, "bad");
    EXPECT_FALSE(loaded.ok());
    EXPECT_EQ(loaded.error.field, "/steps/0/0");
}

TEST(RecordingCompare, ReportsTheFirstDifferenceBeyondTolerance)
{
    sandbox::Recording a = oneBody(sandbox::BodySample{0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0});
    a.steps.push_back(a.steps[0]);
    sandbox::Recording b = a;
    EXPECT_FALSE(sandbox::compareRecordings(a, b).has_value());

    b.steps[1][0][2] = 1.0005f; // 0.5 mm: within 1 mm
    EXPECT_FALSE(sandbox::compareRecordings(a, b).has_value());
    b.steps[1][0][2] = 1.002f; // 2 mm
    auto difference = sandbox::compareRecordings(a, b);
    ASSERT_TRUE(difference.has_value());
    EXPECT_EQ(difference->step, 1U);
    EXPECT_EQ(difference->body, "x/Bone");
    EXPECT_EQ(difference->field, "position");

    b = a;
    b.steps[1][0][6] = -1.0f; // -q against q: the same rotation
    EXPECT_FALSE(sandbox::compareRecordings(a, b).has_value());

    b = a;
    b.steps[0][0][10] = 0.1f; // angular velocity 0.1 rad/s > 0.05
    difference = sandbox::compareRecordings(a, b);
    ASSERT_TRUE(difference.has_value());
    EXPECT_EQ(difference->step, 0U);
    EXPECT_EQ(difference->field, "angularVelocity");

    b = a;
    b.bodies[0] = "y/Bone";
    difference = sandbox::compareRecordings(a, b);
    ASSERT_TRUE(difference.has_value());
    EXPECT_EQ(difference->field, "bodies");
}

} // namespace
