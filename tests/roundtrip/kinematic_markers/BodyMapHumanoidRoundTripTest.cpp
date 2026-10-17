// The synthetic humanoid's body mapping, written to text and loaded back, must work with
// measureBody and alignPair exactly as the original mapping does (bit for bit).

#include "AlignmentCompare.hpp"
#include "AlignmentScenes.hpp"
#include "BodyMapTestData.hpp"
#include "SyntheticHumanoid.hpp"
#include "opas/markers/Alignment.hpp"
#include "opas/markers/BodyMapFile.hpp"
#include "opas/markers/BodyMeasurements.hpp"

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

namespace {

using opas::markers::BodyMapLoadResult;
using opas::markers::BodyMapping;
using opas::markers::loadBodyMap;
using opas::markers::measureBody;
using opas::markers::writeBodyMap;
using opas::test::expectSameAlignment;
using opas::test::expectSameMeasurements;
using opas::test::Scene;

// Writes the mapping to text and loads it back.
BodyMapping roundTripped(const BodyMapping& original)
{
    const std::string text = writeBodyMap(original);
    const BodyMapLoadResult loaded = loadBodyMap(text, "humanoid.bodymap.json");
    EXPECT_TRUE(loaded.ok()) << loaded.error.toString() << "\n" << text;
    return loaded.value.value_or(BodyMapping{});
}

TEST(BodyMapHumanoid, TheSyntheticHumanoidMappingSurvivesWriteAndLoad)
{
    for (const double scale : {0.8, 1.0, 1.25}) {
        SCOPED_TRACE(scale);
        const opas::test::Humanoid h = opas::test::makeHumanoid(scale);
        const std::string text = writeBodyMap(h.mapping);
        const BodyMapping back = roundTripped(h.mapping);
        EXPECT_TRUE(opas::test::sameMapping(back, h.mapping));
        EXPECT_EQ(writeBodyMap(back), text);
    }
}

TEST(BodyMapHumanoid, MeasureBodyGivesBitIdenticalResultsForTheLoadedMapping)
{
    for (const double scale : {0.8, 1.0, 1.25}) {
        SCOPED_TRACE(scale);
        const opas::test::Humanoid h = opas::test::makeHumanoid(scale);
        const BodyMapping back = roundTripped(h.mapping);
        const auto original = measureBody(h.skeleton, h.mapping);
        const auto loaded = measureBody(h.skeleton, back);
        ASSERT_TRUE(original.value.has_value()) << original.error;
        ASSERT_TRUE(loaded.value.has_value()) << loaded.error;
        expectSameMeasurements(original, loaded);
    }
}

// Replaces both actors' body mapping by one that went through a file.
Scene withRoundTrippedMappings(Scene scene)
{
    for (opas::test::SceneActor& actor : scene.actors) {
        actor.body.mapping = roundTripped(actor.body.mapping);
    }
    return scene;
}

TEST(BodyMapHumanoid, AlignPairGivesBitIdenticalResultsForTheLoadedMapping)
{
    struct Case {
        std::string name;
        Scene scene;
    };
    std::vector<Case> cases;
    for (const auto& [s0, s1] :
         std::vector<std::pair<double, double>>{{0.8, 1.25}, {1.25, 0.8}, {1.0, 1.0}}) {
        const std::string scales = std::to_string(s0) + "/" + std::to_string(s1);
        cases.push_back({"hug " + scales, opas::test::makeHugScene(s0, s1)});
        cases.push_back({"handshake " + scales, opas::test::makeHandshakeScene(s0, s1)});
        cases.push_back({"shoulder " + scales, opas::test::makeShoulderScene(s0, s1)});
    }
    // The penetration scene overrides actor 0's pole direction, so it also checks that a
    // changed pole survives the file.
    cases.push_back({"penetration", opas::test::makePenetrationScene()});

    for (const Case& c : cases) {
        SCOPED_TRACE(c.name);
        const Scene loaded = withRoundTrippedMappings(c.scene);
        const auto original = opas::test::alignScene(c.scene);
        const auto viaFile = opas::test::alignScene(loaded);
        ASSERT_TRUE(original.valid) << original.error;
        ASSERT_TRUE(viaFile.valid) << viaFile.error;
        expectSameAlignment(original, viaFile);
    }
}

TEST(BodyMapHumanoid, AlignPairAlsoMatchesWithSettingsThatChangeThePoleSearch)
{
    const Scene scene = opas::test::makePenetrationScene();
    const Scene loaded = withRoundTrippedMappings(scene);
    opas::markers::AlignmentSettings settings;
    settings.poleSamples = 1;
    expectSameAlignment(opas::test::alignScene(scene, settings),
                        opas::test::alignScene(loaded, settings));
}

} // namespace
