#pragma once

// A sandbox scenario: a world, static boxes, ragdolls and impulses, played by
// the headless runner (M4, docs/tasks/0015). File format:
// docs/formats/scenario.md.

#include "opas/format/LoadError.hpp"
#include "opas/math/Transform.hpp"
#include "opas/math/Vec3.hpp"
#include "opas/motor/Ragdoll.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace opas::sandbox {

// A static box: floors, stair steps and slopes are all boxes.
struct StaticBox {
    std::string name;
    math::Vec3 halfExtentsM;
    math::Transform pose; // centre and rotation in the world
    double friction = 0.0;
    std::size_t line = 0; // line of this entry in its file (0 when built in code)
};

enum class MotorMode : std::uint8_t {
    Profile, // the profile's motor gains
    Off,     // stiffness 0 on every joint, damping kept: a limp ragdoll
};

struct ScenarioRagdoll {
    std::string name;
    std::string rigPath;     // as written, relative to the scenario file
    std::string profilePath; // as written, relative to the scenario file
    math::Transform root;    // where the model root sits in the world
    motor::RootMotion rootMotion = motor::RootMotion::Dynamic;
    MotorMode motors = MotorMode::Profile;
    std::string pose; // a built-in pose source (PoseSources.hpp)
    // Line of this entry in its file (0 when built in code), so errors found
    // while running (a missing rig file) still point at the file.
    std::size_t line = 0;
};

// Applied before the first step whose start time is >= atS, to the named
// bone's body, at its centre plus offsetM (world axes).
struct Impulse {
    double atS = 0.0;
    std::string ragdoll;
    std::string bone;
    math::Vec3 impulseNs;
    math::Vec3 offsetM;
    std::size_t line = 0; // as for ScenarioRagdoll::line
};

struct Scenario {
    std::string name;
    int stepsPerSecond = 60;
    double durationS = 0.0;
    math::Vec3 gravityMps2;
    std::vector<StaticBox> statics;
    std::vector<ScenarioRagdoll> ragdolls;
    std::vector<Impulse> impulses;
};

// durationS * stepsPerSecond, rounded to the nearest step.
int stepCount(const Scenario& scenario);

// The index of the step before which an impulse at `atS` is applied: the
// first step whose start time (index / stepsPerSecond) is >= atS.
int impulseStep(const Scenario& scenario, double atS);

struct ScenarioLoadResult {
    std::optional<Scenario> value;
    format::LoadError error; // meaningful only when value is empty

    [[nodiscard]] bool ok() const { return value.has_value(); }
};

// Parses and validates scenario text. Never throws.
ScenarioLoadResult loadScenario(std::string_view text, std::string_view fileName);
ScenarioLoadResult loadScenarioFile(const std::filesystem::path& path);

// Canonical text (docs/formats/scenario.md); loading it and writing again
// gives the same bytes. Line numbers are not written.
std::string writeScenario(const Scenario& scenario);

} // namespace opas::sandbox
