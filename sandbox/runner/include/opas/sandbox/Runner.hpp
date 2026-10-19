#pragma once

// The headless runner: plays a scenario with the Jolt backend and the motor
// solver and records every ragdoll body each step (M4, docs/tasks/0015).

#include "opas/format/LoadError.hpp"
#include "opas/math/Transform.hpp"
#include "opas/motor/MotorSolver.hpp"
#include "opas/rig/Rig.hpp"
#include "opas/sandbox/Recording.hpp"
#include "opas/sandbox/Scenario.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace opas::sandbox {

struct RunOptions {
    // Where relative rig and profile paths start: the scenario file's folder.
    std::filesystem::path baseDir;
    // Used only in errors, e.g. "push.scenario.json".
    std::string scenarioFile;
};

struct RunResult {
    std::optional<Recording> recording;
    format::LoadError error; // located at the scenario entry that failed

    [[nodiscard]] bool ok() const { return recording.has_value(); }
};

// Builds the world, statics and ragdolls, drives each ragdoll with its pose
// source, applies impulses and records. Deterministic: the same scenario
// always gives the same recording on the same build. Never throws.
RunResult run(const Scenario& scenario, const RunOptions& options);

// Loads a scenario file and runs it with baseDir = the file's folder.
RunResult runFile(const std::filesystem::path& scenarioPath);

// The built-in pose sources a scenario's ragdoll may name.
// "rest": the rig's rest pose. "armWave": M3's arm wave (the right shoulder
// swings 0.6 rad about +X and the right elbow bends 0.3 to 1.2 rad about +X,
// both at 0.5 Hz); it needs bones named RightUpperArm and RightForearm.
bool isPoseSourceName(std::string_view name);

// Makes the named pose source for a rig, or an empty result with a message
// in *error (an unknown name, or a bone the source needs is missing).
std::optional<motor::PoseSource> makePoseSource(std::string_view name, const rig::Rig& rig,
                                                std::string* error);

} // namespace opas::sandbox
