#pragma once

// A recording: every ragdoll body's state at every step of a scenario run
// (M4, docs/tasks/0015). File format: docs/formats/recording.md.

#include "opas/format/LoadError.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace opas::sandbox {

// One body at one step, as the physics interface reports it: position (3),
// rotation x, y, z, w (4), linear velocity (3), angular velocity (3).
using BodySample = std::array<float, 13>;
inline constexpr std::size_t kBodySampleSize = 13;

struct Recording {
    std::string scenario;
    int stepsPerSecond = 60;
    std::vector<std::string> bodies; // "<ragdoll>/<bone>", in recording order
    // steps[0] is the initial state, steps[k] the state after step k; each
    // has one sample per body.
    std::vector<std::vector<BodySample>> steps;

    friend bool operator==(const Recording&, const Recording&) = default;
};

struct RecordingLoadResult {
    std::optional<Recording> value;
    format::LoadError error;

    [[nodiscard]] bool ok() const { return value.has_value(); }
};

RecordingLoadResult loadRecording(std::string_view text, std::string_view fileName);
RecordingLoadResult loadRecordingFile(const std::filesystem::path& path);

// Canonical text: one step per line, floats in shortest round-trip form.
// Every value must be finite.
std::string writeRecording(const Recording& recording);

// Bit-for-bit equality, including -0 against 0 (operator== on floats would
// call them equal).
bool sameBits(const Recording& a, const Recording& b);

struct Tolerances {
    double positionM = 0.001;
    double rotationRad = 0.01;
    double linearVelocityMps = 0.01;
    double angularVelocityRadps = 0.05;
};

// The first place two recordings disagree beyond the tolerances.
struct Difference {
    std::size_t step = 0;
    std::string body; // empty for a header difference
    std::string
        field; // "position", "rotation", "linearVelocity", "angularVelocity" or a header field
    double amount = 0.0; // distance, angle or speed difference found
    double tolerance = 0.0;

    // e.g. "step 42, hero/Chest: position differs by 0.0031 (tolerance 0.001)"
    [[nodiscard]] std::string toString() const;
};

// Compares `actual` with `expected`: headers (scenario, stepsPerSecond,
// bodies, step count) must match exactly; then step by step, body by body.
// Rotations compare as the angle between them, so q and -q are equal.
std::optional<Difference> compareRecordings(const Recording& actual, const Recording& expected,
                                            const Tolerances& tolerances = {});

} // namespace opas::sandbox
