#include "opas/sandbox/Recording.hpp"

#include "opas/format/FileReader.hpp"
#include "opas/math/Quat.hpp"
#include "opas/math/Vec3.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <set>
#include <utility>

namespace opas::sandbox {
namespace {

using format::detail::FileReader;
using format::detail::floatNumber;
using format::detail::Json;
using format::detail::jsonString;

constexpr std::string_view kFormatName = "opas.recording";
// Doubles below this magnitude round to a finite float (the largest float
// plus half the spacing between floats at that magnitude, 2^103).
constexpr double kFloatRoundingLimit = 0x1.ffffffp+127; // 0x1.fffffep+127 + 2^103

class RecordingReader : public FileReader {
public:
    using FileReader::FileReader;

    // One body sample: 13 finite numbers within float range. Pointers are
    // built only when something fails: a recording holds ~50k numbers.
    bool readSample(const Json& value, std::size_t step, std::size_t body, BodySample& out)
    {
        const auto at = [&](std::size_t i) {
            std::string pointer = "/steps/" + std::to_string(step) + "/" + std::to_string(body);
            if (i < kBodySampleSize) {
                pointer += "/" + std::to_string(i);
            }
            return pointer;
        };
        if (!value.is_array() || value.size() != kBodySampleSize) {
            return fail(value, at(kBodySampleSize), "must be an array of 13 numbers");
        }
        for (std::size_t i = 0; i < kBodySampleSize; ++i) {
            const Json& number = value[i];
            if (number.is_string() &&
                number.get_ref<const std::string&>() == format::detail::kNonFiniteValue) {
                return fail(number, at(i), "must be finite");
            }
            if (!number.is_number()) {
                return fail(number, at(i), "must be a number");
            }
            const double d = number.get<double>();
            // The shortest text of the largest float reads back slightly above
            // it as a double, and still rounds to it: anything below the
            // largest float plus half its spacing (2^103) is a float.
            if (!std::isfinite(d) || !(std::abs(d) < kFloatRoundingLimit)) {
                return fail(number, at(i), "must be a finite float");
            }
            out[i] = static_cast<float>(d);
        }
        return true;
    }

    bool readDocument(const Json& root, Recording& out)
    {
        if (!readHeader(root, kFormatName,
                        {"format", "version", "scenario", "stepsPerSecond", "bodies", "steps"}) ||
            !stringField(root, "", "scenario", out.scenario)) {
            return false;
        }
        const Json* rate = member(root, "", "stepsPerSecond");
        if (rate == nullptr || !readInt(*rate, "/stepsPerSecond", out.stepsPerSecond)) {
            return false;
        }
        if (out.stepsPerSecond <= 0) {
            return fail(*rate, "/stepsPerSecond", "must be > 0");
        }
        std::set<std::string> names;
        const auto body = [&](const Json& v, const std::string& p, std::string& o) {
            if (!readString(v, p, o)) {
                return false;
            }
            return names.insert(o).second || fail(v, p, "duplicate body \"" + o + "\"");
        };
        if (!readArray(root, "bodies", out.bodies, body)) {
            return false;
        }
        if (out.bodies.empty()) {
            return fail(root.at("bodies"), "/bodies", "needs at least one body");
        }
        const Json* steps = member(root, "", "steps");
        if (steps == nullptr) {
            return false;
        }
        if (!steps->is_array() || steps->empty()) {
            return fail(*steps, "/steps",
                        "must be a non-empty array (entry 0 is the initial state)");
        }
        out.steps.resize(steps->size());
        for (std::size_t s = 0; s < steps->size(); ++s) {
            const Json& step = (*steps)[s];
            if (!step.is_array() || step.size() != out.bodies.size()) {
                return fail(step, "/steps/" + std::to_string(s),
                            "must hold one sample per body (" + std::to_string(out.bodies.size()) +
                                ")");
            }
            out.steps[s].resize(out.bodies.size());
            for (std::size_t b = 0; b < out.bodies.size(); ++b) {
                if (!readSample(step[b], s, b, out.steps[s][b])) {
                    return false;
                }
            }
        }
        return true;
    }
};

std::string sampleText(const BodySample& sample)
{
    std::string out = "[";
    for (std::size_t i = 0; i < kBodySampleSize; ++i) {
        out += (i > 0 ? ", " : "") + floatNumber(sample[i]);
    }
    out += ']';
    return out;
}

math::Vec3 vecAt(const BodySample& s, std::size_t first)
{
    return {s[first], s[first + 1], s[first + 2]};
}

math::Quat rotationOf(const BodySample& s)
{
    return {s[3], s[4], s[5], s[6]};
}

} // namespace

RecordingLoadResult loadRecording(std::string_view text, std::string_view fileName)
{
    RecordingLoadResult result;
    Recording recording;
    const std::optional<format::LoadError> error = format::detail::parseAndRead<RecordingReader>(
        text, fileName, [&recording](RecordingReader& r, const Json& root) {
            return r.readDocument(root, recording);
        });
    if (error) {
        result.error = *error;
    } else {
        result.value = std::move(recording);
    }
    return result;
}

RecordingLoadResult loadRecordingFile(const std::filesystem::path& path)
{
    RecordingLoadResult result;
    const std::optional<std::string> text = format::detail::readTextFile(path, result.error);
    if (!text) {
        return result;
    }
    return loadRecording(*text, format::detail::utf8FileName(path));
}

std::string writeRecording(const Recording& recording)
{
    std::string out = "{\n  \"format\": \"opas.recording\",\n  \"version\": 1,\n  \"scenario\": " +
                      jsonString(recording.scenario) + ",\n";
    out += "  \"stepsPerSecond\": " + std::to_string(recording.stepsPerSecond) + ",\n";
    out += "  \"bodies\": [";
    for (std::size_t i = 0; i < recording.bodies.size(); ++i) {
        out += (i > 0 ? ", " : "") + jsonString(recording.bodies[i]);
    }
    out += "],\n  \"steps\": [";
    for (std::size_t s = 0; s < recording.steps.size(); ++s) {
        out += s > 0 ? ",\n    [" : "\n    [";
        for (std::size_t b = 0; b < recording.steps[s].size(); ++b) {
            out += (b > 0 ? ", " : "") + sampleText(recording.steps[s][b]);
        }
        out += ']';
    }
    out += recording.steps.empty() ? "]\n}\n" : "\n  ]\n}\n";
    return out;
}

bool sameBits(const Recording& a, const Recording& b)
{
    if (a.scenario != b.scenario || a.stepsPerSecond != b.stepsPerSecond || a.bodies != b.bodies ||
        a.steps.size() != b.steps.size()) {
        return false;
    }
    for (std::size_t s = 0; s < a.steps.size(); ++s) {
        if (a.steps[s].size() != b.steps[s].size()) {
            return false;
        }
        for (std::size_t body = 0; body < a.steps[s].size(); ++body) {
            for (std::size_t i = 0; i < kBodySampleSize; ++i) {
                if (std::bit_cast<std::uint32_t>(a.steps[s][body][i]) !=
                    std::bit_cast<std::uint32_t>(b.steps[s][body][i])) {
                    return false;
                }
            }
        }
    }
    return true;
}

std::string Difference::toString() const
{
    if (body.empty()) {
        return "header: " + field + " differs";
    }
    return std::format("step {}, {}: {} differs by {:.6g} (tolerance {:.6g})", step, body, field,
                       amount, tolerance);
}

std::optional<Difference> compareRecordings(const Recording& actual, const Recording& expected,
                                            const Tolerances& tolerances)
{
    const auto header = [](std::string field) {
        Difference d;
        d.field = std::move(field);
        return d;
    };
    if (actual.scenario != expected.scenario) {
        return header("scenario");
    }
    if (actual.stepsPerSecond != expected.stepsPerSecond) {
        return header("stepsPerSecond");
    }
    if (actual.bodies != expected.bodies) {
        return header("bodies");
    }
    if (actual.steps.size() != expected.steps.size()) {
        return header("step count");
    }
    for (std::size_t s = 0; s < actual.steps.size(); ++s) {
        for (std::size_t b = 0; b < actual.bodies.size(); ++b) {
            const BodySample& a = actual.steps[s][b];
            const BodySample& e = expected.steps[s][b];
            const struct {
                const char* field;
                double amount;
                double tolerance;
            } checks[] = {
                {"position", math::distance(vecAt(a, 0), vecAt(e, 0)), tolerances.positionM},
                {"rotation", math::angleBetween(rotationOf(a), rotationOf(e)),
                 tolerances.rotationRad},
                {"linearVelocity", math::distance(vecAt(a, 7), vecAt(e, 7)),
                 tolerances.linearVelocityMps},
                {"angularVelocity", math::distance(vecAt(a, 10), vecAt(e, 10)),
                 tolerances.angularVelocityRadps},
            };
            for (const auto& check : checks) {
                // Written as !(<=) so a NaN difference fails.
                if (!(check.amount <= check.tolerance)) {
                    return Difference{s, actual.bodies[b], check.field, check.amount,
                                      check.tolerance};
                }
            }
        }
    }
    return std::nullopt;
}

} // namespace opas::sandbox
