#include "opas/sandbox/Runner.hpp"
#include "opas/sandbox/Scenario.hpp"

#include "opas/format/FileReader.hpp"
#include "opas/format/FileWriter.hpp"

#include <cmath>
#include <set>
#include <utility>

namespace opas::sandbox {
namespace {

using format::detail::FileReader;
using format::detail::Json;
using format::detail::jsonString;
using format::detail::number;
using format::detail::quat;
using format::detail::vec3;

constexpr std::string_view kFormatName = "opas.scenario";
// Limits that keep a scenario a test-sized run (docs/formats/scenario.md).
constexpr int kMaxStepsPerSecond = 10000;
constexpr double kMaxDurationS = 60.0;

std::string rootMotionName(motor::RootMotion motion)
{
    return motion == motor::RootMotion::Kinematic ? "kinematic" : "dynamic";
}

std::string motorModeName(MotorMode mode)
{
    return mode == MotorMode::Off ? "off" : "profile";
}

// The scenario rules (docs/formats/scenario.md) on top of the shared reader.
// Each entry is checked as it is read, in file order.
class ScenarioReader : public FileReader {
public:
    using FileReader::FileReader;

    // A required string field that must be one of `choices`.
    bool choiceField(const Json& object, const std::string& pointer, std::string_view key,
                     std::initializer_list<std::string_view> choices, std::string& out)
    {
        if (!stringField(object, pointer, key, out)) {
            return false;
        }
        for (const std::string_view choice : choices) {
            if (out == choice) {
                return true;
            }
        }
        std::string list;
        for (const std::string_view choice : choices) {
            list += (list.empty() ? "\"" : ", \"") + std::string(choice) + "\"";
        }
        return fail(object.at(std::string(key)), pointer + "/" + std::string(key),
                    "must be one of " + list);
    }

    // Records `name`; a repeat is reported at the element's "name".
    bool uniqueName(std::set<std::string>& seen, const std::string& name, const Json& element,
                    const std::string& pointer, std::string_view what)
    {
        if (seen.insert(name).second) {
            return true;
        }
        return fail(element.at("name"), pointer + "/name",
                    "duplicate " + std::string(what) + " name \"" + name + "\"");
    }

    bool readStatic(const Json& value, const std::string& pointer, StaticBox& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a static box object");
        }
        out.line = lineAt(value.start_pos());
        if (!onlyKeys(value, pointer,
                      {"name", "halfExtentsM", "translationM", "rotation", "friction"}) ||
            !stringField(value, pointer, "name", out.name) ||
            !vec3Field(value, pointer, "halfExtentsM", out.halfExtentsM) ||
            !vec3Field(value, pointer, "translationM", out.pose.translation) ||
            !quatField(value, pointer, "rotation", out.pose.rotation) ||
            !numberField(value, pointer, "friction", out.friction)) {
            return false;
        }
        const math::Vec3 e = out.halfExtentsM;
        if (!(e.x > 0.0 && e.y > 0.0 && e.z > 0.0)) {
            return fail(value.at("halfExtentsM"), pointer + "/halfExtentsM",
                        "each half extent must be > 0");
        }
        if (!(out.friction >= 0.0)) {
            return fail(value.at("friction"), pointer + "/friction", "must be >= 0");
        }
        return true;
    }

    bool readRagdoll(const Json& value, const std::string& pointer, ScenarioRagdoll& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a ragdoll object");
        }
        out.line = lineAt(value.start_pos());
        std::string rootMotion;
        std::string motors;
        if (!onlyKeys(value, pointer,
                      {"name", "rig", "profile", "translationM", "rotation", "rootMotion", "motors",
                       "pose"}) ||
            !stringField(value, pointer, "name", out.name) ||
            !stringField(value, pointer, "rig", out.rigPath) ||
            !stringField(value, pointer, "profile", out.profilePath) ||
            !vec3Field(value, pointer, "translationM", out.root.translation) ||
            !quatField(value, pointer, "rotation", out.root.rotation) ||
            !choiceField(value, pointer, "rootMotion", {"dynamic", "kinematic"}, rootMotion) ||
            !choiceField(value, pointer, "motors", {"profile", "off"}, motors) ||
            !stringField(value, pointer, "pose", out.pose)) {
            return false;
        }
        out.rootMotion =
            rootMotion == "kinematic" ? motor::RootMotion::Kinematic : motor::RootMotion::Dynamic;
        out.motors = motors == "off" ? MotorMode::Off : MotorMode::Profile;
        if (!isPoseSourceName(out.pose)) {
            return fail(value.at("pose"), pointer + "/pose",
                        "unknown pose source \"" + out.pose +
                            "\" (expected \"rest\" or \"armWave\")");
        }
        return true;
    }

    bool readImpulse(const Json& value, const std::string& pointer, Impulse& out,
                     const Scenario& scenario)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be an impulse object");
        }
        out.line = lineAt(value.start_pos());
        if (!onlyKeys(value, pointer, {"atS", "ragdoll", "bone", "impulseNs", "offsetM"}) ||
            !numberField(value, pointer, "atS", out.atS) ||
            !stringField(value, pointer, "ragdoll", out.ragdoll) ||
            !stringField(value, pointer, "bone", out.bone) ||
            !vec3Field(value, pointer, "impulseNs", out.impulseNs) ||
            !vec3Field(value, pointer, "offsetM", out.offsetM)) {
            return false;
        }
        if (!(out.atS >= 0.0 && out.atS < scenario.durationS)) {
            return fail(value.at("atS"), pointer + "/atS", "must be in [0, durationS)");
        }
        // An impulse due after the last step would silently never fire.
        if (impulseStep(scenario, out.atS) >= stepCount(scenario)) {
            return fail(value.at("atS"), pointer + "/atS",
                        "falls after the last step (the run has " +
                            std::to_string(stepCount(scenario)) + " steps)");
        }
        for (const ScenarioRagdoll& ragdoll : scenario.ragdolls) {
            if (ragdoll.name == out.ragdoll) {
                return true;
            }
        }
        return fail(value.at("ragdoll"), pointer + "/ragdoll",
                    "unknown ragdoll \"" + out.ragdoll + "\"");
    }

    bool readDocument(const Json& root, Scenario& out)
    {
        if (!readHeader(root, kFormatName,
                        {"format", "version", "name", "stepsPerSecond", "durationS", "gravityMps2",
                         "statics", "ragdolls", "impulses"}) ||
            !stringField(root, "", "name", out.name)) {
            return false;
        }
        const Json* rate = member(root, "", "stepsPerSecond");
        if (rate == nullptr || !readInt(*rate, "/stepsPerSecond", out.stepsPerSecond)) {
            return false;
        }
        if (out.stepsPerSecond <= 0 || out.stepsPerSecond > kMaxStepsPerSecond) {
            return fail(*rate, "/stepsPerSecond", "must be in [1, 10000]");
        }
        if (!numberField(root, "", "durationS", out.durationS)) {
            return false;
        }
        if (!(out.durationS > 0.0 && out.durationS <= kMaxDurationS)) {
            return fail(root.at("durationS"), "/durationS", "must be in (0, 60]");
        }
        if (stepCount(out) < 1) {
            return fail(root.at("durationS"), "/durationS",
                        "is shorter than half a step: the run would have no steps");
        }
        if (!vec3Field(root, "", "gravityMps2", out.gravityMps2)) {
            return false;
        }

        std::set<std::string> staticNames;
        const auto box = [&](const Json& v, const std::string& p, StaticBox& o) {
            return readStatic(v, p, o) && uniqueName(staticNames, o.name, v, p, "static");
        };
        std::set<std::string> ragdollNames;
        const auto ragdoll = [&](const Json& v, const std::string& p, ScenarioRagdoll& o) {
            return readRagdoll(v, p, o) && uniqueName(ragdollNames, o.name, v, p, "ragdoll");
        };
        const auto impulse = [&](const Json& v, const std::string& p, Impulse& o) {
            return readImpulse(v, p, o, out);
        };
        if (!readArray(root, "statics", out.statics, box) ||
            !readArray(root, "ragdolls", out.ragdolls, ragdoll)) {
            return false;
        }
        if (out.ragdolls.empty()) {
            return fail(root.at("ragdolls"), "/ragdolls", "needs at least one ragdoll");
        }
        return readArray(root, "impulses", out.impulses, impulse);
    }
};

} // namespace

int stepCount(const Scenario& scenario)
{
    return static_cast<int>(std::lround(scenario.durationS * scenario.stepsPerSecond));
}

int impulseStep(const Scenario& scenario, double atS)
{
    // The 1e-9 keeps a time written as an exact step boundary (0.5 s at 60 Hz
    // is step 30) from rounding up to the next step.
    return static_cast<int>(std::ceil(atS * scenario.stepsPerSecond - 1e-9));
}

ScenarioLoadResult loadScenario(std::string_view text, std::string_view fileName)
{
    ScenarioLoadResult result;
    Scenario scenario;
    const std::optional<format::LoadError> error = format::detail::parseAndRead<ScenarioReader>(
        text, fileName, [&scenario](ScenarioReader& r, const Json& root) {
            return r.readDocument(root, scenario);
        });
    if (error) {
        result.error = *error;
    } else {
        result.value = std::move(scenario);
    }
    return result;
}

ScenarioLoadResult loadScenarioFile(const std::filesystem::path& path)
{
    ScenarioLoadResult result;
    const std::optional<std::string> text = format::detail::readTextFile(path, result.error);
    if (!text) {
        return result;
    }
    return loadScenario(*text, format::detail::utf8FileName(path));
}

std::string writeScenario(const Scenario& scenario)
{
    std::string out = "{\n  \"format\": \"opas.scenario\",\n  \"version\": 1,\n  \"name\": " +
                      jsonString(scenario.name) + ",\n";
    out += "  \"stepsPerSecond\": " + std::to_string(scenario.stepsPerSecond) + ",\n";
    out += "  \"durationS\": " + number(scenario.durationS) + ",\n";
    out += "  \"gravityMps2\": " + vec3(scenario.gravityMps2) + ",\n";
    out += "  \"statics\": " +
           format::detail::arrayBlock(scenario.statics,
                                      [](const StaticBox& s) {
                                          return "    {\"name\": " + jsonString(s.name) +
                                                 ", \"halfExtentsM\": " + vec3(s.halfExtentsM) +
                                                 ", \"translationM\": " + vec3(s.pose.translation) +
                                                 ", \"rotation\": " + quat(s.pose.rotation) +
                                                 ", \"friction\": " + number(s.friction) + "}";
                                      }) +
           ",\n";
    out += "  \"ragdolls\": " +
           format::detail::arrayBlock(
               scenario.ragdolls,
               [](const ScenarioRagdoll& r) {
                   return "    {\n      \"name\": " + jsonString(r.name) +
                          ",\n      \"rig\": " + jsonString(r.rigPath) +
                          ",\n      \"profile\": " + jsonString(r.profilePath) +
                          ",\n      \"translationM\": " + vec3(r.root.translation) +
                          ",\n      \"rotation\": " + quat(r.root.rotation) +
                          ",\n      \"rootMotion\": " + jsonString(rootMotionName(r.rootMotion)) +
                          ",\n      \"motors\": " + jsonString(motorModeName(r.motors)) +
                          ",\n      \"pose\": " + jsonString(r.pose) + "\n    }";
               }) +
           ",\n";
    out += "  \"impulses\": " + format::detail::arrayBlock(scenario.impulses, [](const Impulse& i) {
               return "    {\"atS\": " + number(i.atS) + ", \"ragdoll\": " + jsonString(i.ragdoll) +
                      ", \"bone\": " + jsonString(i.bone) +
                      ", \"impulseNs\": " + vec3(i.impulseNs) +
                      ", \"offsetM\": " + vec3(i.offsetM) + "}";
           });
    out += "\n}\n";
    return out;
}

} // namespace opas::sandbox
