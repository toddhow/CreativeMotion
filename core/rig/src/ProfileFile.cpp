#include "opas/rig/ProfileFile.hpp"

#include "Problem.hpp"
#include "opas/format/FileReader.hpp"
#include "opas/format/FileWriter.hpp"

#include <utility>

namespace opas::rig {
namespace {

using format::detail::FileReader;
using format::detail::Json;
using format::detail::jsonString;
using format::detail::number;

constexpr std::string_view kFormatName = "opas.profile";

// The profile rules (docs/formats/profile.md) on top of the shared reader.
// Types and finiteness are checked while reading; ranges and duplicate
// overrides by detail::findProfileProblem on the complete profile.
class ProfileReader : public FileReader {
public:
    using FileReader::FileReader;

    bool readMotor(const Json& parent, const std::string& parentPointer, MotorGains& out)
    {
        const std::string pointer = parentPointer + "/motor";
        const Json* motor = objectField(parent, parentPointer, "motor", "a motor object");
        return motor != nullptr &&
               onlyKeys(*motor, pointer,
                        {"stiffnessNmPerRad", "dampingNmsPerRad", "maxTorqueNm"}) &&
               numberField(*motor, pointer, "stiffnessNmPerRad", out.stiffnessNmPerRad) &&
               numberField(*motor, pointer, "dampingNmsPerRad", out.dampingNmsPerRad) &&
               numberField(*motor, pointer, "maxTorqueNm", out.maxTorqueNm);
    }

    bool readMaterial(const Json& parent, const std::string& parentPointer, Material& out)
    {
        const std::string pointer = parentPointer + "/material";
        const Json* material = objectField(parent, parentPointer, "material", "a material object");
        return material != nullptr &&
               onlyKeys(*material, pointer,
                        {"friction", "restitution", "linearDamping", "angularDamping"}) &&
               numberField(*material, pointer, "friction", out.friction) &&
               numberField(*material, pointer, "restitution", out.restitution) &&
               numberField(*material, pointer, "linearDamping", out.linearDamping) &&
               numberField(*material, pointer, "angularDamping", out.angularDamping);
    }

    bool readJointOverride(const Json& value, const std::string& pointer, JointOverride& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a joint override object");
        }
        return onlyKeys(value, pointer, {"bone", "motor"}) &&
               stringField(value, pointer, "bone", out.bone) &&
               readMotor(value, pointer, out.motor);
    }

    bool readBodyOverride(const Json& value, const std::string& pointer, BodyOverride& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a body override object");
        }
        return onlyKeys(value, pointer, {"bone", "material"}) &&
               stringField(value, pointer, "bone", out.bone) &&
               readMaterial(value, pointer, out.material);
    }

    bool readDocument(const Json& root, Profile& out)
    {
        if (!readHeader(root, kFormatName,
                        {"format", "version", "rig", "defaults", "joints", "bodies"}) ||
            !stringField(root, "", "rig", out.rig)) {
            return false;
        }
        const Json* defaults = objectField(root, "", "defaults", "a defaults object");
        if (defaults == nullptr || !onlyKeys(*defaults, "/defaults", {"motor", "material"}) ||
            !readMotor(*defaults, "/defaults", out.defaultMotor) ||
            !readMaterial(*defaults, "/defaults", out.defaultMaterial)) {
            return false;
        }
        const auto joint = [this](const Json& v, const std::string& p, JointOverride& o) {
            return readJointOverride(v, p, o);
        };
        const auto body = [this](const Json& v, const std::string& p, BodyOverride& o) {
            return readBodyOverride(v, p, o);
        };
        if (!readArray(root, "joints", out.joints, joint) ||
            !readArray(root, "bodies", out.bodies, body)) {
            return false;
        }
        if (const auto problem = detail::findProfileProblem(out)) {
            return failAt(root, problem->pointer, problem->message);
        }
        return true;
    }
};

std::string motorText(const MotorGains& m)
{
    return "{\"stiffnessNmPerRad\": " + number(m.stiffnessNmPerRad) +
           ", \"dampingNmsPerRad\": " + number(m.dampingNmsPerRad) +
           ", \"maxTorqueNm\": " + number(m.maxTorqueNm) + "}";
}

std::string materialText(const Material& m)
{
    return "{\"friction\": " + number(m.friction) + ", \"restitution\": " + number(m.restitution) +
           ", \"linearDamping\": " + number(m.linearDamping) +
           ", \"angularDamping\": " + number(m.angularDamping) + "}";
}

} // namespace

ProfileLoadResult loadProfile(std::string_view text, std::string_view fileName)
{
    ProfileLoadResult result;
    Profile profile;
    const std::optional<format::LoadError> error = format::detail::parseAndRead<ProfileReader>(
        text, fileName,
        [&profile](ProfileReader& r, const Json& root) { return r.readDocument(root, profile); });
    if (error) {
        result.error = *error;
    } else {
        result.value = std::move(profile);
    }
    return result;
}

ProfileLoadResult loadProfileFile(const std::filesystem::path& path)
{
    ProfileLoadResult result;
    const std::optional<std::string> text = format::detail::readTextFile(path, result.error);
    if (!text) {
        return result;
    }
    return loadProfile(*text, format::detail::utf8FileName(path));
}

std::string writeProfile(const Profile& profile)
{
    std::string out = "{\n  \"format\": \"opas.profile\",\n  \"version\": 1,\n  \"rig\": " +
                      jsonString(profile.rig) + ",\n";
    out += "  \"defaults\": {\n    \"motor\": " + motorText(profile.defaultMotor) +
           ",\n    \"material\": " + materialText(profile.defaultMaterial) + "\n  },\n";
    out += "  \"joints\": " +
           format::detail::arrayBlock(profile.joints,
                                      [](const JointOverride& o) {
                                          return "    {\"bone\": " + jsonString(o.bone) +
                                                 ", \"motor\": " + motorText(o.motor) + "}";
                                      }) +
           ",\n";
    out += "  \"bodies\": " + format::detail::arrayBlock(profile.bodies, [](const BodyOverride& o) {
               return "    {\"bone\": " + jsonString(o.bone) +
                      ", \"material\": " + materialText(o.material) + "}";
           });
    out += "\n}\n";
    return out;
}

} // namespace opas::rig
