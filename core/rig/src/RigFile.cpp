#include "opas/rig/RigFile.hpp"

#include "FileReader.hpp"
#include "FileWriter.hpp"

#include <utility>
#include <vector>

namespace opas::rig {
namespace {

using detail::FileReader;
using detail::Json;
using format::detail::jsonString;
using format::detail::number;
using format::detail::quat;
using format::detail::vec3;

constexpr std::string_view kFormatName = "opas.rig";

// A bone as written in the file; the parent is resolved to an index once
// every bone has been read, so a parent listed too late can be told apart
// from an unknown one.
struct FileBone {
    std::string name;
    std::string parent;
    math::Transform rest;
};

// The rig rules (docs/formats/rig.md) on top of the shared reader. Types,
// finiteness and unit rotations are checked while reading; names, parents and
// every other rule are checked on the complete rig by detail::findRigProblem,
// so loaded files and programmatic rigs follow exactly the same rules.
class RigReader : public FileReader {
public:
    using FileReader::FileReader;

    bool readBone(const Json& value, const std::string& pointer, FileBone& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a bone object");
        }
        return onlyKeys(value, pointer, {"name", "parent", "translationM", "rotation"}) &&
               stringField(value, pointer, "name", out.name) &&
               stringOrEmptyField(value, pointer, "parent", out.parent) &&
               vec3Field(value, pointer, "translationM", out.rest.translation) &&
               quatField(value, pointer, "rotation", out.rest.rotation);
    }

    bool readShape(const Json& body, const std::string& bodyPointer, Shape& out)
    {
        const std::string pointer = bodyPointer + "/shape";
        const Json* shape = objectField(body, bodyPointer, "shape", "a shape object");
        if (shape == nullptr) {
            return false;
        }
        std::string type;
        if (!stringField(*shape, pointer, "type", type)) {
            return false;
        }
        if (type == "capsule") {
            CapsuleShape capsule;
            if (!onlyKeys(*shape, pointer, {"type", "halfHeightM", "radiusM"}) ||
                !numberField(*shape, pointer, "halfHeightM", capsule.halfHeightM) ||
                !numberField(*shape, pointer, "radiusM", capsule.radiusM)) {
                return false;
            }
            out = capsule;
        } else if (type == "box") {
            BoxShape box;
            if (!onlyKeys(*shape, pointer, {"type", "halfExtentsM"}) ||
                !vec3Field(*shape, pointer, "halfExtentsM", box.halfExtentsM)) {
                return false;
            }
            out = box;
        } else if (type == "sphere") {
            SphereShape sphere;
            if (!onlyKeys(*shape, pointer, {"type", "radiusM"}) ||
                !numberField(*shape, pointer, "radiusM", sphere.radiusM)) {
                return false;
            }
            out = sphere;
        } else {
            return fail(shape->at("type"), pointer + "/type",
                        "must be \"capsule\", \"box\" or \"sphere\"");
        }
        return true;
    }

    bool readBody(const Json& value, const std::string& pointer, Body& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a body object");
        }
        return onlyKeys(value, pointer, {"bone", "shape", "translationM", "rotation", "massKg"}) &&
               stringField(value, pointer, "bone", out.bone) &&
               readShape(value, pointer, out.shape) &&
               vec3Field(value, pointer, "translationM", out.local.translation) &&
               quatField(value, pointer, "rotation", out.local.rotation) &&
               numberField(value, pointer, "massKg", out.massKg);
    }

    bool readJoint(const Json& value, const std::string& pointer, Joint& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a joint object");
        }
        return onlyKeys(value, pointer,
                        {"bone", "twistAxis", "planeAxis", "normalHalfConeRad", "planeHalfConeRad",
                         "twistMinRad", "twistMaxRad"}) &&
               stringField(value, pointer, "bone", out.bone) &&
               vec3Field(value, pointer, "twistAxis", out.twistAxis) &&
               vec3Field(value, pointer, "planeAxis", out.planeAxis) &&
               numberField(value, pointer, "normalHalfConeRad", out.normalHalfConeRad) &&
               numberField(value, pointer, "planeHalfConeRad", out.planeHalfConeRad) &&
               numberField(value, pointer, "twistMinRad", out.twistMinRad) &&
               numberField(value, pointer, "twistMaxRad", out.twistMaxRad);
    }

    // Turns file bones into skeleton bones. Each parent must be a bone listed
    // earlier; duplicates are reported at their second occurrence.
    bool resolveBones(const Json& root, const std::vector<FileBone>& fileBones, Skeleton& out)
    {
        out.bones.resize(fileBones.size());
        for (std::size_t i = 0; i < fileBones.size(); ++i) {
            const FileBone& bone = fileBones[i];
            const std::string pointer = "/bones/" + std::to_string(i);
            for (std::size_t j = 0; j < i; ++j) {
                if (fileBones[j].name == bone.name) {
                    return fail(root.at(Json::json_pointer(pointer + "/name")), pointer + "/name",
                                "duplicate bone \"" + bone.name + "\"");
                }
            }
            int parent = -1;
            if (!bone.parent.empty()) {
                for (std::size_t j = 0; j < i && parent < 0; ++j) {
                    if (fileBones[j].name == bone.parent) {
                        parent = static_cast<int>(j);
                    }
                }
                if (parent < 0) {
                    return failParent(root, fileBones, i);
                }
            }
            out.bones[i] = Bone{bone.name, parent, bone.rest};
        }
        return true;
    }

    bool readDocument(const Json& root, Rig& out)
    {
        if (!readHeader(root, kFormatName,
                        {"format", "version", "name", "bones", "bodies", "joints"}) ||
            !stringField(root, "", "name", out.name)) {
            return false;
        }
        std::vector<FileBone> fileBones;
        const auto bone = [this](const Json& v, const std::string& p, FileBone& o) {
            return readBone(v, p, o);
        };
        const auto body = [this](const Json& v, const std::string& p, Body& o) {
            return readBody(v, p, o);
        };
        const auto joint = [this](const Json& v, const std::string& p, Joint& o) {
            return readJoint(v, p, o);
        };
        if (!readArray(root, "bones", fileBones, bone) ||
            !resolveBones(root, fileBones, out.skeleton) ||
            !readArray(root, "bodies", out.bodies, body) ||
            !readArray(root, "joints", out.joints, joint)) {
            return false;
        }
        if (const auto problem = detail::findRigProblem(out)) {
            return failAt(root, *problem);
        }
        return true;
    }

private:
    bool failParent(const Json& root, const std::vector<FileBone>& fileBones, std::size_t i)
    {
        const std::string pointer = "/bones/" + std::to_string(i) + "/parent";
        const Json& value = root.at(Json::json_pointer(pointer));
        const std::string& parent = fileBones[i].parent;
        if (parent == fileBones[i].name) {
            return fail(value, pointer, "a bone cannot be its own parent");
        }
        for (std::size_t j = i + 1; j < fileBones.size(); ++j) {
            if (fileBones[j].name == parent) {
                return fail(value, pointer,
                            "\"" + parent + "\" must be listed before this bone (parents first)");
            }
        }
        return fail(value, pointer, "unknown bone \"" + parent + "\"");
    }
};

std::string shapeText(const Shape& shape)
{
    if (const auto* capsule = std::get_if<CapsuleShape>(&shape)) {
        return "{\"type\": \"capsule\", \"halfHeightM\": " + number(capsule->halfHeightM) +
               ", \"radiusM\": " + number(capsule->radiusM) + "}";
    }
    if (const auto* box = std::get_if<BoxShape>(&shape)) {
        return "{\"type\": \"box\", \"halfExtentsM\": " + vec3(box->halfExtentsM) + "}";
    }
    const auto& sphere = std::get<SphereShape>(shape);
    return "{\"type\": \"sphere\", \"radiusM\": " + number(sphere.radiusM) + "}";
}

} // namespace

RigLoadResult loadRig(std::string_view text, std::string_view fileName)
{
    RigLoadResult result;
    Rig rig;
    const std::optional<format::LoadError> error = format::detail::parseAndRead<RigReader>(
        text, fileName,
        [&rig](RigReader& r, const Json& root) { return r.readDocument(root, rig); });
    if (error) {
        result.error = *error;
    } else {
        result.value = std::move(rig);
    }
    return result;
}

RigLoadResult loadRigFile(const std::filesystem::path& path)
{
    RigLoadResult result;
    const std::optional<std::string> text = format::detail::readTextFile(path, result.error);
    if (!text) {
        return result;
    }
    return loadRig(*text, format::detail::utf8FileName(path));
}

std::string writeRig(const Rig& rig)
{
    const std::vector<Bone>& bones = rig.skeleton.bones;
    std::string out =
        "{\n  \"format\": \"opas.rig\",\n  \"version\": 1,\n  \"name\": " + jsonString(rig.name) +
        ",\n";
    out += "  \"bones\": " +
           detail::arrayBlock(bones,
                              [&bones](const Bone& b) {
                                  const std::string parent =
                                      b.parent >= 0 &&
                                              static_cast<std::size_t>(b.parent) < bones.size()
                                          ? bones[static_cast<std::size_t>(b.parent)].name
                                          : std::string();
                                  return "    {\"name\": " + jsonString(b.name) +
                                         ", \"parent\": " + jsonString(parent) +
                                         ", \"translationM\": " + vec3(b.localRest.translation) +
                                         ", \"rotation\": " + quat(b.localRest.rotation) + "}";
                              }) +
           ",\n";
    out += "  \"bodies\": " +
           detail::arrayBlock(rig.bodies,
                              [](const Body& b) {
                                  return "    {\n      \"bone\": " + jsonString(b.bone) +
                                         ",\n      \"shape\": " + shapeText(b.shape) +
                                         ",\n      \"translationM\": " + vec3(b.local.translation) +
                                         ",\n      \"rotation\": " + quat(b.local.rotation) +
                                         ",\n      \"massKg\": " + number(b.massKg) + "\n    }";
                              }) +
           ",\n";
    out += "  \"joints\": " + detail::arrayBlock(rig.joints, [](const Joint& j) {
               return "    {\n      \"bone\": " + jsonString(j.bone) +
                      ",\n      \"twistAxis\": " + vec3(j.twistAxis) +
                      ",\n      \"planeAxis\": " + vec3(j.planeAxis) +
                      ",\n      \"normalHalfConeRad\": " + number(j.normalHalfConeRad) +
                      ",\n      \"planeHalfConeRad\": " + number(j.planeHalfConeRad) +
                      ",\n      \"twistMinRad\": " + number(j.twistMinRad) +
                      ",\n      \"twistMaxRad\": " + number(j.twistMaxRad) + "\n    }";
           });
    out += "\n}\n";
    return out;
}

} // namespace opas::rig
