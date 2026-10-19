#pragma once

// Internal: the shared JSON reader with the helpers the rig and profile
// readers both use. Not a public header (it carries nlohmann/json).

#include "Problem.hpp"

#include "opas/format/JsonSupport.hpp"

#include <cstdint>
#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace opas::rig::detail {

using format::detail::Json;

class FileReader : public format::detail::JsonReader {
public:
    using JsonReader::JsonReader;

    // Checks the document is an object with only `allowedKeys`, the right
    // "format" string and version 1.
    bool readHeader(const Json& root, std::string_view formatName,
                    std::initializer_list<std::string_view> allowedKeys)
    {
        if (!root.is_object()) {
            return fail(root, "", "the document must be a JSON object");
        }
        if (!onlyKeys(root, "", allowedKeys)) {
            return false;
        }
        const Json* format = member(root, "", "format");
        if (format == nullptr) {
            return false;
        }
        if (!format->is_string() || format->get<std::string>() != formatName) {
            return fail(*format, "/format", "must be \"" + std::string(formatName) + "\"");
        }
        const Json* version = member(root, "", "version");
        if (version == nullptr) {
            return false;
        }
        if (!version->is_number_integer() || version->get<std::int64_t>() != 1) {
            return fail(*version, "/version", "unsupported version (expected 1)");
        }
        return true;
    }

    bool stringField(const Json& object, const std::string& pointer, std::string_view key,
                     std::string& out)
    {
        const Json* value = member(object, pointer, key);
        return value != nullptr && readString(*value, pointer + "/" + std::string(key), out);
    }

    // Like stringField, but "" is allowed (a bone's parent; "" marks a root).
    bool stringOrEmptyField(const Json& object, const std::string& pointer, std::string_view key,
                            std::string& out)
    {
        const Json* value = member(object, pointer, key);
        if (value == nullptr) {
            return false;
        }
        if (!value->is_string() || value->get<std::string>() == format::detail::kNonFiniteValue) {
            return fail(*value, pointer + "/" + std::string(key), "must be a string");
        }
        out = value->get<std::string>();
        return true;
    }

    bool numberField(const Json& object, const std::string& pointer, std::string_view key,
                     double& out)
    {
        const Json* value = member(object, pointer, key);
        return value != nullptr && readNumber(*value, pointer + "/" + std::string(key), out);
    }

    bool vec3Field(const Json& object, const std::string& pointer, std::string_view key,
                   math::Vec3& out)
    {
        const Json* value = member(object, pointer, key);
        return value != nullptr && readVec3(*value, pointer + "/" + std::string(key), out);
    }

    bool quatField(const Json& object, const std::string& pointer, std::string_view key,
                   math::Quat& out)
    {
        const Json* value = member(object, pointer, key);
        return value != nullptr && readUnitQuat(*value, pointer + "/" + std::string(key), out);
    }

    // Finds a required object member; `what` names it in the error.
    const Json* objectField(const Json& object, const std::string& pointer, std::string_view key,
                            std::string_view what)
    {
        const Json* value = member(object, pointer, key);
        if (value == nullptr) {
            return nullptr;
        }
        if (!value->is_object()) {
            fail(*value, pointer + "/" + std::string(key), "must be " + std::string(what));
            return nullptr;
        }
        return value;
    }

    // Reads a required top-level array and each element with `readItem`.
    template <typename Item, typename ReadItem>
    bool readArray(const Json& root, std::string_view key, std::vector<Item>& out,
                   ReadItem readItem)
    {
        const std::string pointer = "/" + std::string(key);
        const Json* array = member(root, "", key);
        if (array == nullptr) {
            return false;
        }
        if (!array->is_array()) {
            return fail(*array, pointer, "must be an array");
        }
        out.resize(array->size());
        for (std::size_t i = 0; i < array->size(); ++i) {
            if (!readItem((*array)[i], pointer + "/" + std::to_string(i), out[i])) {
                return false;
            }
        }
        return true;
    }

    // Reports a rule found after reading (a Problem) at the line of the field
    // its pointer names, or of the document if the pointer names no value.
    bool failAt(const Json& root, const Problem& problem)
    {
        const Json::json_pointer pointer(problem.pointer);
        const Json& value = root.contains(pointer) ? root.at(pointer) : root;
        return fail(value, problem.pointer, problem.message);
    }
};

} // namespace opas::rig::detail
