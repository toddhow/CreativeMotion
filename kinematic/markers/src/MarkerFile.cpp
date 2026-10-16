#include "opas/markers/MarkerFile.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <set>
#include <sstream>

namespace opas::markers {
namespace {

using Json = nlohmann::json;

constexpr std::string_view kFormatName = "opas.markers";
// How far contactDirection may be from unit length (docs/formats/markers.md).
constexpr double kUnitTolerance = 1e-6;

// nlohmann/json rejects a number too large for a double (e.g. 1e999) with an
// exception that carries no position (out_of_range 406). To still report the
// line and field, such literals are replaced by this string and the text is
// parsed again; the replacement keeps every line intact, and the readers
// below report it as "must be finite". The leading NUL (written \u0000) means
// no real marker file contains it.
constexpr std::string_view kNonFiniteSentinel = "\"\\u0000opas-non-finite-number\"";
// The decoded sentinel: a NUL byte, then the name (23 bytes in all).
constexpr std::string_view kNonFiniteValue("\0opas-non-finite-number", 23);

// Returns the text with every number literal that overflows a double replaced
// by kNonFiniteSentinel. String contents are skipped.
std::string replaceOverflowingNumbers(std::string_view text)
{
    std::string out;
    out.reserve(text.size());
    std::size_t i = 0;
    while (i < text.size()) {
        const char c = text[i];
        if (c == '"') {
            // Copy the whole string literal, honoring backslash escapes.
            const std::size_t start = i++;
            while (i < text.size() && text[i] != '"') {
                i += text[i] == '\\' ? 2 : 1;
            }
            i = std::min(i + 1, text.size());
            out.append(text.substr(start, i - start));
        } else if (c == '-' || (c >= '0' && c <= '9')) {
            const std::size_t start = i;
            while (i < text.size() &&
                   std::string_view("+-0123456789.eE").find(text[i]) != std::string_view::npos) {
                ++i;
            }
            const std::string literal(text.substr(start, i - start));
            const double value = std::strtod(literal.c_str(), nullptr);
            if (std::isfinite(value)) {
                out += literal;
            } else {
                out += kNonFiniteSentinel;
            }
        } else {
            out += c;
            ++i;
        }
    }
    return out;
}

// JSON pointer escaping (RFC 6901): "~" -> "~0", "/" -> "~1".
std::string pointerToken(std::string_view key)
{
    std::string token;
    for (const char c : key) {
        if (c == '~') {
            token += "~0";
        } else if (c == '/') {
            token += "~1";
        } else {
            token += c;
        }
    }
    return token;
}

// Validates parsed JSON against the marker format, remembering the first
// problem with its line. Every read function returns false once an error is
// recorded, so callers can simply stop.
class Reader {
public:
    Reader(std::string_view text, std::string_view file) : text_(text), file_(file) {}

    [[nodiscard]] const LoadError& error() const { return error_; }

    // Line (1-based) of a byte offset in the text.
    [[nodiscard]] std::size_t lineAt(std::size_t offset) const
    {
        const std::size_t end = std::min(offset, text_.size());
        std::size_t line = 1;
        for (std::size_t i = 0; i < end; ++i) {
            if (text_[i] == '\n') {
                ++line;
            }
        }
        return line;
    }

    bool failAtLine(std::size_t line, std::string field, std::string message)
    {
        error_ = LoadError{std::string(file_), line, std::move(field), std::move(message)};
        return false;
    }

    // Records an error at the line where `value` starts (positions are kept by
    // nlohmann/json's JSON_DIAGNOSTIC_POSITIONS).
    bool fail(const Json& value, std::string field, std::string message)
    {
        const std::size_t start = value.start_pos();
        return failAtLine(start == std::string::npos ? 1 : lineAt(start), std::move(field),
                          std::move(message));
    }

    // Rejects keys outside `allowed` (catches typos).
    bool onlyKeys(const Json& object, const std::string& pointer,
                  std::initializer_list<std::string_view> allowed)
    {
        for (const auto& [key, value] : object.items()) {
            if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
                return fail(value, pointer + "/" + pointerToken(key),
                            "unknown key \"" + key + "\"");
            }
        }
        return true;
    }

    // Finds a required member; a missing one is reported at the object's line.
    const Json* member(const Json& object, const std::string& pointer, std::string_view key)
    {
        const auto it = object.find(key);
        if (it == object.end()) {
            fail(object, pointer + "/" + std::string(key), "missing required field");
            return nullptr;
        }
        return &*it;
    }

    bool readString(const Json& value, const std::string& pointer, std::string& out)
    {
        if (!value.is_string() || value.get<std::string>() == kNonFiniteValue) {
            return fail(value, pointer, "must be a string");
        }
        out = value.get<std::string>();
        if (out.empty()) {
            return fail(value, pointer, "must not be empty");
        }
        return true;
    }

    bool readNumber(const Json& value, const std::string& pointer, double& out)
    {
        if (value.is_string() && value.get<std::string>() == kNonFiniteValue) {
            return fail(value, pointer, "must be finite");
        }
        if (!value.is_number()) {
            return fail(value, pointer, "must be a number");
        }
        out = value.get<double>();
        if (!std::isfinite(out)) {
            return fail(value, pointer, "must be finite");
        }
        return true;
    }

    bool readInt(const Json& value, const std::string& pointer, int& out)
    {
        if (!value.is_number_integer()) {
            return fail(value, pointer, "must be an integer");
        }
        if (value.is_number_unsigned()) {
            const auto v = value.get<std::uint64_t>();
            if (v > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
                return fail(value, pointer, "is out of range");
            }
            out = static_cast<int>(v);
            return true;
        }
        const auto v = value.get<std::int64_t>();
        if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) {
            return fail(value, pointer, "is out of range");
        }
        out = static_cast<int>(v);
        return true;
    }

    bool readVec3(const Json& value, const std::string& pointer, math::Vec3& out)
    {
        if (!value.is_array() || value.size() != 3) {
            return fail(value, pointer, "must be an array of 3 numbers");
        }
        std::array<double, 3> v{};
        for (std::size_t i = 0; i < 3; ++i) {
            if (!readNumber(value[i], pointer + "/" + std::to_string(i), v[i])) {
                return false;
            }
        }
        out = math::Vec3{v[0], v[1], v[2]};
        return true;
    }

    bool readWindow(const Json& value, const std::string& pointer, TimeWindow& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be an object {startS, endS}");
        }
        if (!onlyKeys(value, pointer, {"startS", "endS"})) {
            return false;
        }
        const Json* start = member(value, pointer, "startS");
        if (start == nullptr || !readNumber(*start, pointer + "/startS", out.startS)) {
            return false;
        }
        const Json* end = member(value, pointer, "endS");
        if (end == nullptr || !readNumber(*end, pointer + "/endS", out.endS)) {
            return false;
        }
        if (out.startS < 0.0) {
            return fail(*start, pointer + "/startS", "must be >= 0");
        }
        if (out.startS > out.endS) {
            return fail(*start, pointer + "/startS", "must be <= endS");
        }
        return true;
    }

    bool readMarker(const Json& value, const std::string& pointer, InteractionMarker& out)
    {
        if (!value.is_object()) {
            return fail(value, pointer, "must be a marker object");
        }
        if (!onlyKeys(value, pointer,
                      {"id", "sourceBone", "targetRegion", "offsetM", "contactDirection", "window",
                       "priority", "toleranceM"})) {
            return false;
        }
        // Fields are checked in file order, so the first problem is reported.
        const Json* id = member(value, pointer, "id");
        if (id == nullptr || !readString(*id, pointer + "/id", out.id)) {
            return false;
        }
        const Json* sourceBone = member(value, pointer, "sourceBone");
        if (sourceBone == nullptr ||
            !readString(*sourceBone, pointer + "/sourceBone", out.sourceBone)) {
            return false;
        }
        const Json* targetRegion = member(value, pointer, "targetRegion");
        if (targetRegion == nullptr ||
            !readString(*targetRegion, pointer + "/targetRegion", out.targetRegion)) {
            return false;
        }
        const Json* offset = member(value, pointer, "offsetM");
        if (offset == nullptr || !readVec3(*offset, pointer + "/offsetM", out.offsetM)) {
            return false;
        }
        const Json* direction = member(value, pointer, "contactDirection");
        if (direction == nullptr ||
            !readVec3(*direction, pointer + "/contactDirection", out.contactDirection)) {
            return false;
        }
        if (std::abs(math::length(out.contactDirection) - 1.0) > kUnitTolerance) {
            return fail(*direction, pointer + "/contactDirection", "must be unit length");
        }
        const Json* window = member(value, pointer, "window");
        if (window == nullptr || !readWindow(*window, pointer + "/window", out.window)) {
            return false;
        }
        const Json* priority = member(value, pointer, "priority");
        if (priority == nullptr || !readInt(*priority, pointer + "/priority", out.priority)) {
            return false;
        }
        const Json* tolerance = member(value, pointer, "toleranceM");
        if (tolerance == nullptr ||
            !readNumber(*tolerance, pointer + "/toleranceM", out.toleranceM)) {
            return false;
        }
        if (!(out.toleranceM > 0.0)) {
            return fail(*tolerance, pointer + "/toleranceM", "must be > 0");
        }
        return true;
    }

    bool readDocument(const Json& root, MarkerSet& out)
    {
        if (!root.is_object()) {
            return fail(root, "", "the document must be a JSON object");
        }
        if (!onlyKeys(root, "", {"format", "version", "markers"})) {
            return false;
        }
        const Json* format = member(root, "", "format");
        if (format == nullptr) {
            return false;
        }
        if (!format->is_string() || format->get<std::string>() != kFormatName) {
            return fail(*format, "/format", "must be \"opas.markers\"");
        }
        const Json* version = member(root, "", "version");
        if (version == nullptr) {
            return false;
        }
        if (!version->is_number_integer() ||
            version->get<std::int64_t>() != MarkerSet::kFormatVersion) {
            return fail(*version, "/version", "unsupported version (expected 1)");
        }
        const Json* markers = member(root, "", "markers");
        if (markers == nullptr) {
            return false;
        }
        if (!markers->is_array()) {
            return fail(*markers, "/markers", "must be an array");
        }
        std::set<std::string> ids;
        for (std::size_t i = 0; i < markers->size(); ++i) {
            const std::string pointer = "/markers/" + std::to_string(i);
            InteractionMarker marker;
            if (!readMarker((*markers)[i], pointer, marker)) {
                return false;
            }
            if (!ids.insert(marker.id).second) {
                return fail((*markers)[i]["id"], pointer + "/id",
                            "duplicate id \"" + marker.id + "\"");
            }
            out.markers.push_back(std::move(marker));
        }
        return true;
    }

private:
    std::string_view text_;
    std::string_view file_;
    LoadError error_;
};

// --- Writing ------------------------------------------------------------------

std::string number(double value)
{
    // Shortest text that reads back as exactly the same double.
    std::array<char, 32> buffer{};
    const auto result = std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    return std::string(buffer.data(), result.ptr);
}

std::string jsonString(std::string_view text)
{
    std::string out = "\"";
    for (const char c : text) {
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        case '\b':
            out += "\\b";
            break;
        case '\f':
            out += "\\f";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                // Other control characters must be \u escapes in JSON.
                constexpr std::string_view hex = "0123456789abcdef";
                const auto u = static_cast<unsigned char>(c);
                out += "\\u00";
                out += hex[u >> 4U];
                out += hex[u & 0x0FU];
            } else {
                out += c; // UTF-8 bytes pass through unchanged
            }
        }
    }
    out += '"';
    return out;
}

std::string vec3(const math::Vec3& v)
{
    return "[" + number(v.x) + ", " + number(v.y) + ", " + number(v.z) + "]";
}

} // namespace

std::string LoadError::toString() const
{
    return file + ":" + std::to_string(line) + ": " + (field.empty() ? "(document)" : field) +
           ": " + message;
}

LoadResult loadMarkerSet(std::string_view text, std::string_view fileName)
{
    LoadResult result;
    try {
        // `parsed` is the text nlohmann/json actually read: the input, or the
        // input with overflowing numbers replaced (see replaceOverflowingNumbers).
        // Line numbers are counted in it; replacements never change lines.
        std::string replaced;
        std::string_view parsed = text;
        Json root;
        try {
            try {
                root = Json::parse(parsed.begin(), parsed.end());
            } catch (const Json::out_of_range&) {
                replaced = replaceOverflowingNumbers(text);
                parsed = replaced;
                root = Json::parse(parsed.begin(), parsed.end());
            }
        } catch (const Json::parse_error& e) {
            Reader reader(parsed, fileName);
            // e.byte is the 1-based index of the last character read.
            const std::size_t offset = e.byte > 0 ? e.byte - 1 : 0;
            reader.failAtLine(reader.lineAt(offset), "", e.what());
            result.error = reader.error();
            return result;
        }
        Reader reader(parsed, fileName);
        MarkerSet set;
        if (reader.readDocument(root, set)) {
            result.value = std::move(set);
        } else {
            result.error = reader.error();
        }
    } catch (const std::exception& e) {
        // Validation only reads values whose types were checked, so this should
        // not happen; still, never let an exception escape.
        result.value.reset();
        result.error =
            LoadError{std::string(fileName), 1, "", std::string("internal error: ") + e.what()};
    }
    return result;
}

LoadResult loadMarkerSetFile(const std::filesystem::path& path)
{
    const std::string fileName = path.string();
    const std::ifstream in(path, std::ios::binary);
    if (!in) {
        LoadResult result;
        result.error = LoadError{fileName, 1, "", "cannot open file"};
        return result;
    }
    std::ostringstream contents;
    contents << in.rdbuf();
    return loadMarkerSet(contents.str(), fileName);
}

std::string writeMarkerSet(const MarkerSet& set)
{
    std::string out = "{\n  \"format\": \"opas.markers\",\n  \"version\": 1,\n  \"markers\": ";
    if (set.markers.empty()) {
        out += "[]\n}\n";
        return out;
    }
    out += "[\n";
    for (std::size_t i = 0; i < set.markers.size(); ++i) {
        const InteractionMarker& m = set.markers[i];
        out += "    {\n";
        out += "      \"id\": " + jsonString(m.id) + ",\n";
        out += "      \"sourceBone\": " + jsonString(m.sourceBone) + ",\n";
        out += "      \"targetRegion\": " + jsonString(m.targetRegion) + ",\n";
        out += "      \"offsetM\": " + vec3(m.offsetM) + ",\n";
        out += "      \"contactDirection\": " + vec3(m.contactDirection) + ",\n";
        out += "      \"window\": {\"startS\": " + number(m.window.startS) +
               ", \"endS\": " + number(m.window.endS) + "},\n";
        out += "      \"priority\": " + std::to_string(m.priority) + ",\n";
        out += "      \"toleranceM\": " + number(m.toleranceM) + "\n";
        out += i + 1 < set.markers.size() ? "    },\n" : "    }\n";
    }
    out += "  ]\n}\n";
    return out;
}

} // namespace opas::markers
