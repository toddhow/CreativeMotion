#pragma once

// Reading and writing marker files (JSON). Format: docs/formats/markers.md.

#include "opas/markers/InteractionMarker.hpp"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace opas::markers {

// Where and why a file was rejected.
struct LoadError {
    std::string file;     // as passed by the caller
    std::size_t line = 0; // 1-based
    std::string field;    // JSON pointer, e.g. "/markers/2/toleranceM"; "" for the whole document
    std::string message;

    // "file:line: field: message"
    [[nodiscard]] std::string toString() const;
};

struct LoadResult {
    std::optional<MarkerSet> value; // set on success
    LoadError error;                // meaningful only when value is empty

    [[nodiscard]] bool ok() const { return value.has_value(); }
};

// Parses and validates marker-file text. `fileName` is only used in errors.
// Never throws.
LoadResult loadMarkerSet(std::string_view text, std::string_view fileName);

// Reads a file from disk, then as loadMarkerSet. Never throws.
LoadResult loadMarkerSetFile(const std::filesystem::path& path);

// Canonical text: fixed key order, 2-space indent, LF line endings, trailing
// newline, numbers in shortest round-trip form. Loading this text and writing
// it again gives the same bytes.
std::string writeMarkerSet(const MarkerSet& set);

} // namespace opas::markers
