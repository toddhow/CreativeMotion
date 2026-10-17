// opas_validate_markers: runs the A4b validation pass on files and reports
// which markers cannot be met across body-size pairings.
// Usage and output: docs/tasks/0007-a4b-marker-validation.md, rule 5.

#include "opas/markers/BodyMapFile.hpp"
#include "opas/markers/MarkerFile.hpp"
#include "opas/markers/PoseClip.hpp"
#include "opas/markers/Validation.hpp"

#include <charconv>
#include <cstdio>
#include <exception>
#include <format>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr int kAllPassed = 0;
constexpr int kSomeFailed = 1;
constexpr int kBadInput = 2;

struct Arguments {
    std::string markers;
    std::string clip;
    std::string bodyMap;
    std::string bodyMap1;
    std::optional<std::vector<double>> scales;
};

int usage(std::string_view problem)
{
    std::cerr << "error: " << problem << "\n"
              << "usage: opas_validate_markers --markers <file> --clip <file> --bodymap <file>\n"
              << "                             [--bodymap1 <file>] [--scales 0.8,1,1.25]\n";
    return kBadInput;
}

// "0.8,1,1.25" -> {0.8, 1, 1.25}; nullopt if any entry is not a number.
std::optional<std::vector<double>> parseScales(std::string_view text)
{
    std::vector<double> scales;
    while (!text.empty()) {
        const auto comma = text.find(',');
        const std::string_view item = text.substr(0, comma);
        double value = 0.0;
        const auto [ptr, ec] = std::from_chars(item.data(), item.data() + item.size(), value);
        if (ec != std::errc{} || ptr != item.data() + item.size()) {
            return std::nullopt;
        }
        scales.push_back(value);
        if (comma == std::string_view::npos) {
            break;
        }
        text.remove_prefix(comma + 1);
        if (text.empty()) {
            return std::nullopt; // a trailing comma, e.g. "0.8,1,"
        }
    }
    return scales;
}

std::optional<Arguments> parse(int argc, const char* const* argv, std::string& problem)
{
    Arguments args;
    std::set<std::string> seen; // each flag may appear once
    for (int i = 1; i < argc; ++i) {
        const std::string_view flag = argv[i];
        if (i + 1 >= argc) {
            problem = std::format("{} needs a value", flag);
            return std::nullopt;
        }
        const std::string_view value = argv[++i];
        if (value.starts_with("--")) {
            problem = std::format("{} needs a value, got the flag \"{}\"", flag, value);
            return std::nullopt;
        }
        if (!seen.insert(std::string(flag)).second) {
            problem = std::format("{} is given more than once", flag);
            return std::nullopt;
        }
        if (flag == "--markers") {
            args.markers = value;
        } else if (flag == "--clip") {
            args.clip = value;
        } else if (flag == "--bodymap") {
            args.bodyMap = value;
        } else if (flag == "--bodymap1") {
            args.bodyMap1 = value;
        } else if (flag == "--scales") {
            args.scales = parseScales(value);
            if (!args.scales) {
                problem =
                    std::format("--scales must be comma-separated numbers, got \"{}\"", value);
                return std::nullopt;
            }
        } else {
            problem = std::format("unknown argument \"{}\"", flag);
            return std::nullopt;
        }
    }
    if (args.markers.empty() || args.clip.empty() || args.bodyMap.empty()) {
        problem = "--markers, --clip and --bodymap are required";
        return std::nullopt;
    }
    return args;
}

std::string joinFailures(const opas::markers::MarkerValidation& m)
{
    std::string out;
    for (std::size_t i = 0; i < m.failures.size(); ++i) {
        out +=
            std::format("{}{}/{}", i > 0 ? ", " : "", m.failures[i].scale0, m.failures[i].scale1);
    }
    return out;
}

int run(int argc, const char* const* argv)
{
    std::string problem;
    const std::optional<Arguments> args = parse(argc, argv, problem);
    if (!args) {
        return usage(problem);
    }

    const auto markers = opas::markers::loadMarkerSetFile(args->markers);
    if (!markers.value) {
        std::cerr << markers.error.toString() << "\n";
        return kBadInput;
    }
    const auto clip = opas::markers::loadPoseClipFile(args->clip);
    if (!clip.value) {
        std::cerr << clip.error.toString() << "\n";
        return kBadInput;
    }
    const auto map0 = opas::markers::loadBodyMapFile(args->bodyMap);
    if (!map0.value) {
        std::cerr << map0.error.toString() << "\n";
        return kBadInput;
    }
    opas::markers::BodyMapping map1 = *map0.value;
    if (!args->bodyMap1.empty()) {
        const auto loaded = opas::markers::loadBodyMapFile(args->bodyMap1);
        if (!loaded.value) {
            std::cerr << loaded.error.toString() << "\n";
            return kBadInput;
        }
        map1 = *loaded.value;
    }

    opas::markers::ValidationSettings settings;
    if (args->scales) {
        settings.scales = *args->scales;
    }
    const opas::markers::ValidationReport report =
        opas::markers::validateMarkers(*markers.value, *clip.value, {*map0.value, map1}, settings);
    if (!report.valid) {
        std::cerr << "error: " << report.error << "\n";
        return kBadInput;
    }

    std::size_t passed = 0;
    for (const auto& m : report.markers) {
        if (m.passed()) {
            ++passed;
            std::cout << std::format("PASS {} worst={:.3f}m\n", m.id, m.worstResidualM);
        } else {
            std::cout << std::format("FAIL {} worst={:.3f}m at {}\n", m.id, m.worstResidualM,
                                     joinFailures(m));
        }
    }
    std::cout << std::format("summary: {} passed, {} failed\n", passed,
                             report.markers.size() - passed);
    return report.allPassed() ? kAllPassed : kSomeFailed;
}
} // namespace

// The standard fixes main's signature, so argv cannot be made const here.
int main(int argc, char** argv) // NOLINT(misc-const-correctness)
{
    // Report any exception (e.g. out of memory) as bad input rather than let
    // it escape and abort.
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fputs("error: ", stderr);
        std::fputs(e.what(), stderr);
        std::fputs("\n", stderr);
    } catch (...) {
        std::fputs("error: unknown failure\n", stderr);
    }
    return kBadInput;
}
