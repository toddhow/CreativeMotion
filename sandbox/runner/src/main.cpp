// opas_run: plays a scenario headlessly and records it, or compares the run
// with a golden recording (docs/tasks/0015).
//
//   opas_run --scenario <file> --record <out>
//   opas_run --scenario <file> --compare <golden>
//
// Exit codes: 0 = recorded, or within tolerance; 1 = differs from the golden;
// 2 = bad arguments or files.

#include "opas/sandbox/Recording.hpp"
#include "opas/sandbox/Runner.hpp"

#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace {

constexpr int kOk = 0;
constexpr int kDiffers = 1;
constexpr int kBadInput = 2;

int usage(std::string_view problem)
{
    std::cerr << "error: " << problem << "\n"
              << "usage: opas_run --scenario <file> (--record <out> | --compare <golden>)\n";
    return kBadInput;
}

int runTool(int argc, const char* const* argv)
{
    std::string scenario;
    std::string record;
    std::string compare;
    for (int i = 1; i < argc; ++i) {
        const std::string_view option = argv[i];
        if (option != "--scenario" && option != "--record" && option != "--compare") {
            return usage("unknown option " + std::string(option));
        }
        if (i + 1 >= argc) {
            return usage(std::string(option) + " needs a value");
        }
        const std::string value = argv[++i];
        std::string& target = option == "--scenario" ? scenario
                              : option == "--record" ? record
                                                     : compare;
        if (!target.empty()) {
            return usage(std::string(option) + " given twice");
        }
        target = value;
    }
    if (scenario.empty()) {
        return usage("--scenario is required");
    }
    if (record.empty() == compare.empty()) {
        return usage("give exactly one of --record and --compare");
    }

    const opas::sandbox::RunResult result = opas::sandbox::runFile(scenario);
    if (!result.recording) {
        std::cerr << result.error.toString() << "\n";
        return kBadInput;
    }
    const opas::sandbox::Recording& recording = *result.recording;
    if (!record.empty()) {
        std::ofstream out(record, std::ios::binary);
        out << opas::sandbox::writeRecording(recording);
        out.close(); // flush now, so a failed write is seen below
        if (!out) {
            std::cerr << record << ": cannot write the recording\n";
            return kBadInput;
        }
        std::cout << "recorded " << recording.steps.size() << " states of "
                  << recording.bodies.size() << " bodies to " << record << "\n";
        return kOk;
    }
    const opas::sandbox::RecordingLoadResult golden = opas::sandbox::loadRecordingFile(compare);
    if (!golden.value) {
        std::cerr << golden.error.toString() << "\n";
        return kBadInput;
    }
    if (const auto difference = opas::sandbox::compareRecordings(recording, *golden.value)) {
        std::cout << "DIFFERS " << difference->toString() << "\n";
        return kDiffers;
    }
    std::cout << "MATCHES " << recording.steps.size() << " states of " << recording.bodies.size()
              << " bodies\n";
    return kOk;
}

} // namespace

int main(int argc, char** argv) // NOLINT(misc-const-correctness)
{
    try {
        return runTool(argc, argv);
    } catch (const std::exception& e) {
        // C stdio here: a stream that failed may throw again.
        std::fputs("error: ", stderr);
        std::fputs(e.what(), stderr);
        std::fputs("\n", stderr);
        return kBadInput;
    } catch (...) {
        return kBadInput;
    }
}
