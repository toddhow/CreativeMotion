#pragma once

// Internal: a rule broken by a rig or profile, located by the JSON pointer of
// the field as it appears in the file. The validate* functions turn it into
// "<pointer>: <message>"; the loaders look the pointer up in the parsed
// document to report the line.

#include <optional>
#include <string>

namespace opas::rig {

struct Rig;
struct Profile;

namespace detail {

struct Problem {
    std::string pointer;
    std::string message;
};

std::optional<Problem> findRigProblem(const Rig& rig);

// Rules a profile follows on its own (ranges, duplicate overrides).
std::optional<Problem> findProfileProblem(const Profile& profile);

// Rules that need the rig: the rig name and every overridden bone.
std::optional<Problem> findProfileRigProblem(const Profile& profile, const Rig& rig);

inline std::string toString(const std::optional<Problem>& problem)
{
    return problem ? problem->pointer + ": " + problem->message : std::string();
}

} // namespace detail
} // namespace opas::rig
