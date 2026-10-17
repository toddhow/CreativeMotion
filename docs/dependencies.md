# Dependencies

Every third-party dependency, its pinned version and its license. A dependency
is added only after its license is checked for compatibility with the project
license, GPL-3.0-or-later. Permissive licenses (MIT, BSD, zlib, Apache-2.0) are
fine; GPLv2-only, non-commercial and proprietary licenses are not.

Pins live in `cmake/OpasDependencies.cmake` (commit + SHA-256 of the source archive).

| Dependency | Version | Commit | License | GPLv3-compatible | Used by | Source |
| --- | --- | --- | --- | --- | --- | --- |
| Jolt Physics | v5.6.0 | `e77f175595e64cb44218cc9d9d56fc365ad0e36a` | MIT | Yes | `physics/jolt` only | https://github.com/jrouwe/JoltPhysics |
| nlohmann/json | v3.12.0 | `55f93686c01528224f448c19128836e7df245f72` | MIT | Yes | `kinematic/markers` only (private) | https://github.com/nlohmann/json |
| GoogleTest | v1.17.0 | `52eb8108c5bdec04579160ae17225d66034bd723` | BSD-3-Clause | Yes | `tests` only | https://github.com/google/googletest |

## Build options we set on Jolt

Defined in `Build/CMakeLists.txt` of the pinned Jolt commit.

| Option | Value | Why |
| --- | --- | --- |
| `CROSS_PLATFORM_DETERMINISTIC` | ON | Same results on Windows and Linux so golden replays can be shared. Costs some speed. |
| `USE_STATIC_MSVC_RUNTIME_LIBRARY` | OFF | Match GoogleTest and our code (`/MD`). |
| `INTERPROCEDURAL_OPTIMIZATION` | OFF | Jolt documents link errors unless the whole project also uses LTO. |
| `ENABLE_ALL_WARNINGS` | OFF | Jolt's own warnings-as-errors must not break our build on newer compilers. |
| `JPH_USE_DX12`, `JPH_USE_VK`, `JPH_USE_MTL`, `JPH_USE_CPU_COMPUTE` | OFF | Compute backends are not needed. |
| `ENABLE_INSTALL`, `ENABLE_OBJECT_STREAM` | OFF | Not needed. |

## Tools (not linked into the build)

| Tool | Version | Where pinned |
| --- | --- | --- |
| clang-format | 23.1.3 | `.clang-format`, `.github/workflows/ci.yml` `format` job |
| clang-tidy | 22.1.8 in CI | `.github/workflows/ci.yml` `lint` job |
| CMake | ≥ 3.25 | `CMakeLists.txt` |
| Ninja (Windows CI) | 1.13.2 | `tools/ci/windows-build.ps1` (SHA-256 checked) |
