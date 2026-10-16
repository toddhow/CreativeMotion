# Task: M0 Bootstrap
Design doc section: CLAUDE.md → "First milestones with acceptance criteria" → M0 Bootstrap
Goal: An empty-but-real repository that builds, tests and lints on Windows and Linux, with Jolt pinned and confined to `physics/jolt`.
Inputs / outputs: `CMakeLists.txt`, `CMakePresets.json`, `cmake/`, `physics/jolt/`, `tests/`, `.gitlab-ci.yml`, `tools/`, `.clang-format`, `.clang-tidy`, `docs/`.
Constraints: no system-wide dependency installs required for the build (FetchContent, pinned); warnings as errors on our targets; Jolt linked PRIVATE into `physics/jolt` only.
Acceptance criteria:
- [x] `debug`, `release` and `asan` presets configure, build and pass tests on Windows (MSVC 19.44) — checked locally 2026-10-16
- [ ] Same presets pass on Linux (GCC, Clang) — checked in CI
- [x] GoogleTest wired in with one passing test (`Smoke.TestRunnerWorks`)
- [x] Jolt fetched at a pinned commit with archive hash, linked only into `physics/jolt` (`JoltBackendInfo.*` tests)
- [ ] CI runs build, format, lint, tests and sanitizers on every merge request — `.gitlab-ci.yml` written, not yet run (no GitLab project yet)
- [x] Guide committed as `CLAUDE.md`
- [ ] Design docs exported into `docs/` — **blocked: the two design docs are not in the repository yet**
Out of scope: the physics interface (M1), the CI check for Jolt includes outside `physics/jolt` (M1), choosing a license or project name.

Notes:
- Tests-first and the fresh reviewer step are light for M0: the only behavior is "it builds and links".
- `opas` / `OPAS_` is a placeholder prefix until the project is named.
