# 0003 — Hosting on GitHub (replaces the GitLab choice)

Date: 2026-10-16 · Decided by: toddhow

## Decision
Both repositories, CreativeMotion (this one) and CreativeAlignment (the SKSE plugin), are hosted on GitHub, and CI runs on GitHub Actions (`.github/workflows/ci.yml`). This replaces the earlier same-day choice of GitLab.

## Why
- toddhow's preference. Keeping both repositories on one host keeps the plugin's pinned dependency on this repository (by tag) simple.

## Consequences
- `.gitlab-ci.yml` is removed. The same gates (Linux GCC and Clang, ASan+UBSan, Windows MSVC, clang-format, clang-tidy, the Jolt include check) run as GitHub Actions jobs. The Windows job reuses `tools/ci/windows-build.ps1`.
- The guide says "pull request" again.
- OpenMW itself stays on GitLab. M7's proposal to OpenMW and its merge request !3919 are unaffected.
