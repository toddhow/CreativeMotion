# Engineering Guide for Claude Code — Open Procedural Animation System

Oct 16, 2026 · @toddhow

## Purpose and how to use this guide

This guide tells Claude Code, and any human contributor, how to build the system: what to build first, where code lives, the rules every change follows, and how each change is verified. What to build and why lives in Open Procedural Animation System — Living Design Doc and OpenMW Animation System — Implementation Doc; this guide covers how.

- **Placing it:** export this doc as Markdown and save it at the repository root as `CLAUDE.md`, so Claude Code reads it at the start of every session. Export the two design docs into `docs/` as local copies: `.gitignore` keeps them (and any root export of this guide) out of both repositories, so they are never committed.
- **Authority:** the design docs decide what the system does; this guide decides how work is done. When they disagree, stop and ask the project owner (toddhow) rather than choosing.
- **Owner:** toddhow is the architect and final reviewer. He is learning C++ and engine development through this project, so explain non-obvious changes in plain language in each pull request.
- **Updating:** when a decision changes, update the relevant design doc and this guide in the same change, and add a changelog row.

## Scope of the first build

The first build is the physical layer, standalone: the behavior core, a Jolt backend and a sandbox, with no game engine involved. It later moves into OpenMW as a self-contained component, so it must not depend on anything outside this repository except its listed libraries.

**In scope now**

- Physics interface and its Jolt implementation
- Rig and profile file formats with loaders
- Motor solver that makes a ragdoll track an animated target pose
- Sensors, balance and hit reaction behaviors
- Headless sandbox runner with record and replay, then a debug viewer

**Not yet**

- OpenMW integration (waits for Jolt to land in OpenMW, merge request !3919)
- SKSE or Havok code in this repository (Skyrim-specific code lives in the plugin repository; see Alignment plugin track)
- Animation runtime, style layer, face layer, importers (IK and interaction markers start in the alignment track)
- Learned (machine learning) controllers

## Tech stack and toolchain

Defaults below are chosen to match where the code is headed (OpenMW) and to keep the build simple; items marked *confirm* are open questions for toddhow.

| Area | Choice | Notes |
| --- | --- | --- |
| Language | C++20 | Confirm against OpenMW's current required standard before integration |
| Build | CMake | Out-of-source builds; presets for Debug, Release, ASan |
| Dependencies | CMake FetchContent, pinned versions | No system-wide installs required |
| Physics | [Jolt Physics](https://github.com/jrouwe/JoltPhysics) | MIT license; only the backend module includes Jolt headers |
| Tests | GoogleTest | Unit, property and replay tests in one runner |
| Data files | JSON for authored files and recordings | nlohmann/json for parsing; our own canonical writer (`docs/decisions/0002`) |
| Debug viewer | raylib + Dear ImGui | Confirm; the headless runner comes first |
| Platforms | Windows (MSVC), Linux (GCC and Clang) | Both built in CI |
| CI | GitHub Actions (`.github/workflows/ci.yml`) | Every pull request runs all gates |
| Formatting | clang-format, clang-tidy | Checked in CI |

Units and conventions: meters, kilograms, seconds, radians; right-handed coordinates with Z up (matching OpenMW and Bethesda content); fixed physics step of 1/60 s by default, configurable.

## Repository layout

Each top-level folder is one module with its own short `CLAUDE.md` describing its rules and pitfalls; create that file when the module is created.

```text
CLAUDE.md                 this guide (exported)
docs/                     design doc exports, decision records (docs/decisions/NNNN-title.md)
core/
  math/                   vectors, quaternions, transforms, springs (no external math library)
  rig/                    rig and profile data model, loaders, validation
  sensors/                center of mass, support area, capture point, contacts
  behaviors/              balance, hit reaction, stumble, ... (one file per behavior)
  arbiter/                state machine and per-body-part blending
  motor/                  joint targets, stiffness, damping
  reconcile/              capsule and event outputs
kinematic/
  ik/                     two-bone and chain IK with joint limits
  markers/                interaction markers, contact detection, alignment solver
physics/
  interface/              the physics interface (headers only)
  jolt/                   Jolt backend (the only place Jolt headers appear)
sandbox/
  runner/                 headless deterministic runner, record and replay
  viewer/                 debug viewer and overlays (later)
  scenarios/              scenario files: floor, stairs, slopes, pushes
tests/
  unit/  property/  replay/  roundtrip/
data/
  rigs/  profiles/        sample humanoid rig and profiles
  recordings/             golden replay recordings (synthetic only)
.github/workflows/        CI (GitHub Actions)
tools/                    helper scripts for CI and developers
```

## Architecture rules and module boundaries

Dependencies point one way, and the physics interface is the boundary that keeps the core portable to OpenMW or any other host.

&#91;embedded content: module dependencies · arrows point to what a module depends on\]

- **One direction:** `core` depends only on `physics/interface` and the standard library; `physics/jolt` depends on `physics/interface` and Jolt; `sandbox` and `tests` may depend on everything. Nothing depends on `sandbox`.
- **No engine types in the core:** `core` never includes Jolt, OpenMW or rendering headers and uses its own math types.
- **Interface stays thin:** `physics/interface` is headers only, with no third-party types in its signatures.
- **Data, not code:** characters, rigs and tuning come from rig and profile files; no hard-coded character data.
- **Fixed step:** the core runs inside the backend's step callbacks at the physics rate and is deterministic for identical inputs.
- **Core never renders:** debug overlays are produced as data (points, lines, labels, colors) and drawn by the viewer.
- **Same pattern later:** the kinematic layers, face layer and importers each become modules with the same boundaries; OpenMW becomes one more host.

The `kinematic` modules follow the same rules: they depend only on `core/math` and `core/rig`, never on a physics backend, a game or a framework.

## Coding standards

Write code a learning C++ developer can follow: clear ownership, small functions, and comments that explain why, not what.

- **Ownership:** RAII everywhere; no raw `new` or `delete`; `std::unique_ptr` by default, `std::shared_ptr` only with a comment saying why. Every class states who owns it.
- **Host and physics objects:** store handles or IDs, never raw pointers, to anything a host or backend can destroy; look them up when used.
- **Hot paths:** no heap allocation inside the per-step update; preallocate and reuse buffers; use the backend's temporary allocator where it provides one.
- **Determinism:** no wall-clock time, no unseeded randomness, no iteration over unordered containers where order affects results.
- **Frame-rate independence:** smoothing and decay use dt with exponentials, never fixed per-step fractions.
- **Quaternions:** normalize after composition; pick the shorter path (handle q and -q) before any blend or error calculation.
- **Errors:** loaders return clear, located errors (file, line, field); no exceptions cross the future C API boundary.
- **Style:** clang-format and clang-tidy clean; warnings as errors; `const` by default; names say units where ambiguous (`stiffnessNmPerRad`).
- **Tuning values:** no magic numbers in code; defaults live in profile files or named constants with a comment on their source.

## Agent workflow and task spec template

Every change follows the same loop, with tests written independently of the implementation and a fresh reviewer before toddhow.

1. **Spec:** write or confirm a task spec in `docs/tasks/` using the template below; it links the design doc section it implements.
2. **Tests first:** a test-writing subagent writes tests from the spec alone, without seeing any implementation.
3. **Implement:** the implementing agent makes the tests pass. It may not edit or delete tests without toddhow's approval; if a test looks wrong, it stops and says why.
4. **Review:** a fresh reviewing subagent that did not write the code checks the diff against the spec, with extra attention to lifetimes, threading, determinism and allocations in hot paths.
5. **Verify:** all quality gates pass in CI; behavior changes include a sandbox replay and, once the viewer exists, a short recording for toddhow to judge the look.
6. **Explain and merge:** the pull request explains the change in plain language for toddhow, who approves the merge.

**Lighter loop for A4 to A6 (toddhow, 2026-10-17, to save tokens):** steps 2 and 4 are paused. No separate test-writing subagent: the implementing agent writes focused tests itself where they are cheap (core math, formats), and a fresh reviewing subagent runs once per milestone, not per change. toddhow still approves each spec before implementation and judges in-game behaviour. The full loop returns when toddhow says so.

Rules for every session:

- One task per branch; small pull requests.
- Ask when the spec is ambiguous instead of guessing.
- Never invent an API, file format detail or engine fact; cite a source or ask.
- Keep each module's `CLAUDE.md` current with rules and pitfalls learned.

```markdown
# Task: <short name>
Design doc section: <link>
Goal: <one sentence>
Inputs / outputs: <types, files, interfaces touched>
Constraints: <determinism, allocation, threading, performance budget>
Acceptance criteria:
- [ ] <testable statement>
- [ ] <testable statement>
Out of scope: <what not to change>
```

## Testing and quality gates

A pull request merges only when every gate below passes; gates are added as the code they check comes to exist.

| Gate | What it checks | Runs |
| --- | --- | --- |
| Build | Compiles on Windows (MSVC) and Linux (GCC, Clang) with warnings as errors | Every pull request |
| Format and lint | clang-format and clang-tidy clean | Every pull request |
| Unit tests | Each module's functions against hand-computed cases | Every pull request |
| Property tests | Quaternions stay normalized, IK reaches reachable targets, results independent of step rate, capture point matches the formula | Every pull request |
| Sanitizers | AddressSanitizer and UndefinedBehaviorSanitizer test run on Linux | Every pull request |
| Round-trip | Rig, profile and recording files read and written back byte-for-byte identical | Every pull request touching formats |
| Replay | Recorded sandbox scenarios rerun and compared to golden recordings within tolerance | Every pull request touching core or physics |
| Determinism | The same scenario run twice gives identical results | Every pull request |
| Performance budget | Cost per active ragdoll per step stays under the budget in `docs/budgets.md` | Nightly, and on request |

Golden recordings change only in a pull request that says why, reviewed by toddhow.

## First milestones with acceptance criteria

Seven milestones take the project from an empty repository to a ragdoll that balances, reacts to pushes and can be watched in a viewer. Work them in order; numeric tolerances and push sizes are set in each task spec, not guessed.

### M0 Bootstrap

- [ ] CMake project with Debug, Release and ASan presets builds on Windows and Linux
- [x] GoogleTest wired in with one passing test
- [x] Jolt fetched at a pinned version and linked only into `physics/jolt`
- [ ] CI runs build, format, lint, tests and sanitizers on every pull request
- [ ] This guide committed as `CLAUDE.md`; design docs exported into `docs/` locally (git-ignored, never committed)

### M1 Physics interface and Jolt backend

- [x] Interface covers build, drive, read, query, step and ownership, as in the design doc (drafted from this guide in `docs/tasks/0001`, approved by toddhow; reconcile once the design doc is exported)
- [x] A box dropped on a floor comes to rest at the expected height within tolerance
- [x] Two identical runs produce identical results
- [x] CI check fails if a Jolt header is included outside `physics/jolt` (job written; first CI run pending)

### M2 Rig and profile formats

- [ ] Schema documented in `docs/formats/`
- [ ] Sample humanoid rig with bodies, masses and joint limits
- [ ] Loader rejects invalid files with file, line and field
- [ ] Round-trip test passes

### M3 Ragdoll and motor solver

- [ ] The sample rig builds a ragdoll that falls and settles on the floor
- [ ] The motor solver tracks a procedural target pose (such as an arm wave) within the spec's tolerance
- [ ] Tracking stays stable at 30, 60 and 120 Hz physics steps
- [ ] No heap allocation in the per-step update, checked by an allocation counter in tests

### M4 Headless runner, record and replay

- [ ] Scenario files for floor, stairs, slope and push
- [ ] JSON recording of per-step body states
- [ ] Replay gate in CI against golden recordings

### M5 Sensors, balance and hit reaction

- [ ] Center of mass, support area and capture point computed and unit-tested
- [ ] A standing ragdoll recovers from small pushes by leaning and from larger ones by stepping
- [ ] Hit reaction lowers stiffness at the hit part, spreads a smaller drop to neighbors and ramps back

### M6 Debug viewer

- [ ] Draws bodies, center of mass, support area, capture point and active state
- [ ] Plays recordings step by step

M6 can move ahead of M5 if seeing progress early matters more.

## Alignment plugin track

After M1, a side track builds actor alignment for paired animations in Skyrim: the engine-independent parts in this repository, the Skyrim parts in a separate plugin repository. M2 resumes after A4, or earlier if toddhow decides.

| Repository | Contains | Builds and tests |
| --- | --- | --- |
| Main (this one) | `kinematic/ik` and `kinematic/markers`: IK, interaction markers, contact detection, alignment solver | Windows and Linux CI; sandbox and synthetic tests |
| Plugin (separate) | SKSE host module, SexLab and OStim adapters, in-game recording tool, packaging, Nexus releases | Windows/MSVC CI; CommonLibSSE-NG; main repository pinned to a tag |

- **Generic solver:** it serves all paired animations (killmoves, hugs, carrying, scenes); framework adapters exist only in the plugin repository.
- **Change order:** a solver change lands here first and is tagged; the plugin then bumps its pinned version.
- **Game-derived data:** bone transforms recorded in Skyrim stay out of public repositories; CI uses synthetic poses.
- **Licenses:** check CommonLibSSE-NG's license and each framework's terms before depending on them.

### A1 Kinematic foundations (main repository)

- [x] Two-bone IK with joint limits; property tests show reachable targets are reached and unreachable ones clamp cleanly
- [x] Interaction marker data model and file format (source bone, target body region, offset, contact direction, time window, priority, tolerance) with round-trip tests
- [x] Body measurement model: limb lengths and simple surface capsules from a skeleton

### A2 Alignment solver (main repository)

- [x] Root alignment for two actors from body measurements, with the correction split between both
- [x] Limb IK satisfies markers within joint limits, with penetration avoidance against capsules
- [x] Synthetic test scenes pair very different body sizes; markers are met within the spec's tolerance

### A3 First plugin release (plugin repository)

- [ ] SKSE plugin built on CommonLibSSE-NG that samples the scene actors' posed skeletons once per scene node (the per-frame pose hook moves to A4)
- [ ] One framework adapter: OStim NG (decided 2026-10-16); target Skyrim SE 1.6.1170
- [ ] Root alignment only, behind an in-game toggle, plus a recording command for local testing
- [ ] toddhow judges the result in game across several body-size pairings

### A4 Contacts and detection

- [ ] Automatic marker detection from paired clips (distance, duration, relative motion) with confidence tiers
- [ ] Hand and foot contact IK in game through a post-graph pose hook (hook points cited); the second framework adapter (SexLab P+) is deferred (toddhow, 2026-10-17)
- [ ] A validation pass flags stages whose markers cannot be met

### A5 Body surfaces and finger contact (both repositories; spec `docs/tasks/0009`)

- [ ] Capsule radii fitted per actor from points sampled on the skinned body mesh
- [ ] Fingers curl to touch the surface a hand marker meets

### A6 Marker editor and community presets (both repositories; spec `docs/tasks/0010`)

- [ ] In-game marker editor: capture, adjust, validate and save markers while a scene plays
- [ ] Community presets: a manifest format with licenses and priorities, installable as separate mods

### A7 Automatic marker placement (both repositories; spec `docs/tasks/0011`)

- [ ] Every frame, hands and feet near the partner keep their animated motion and only their depth is corrected against the partner's real surface (A7 v2, toddhow 2026-10-18; pinned automatic markers are off)
- [ ] Authored (user or preset) markers adapt to the touched actor's real surface before IK uses it

## Path to OpenMW

After M6, work moves into OpenMW in milestones that mirror the phases in OpenMW Animation System — Implementation Doc. None of them are active yet: each gets a full task spec with acceptance criteria before work starts.

**Gate:** work starts only when Jolt (merge request !3919) is merged into OpenMW and the OpenMW maintainers have answered the component proposal, deciding upstream or the modding-focused fork.

### M7 Propose and prepare

- [ ] Design proposal posted to OpenMW (GitLab or Discord): the component, its hook points, opt-in settings
- [ ] Upstream-or-fork decision recorded in `docs/decisions/`
- [ ] OpenMW builds locally and in CI on the Jolt branch

### M8 Move the core into OpenMW

- [ ] `core` and `physics/interface` added as a self-contained OpenMW component, mostly as new files
- [ ] The Jolt backend uses OpenMW's Jolt world instead of creating a second one
- [ ] Existing tests and replays run inside OpenMW's test setup

### M9 IK for Morrowind, no new physics

- [ ] Hook points located in OpenMW's source and cited by file and function
- [ ] Foot planting, look-at and smoother turns for Morrowind actors, each behind a setting that is off by default
- [ ] With the settings off, Morrowind movement is unchanged, verified by comparison

### M10 Dynamics and ragdolls

- [ ] Optional ragdolls for Morrowind actors on Jolt
- [ ] Actor capsules follow the ragdoll pelvis while the body is disturbed
- [ ] Per-character work runs on the engine's job system once it exists

### M11 Behavior core in game

- [ ] Hit reactions, stumbles, catch fall and get-ups from M5 work in game
- [ ] Stagger and knockdown events reach OpenMW's mechanics and AI
- [ ] Lua API to trigger behaviors, set styles, read state and subscribe to events

### M12 and beyond

- Skyrim content: importers, conversion cache, presets, interaction markers, face morphs
- Full system: character-to-character physics, style profiles, motion matching

OpenMW internals named in any of these tasks must come from OpenMW's source at the time; this guide deliberately names no OpenMW functions.

## Definition of done

A task is done only when all of these hold:

- Every acceptance criterion in its spec is met and checked by a test
- All quality gates pass in CI
- A fresh reviewing subagent found no unresolved issues
- The module's `CLAUDE.md` and any affected doc are updated
- The pull request explains what changed and why in plain language
- toddhow approved the merge

## Guardrails

These rules protect the project legally and keep it honest; breaking one blocks a merge.

- **No proprietary code or SDKs.** Never add the Havok SDK, decompiled game code, or code copied from mods that are not open-source licensed (Community Behaviors is currently All Rights Reserved; Universal Kinematics publishes no source).
- **No game assets.** No Bethesda animations, meshes, audio or other files in the repository; tests use generated data or permissively licensed samples with the license recorded.
- **Licenses checked before adding a dependency.** Each dependency's license is recorded in `docs/dependencies.md`; it must be compatible with the project license, GPL-3.0-or-later (OpenMW is also GPLv3).
- **Cite engine facts.** Any statement about OpenMW, Skyrim or Jolt internals cites a file, commit or documentation page; invented engine facts are treated as bugs.
- **Datasets.** Motion capture data is used only after its license is checked; several common datasets carry non-commercial terms.

## Open questions

Claude Code should raise these with toddhow when a task depends on one, rather than deciding alone.

- [ ] Project name (code uses the placeholder prefix `opas` until decided)
- [x] License: GPL-3.0-or-later, decided 2026-10-16 (`docs/decisions/0001-license.md`)
- [ ] C++ standard: confirm C++20 matches OpenMW's current requirement
- [x] Authored data format: JSON, decided 2026-10-16 (`docs/decisions/0002-json-for-authored-data.md`)
- [ ] Debug viewer library: raylib with Dear ImGui, or an alternative
- [ ] Where test animations come from: procedural poses first, then which permissively licensed clips
- [ ] Performance budget per active ragdoll for `docs/budgets.md`
- [x] Hosting: GitHub, with GitHub Actions; decided 2026-10-16, replacing GitLab (`docs/decisions/0003-github-hosting.md`)

## References

- [Jolt Physics](https://github.com/jrouwe/JoltPhysics)
- [OpenMW merge request !3919: Jolt physics implementation](https://gitlab.com/OpenMW/openmw/-/merge_requests/3919)
- [OpenMW physics settings: async num threads](https://openmw.readthedocs.io/en/openmw-0.47.0_a/reference/modding/settings/physics.html)
- [PLANCK, physics-driven animation in Skyrim](https://www.nexusmods.com/skyrimspecialedition/mods/66025)

## Changelog

| Date | Change |
| --- | --- |
| 2026-10-18 | A7 v2: per-frame contact correction replaces pinned automatic markers; root alignment only from user or preset files (toddhow) |
| 2026-10-18 | A7 automatic marker placement added (toddhow approved the spec); lighter workflow continues |
| 2026-10-17 | A5 (body surfaces, fingers) and A6 (marker editor, presets) scheduled after A4 (toddhow); specs drafted |
| 2026-10-17 | Workflow: test-writer subagent paused and reviews once per milestone for A4 to A6, to save tokens (toddhow) |
| 2026-10-17 | Design docs stay local: git-ignored in both repositories (none was ever committed) |
| 2026-10-17 | A4: second framework adapter (SexLab P+) deferred; A4 completes without it |
| 2026-10-16 | A3 design approved: root alignment through OStim's alignment API, poses sampled per node; the per-frame pose hook moves to A4; body maps become a JSON format (A3a) |
| 2026-10-16 | A3: first framework is OStim NG; target runtime Skyrim SE 1.6.1170 |
| 2026-10-16 | Hosting moved to GitHub (both repositories): GitHub Actions replaces GitLab CI; "merge request" is now "pull request" again |
| 2026-10-16 | Authored data format decided: JSON |
| 2026-10-16 | Added Alignment plugin track (A1 to A4), kinematic modules in the layout and rules, two-repository split |
| 2026-10-16 | Hosting moved to GitLab: GitLab CI replaces GitHub Actions; "pull request" is now "merge request" |
| 2026-10-16 | License decided: GPL-3.0-or-later; project name stays open |
| 2026-10-16 | Added Path to OpenMW: start gate and milestones M7 to M12 |
| 2026-10-16 | Guide created: scope, stack, layout, standards, agent workflow, gates, milestones M0 to M6, guardrails |
