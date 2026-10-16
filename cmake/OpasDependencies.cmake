# Third-party dependencies, fetched at pinned versions.
#
# Each dependency is pinned to an exact commit with a SHA-256 of the source
# archive, so a build can never silently pick up different code. Licenses are
# recorded in docs/dependencies.md; update it whenever this file changes.

include(FetchContent)

# --- Jolt Physics -----------------------------------------------------------
# v5.6.0, commit e77f175595e64cb44218cc9d9d56fc365ad0e36a (2026-07-11). MIT.
# Only physics/jolt may link this target (see physics/jolt/CLAUDE.md).
#
# Options below are defined in Jolt's Build/CMakeLists.txt at that commit.
# They must be set before FetchContent_MakeAvailable so Jolt sees them.

# Same simulation results on Windows and Linux, so golden replay recordings
# can be shared between CI platforms. Costs some speed; revisit with the
# performance budget.
set(CROSS_PLATFORM_DETERMINISTIC ON CACHE BOOL "" FORCE)
# Match GoogleTest and our code, which use the DLL MSVC runtime (/MD).
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
# Jolt's own comment warns LTO must then be enabled for the whole project too.
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
# We do not own Jolt's warnings; newer compilers must not break our build.
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(ENABLE_OBJECT_STREAM OFF CACHE BOOL "" FORCE)
# GPU / compute backends are not needed for rigid-body ragdolls.
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)
set(JPH_USE_VK OFF CACHE BOOL "" FORCE)
set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)
set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  JoltPhysics
  URL https://github.com/jrouwe/JoltPhysics/archive/e77f175595e64cb44218cc9d9d56fc365ad0e36a.tar.gz
  URL_HASH SHA256=1f32328fb763135de10a244568d6ccb2ed9b1e6593fafe6dc6db5b2719d330bd
  SOURCE_SUBDIR Build
  SYSTEM
)

# --- nlohmann/json ----------------------------------------------------------
# v3.12.0, commit 55f93686c01528224f448c19128836e7df245f72 (2025-04-11). MIT.
# Used privately by kinematic/markers for JSON files. Diagnostic positions
# (an option in its CMakeLists.txt at that commit) record each value's byte
# offset, so loaders can report line numbers. The define is an INTERFACE
# definition of the target, so every user sees the same layout.
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
set(JSON_Diagnostic_Positions ON CACHE BOOL "" FORCE)

FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/archive/55f93686c01528224f448c19128836e7df245f72.tar.gz
  URL_HASH SHA256=67f4cdd9ca930c9c1e130af4a437c7fc98fab77a2846fc2d2a14b4943831f8ef
  SYSTEM
)
FetchContent_MakeAvailable(nlohmann_json)

# --- GoogleTest -------------------------------------------------------------
# v1.17.0, commit 52eb8108c5bdec04579160ae17225d66034bd723 (2025-04-30). BSD-3-Clause.
if(OPAS_BUILD_TESTS)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)

  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/52eb8108c5bdec04579160ae17225d66034bd723.tar.gz
    URL_HASH SHA256=745c55415660044610f7fcd3af7a6420d5de16a7dbb9ebfe2e131275676232be
    SYSTEM
  )
  FetchContent_MakeAvailable(JoltPhysics googletest)
else()
  FetchContent_MakeAvailable(JoltPhysics)
endif()
