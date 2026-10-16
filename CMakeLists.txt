# Top-level build for the Open Procedural Animation System.
# "opas" / OPAS_ is a placeholder prefix until the project name is decided
# (see "Open questions" in CLAUDE.md).

# 3.25 is the first version where FetchContent_Declare accepts SYSTEM, which
# keeps third-party headers from tripping our warnings-as-errors.
cmake_minimum_required(VERSION 3.25)

project(opas VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
  message(FATAL_ERROR "In-source builds are not allowed. Use a preset, e.g. `cmake --preset debug`.")
endif()

option(OPAS_BUILD_TESTS "Build the test runner" ON)
option(OPAS_SANITIZE "Build everything with AddressSanitizer (and UBSan where the compiler supports it)" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(OpasWarnings)
include(OpasSanitizers)

# Sanitizer flags must be set before dependencies are added so that Jolt and
# GoogleTest are instrumented the same way as our code.
opas_enable_sanitizers_globally()

include(OpasDependencies)

add_subdirectory(physics)

if(OPAS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)

  # Architecture check: Jolt headers only under physics/jolt/.
  set(jolt_check "${CMAKE_CURRENT_SOURCE_DIR}/tools/CheckJoltIncludes.cmake")
  add_test(NAME JoltIncludeCheck.Repository
    COMMAND ${CMAKE_COMMAND} -DROOT=${CMAKE_CURRENT_SOURCE_DIR} -P ${jolt_check})

  # Self-tests: the check must pass when Jolt is included only in the backend,
  # and fail when anything else includes it.
  set(fixture "${CMAKE_CURRENT_BINARY_DIR}/jolt_include_fixture")
  file(WRITE "${fixture}/allowed/physics/jolt/Backend.cpp" "#include <Jolt/Jolt.h>\n")
  file(WRITE "${fixture}/allowed/core/Clean.cpp" "#include <vector>\n")
  file(WRITE "${fixture}/violation/physics/jolt/Backend.cpp" "#include <Jolt/Jolt.h>\n")
  file(WRITE "${fixture}/violation/core/Leak.hpp" "  #  include \"Jolt/Physics/Body/Body.h\"\n")
  add_test(NAME JoltIncludeCheck.AllowsBackend
    COMMAND ${CMAKE_COMMAND} -DROOT=${fixture}/allowed -P ${jolt_check})
  add_test(NAME JoltIncludeCheck.RejectsIncludeOutsideBackend
    COMMAND ${CMAKE_COMMAND} -DROOT=${fixture}/violation -P ${jolt_check})
  set_tests_properties(JoltIncludeCheck.RejectsIncludeOutsideBackend PROPERTIES WILL_FAIL TRUE)
endif()
