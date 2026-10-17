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

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(FATAL_ERROR "In-source builds are not allowed. Use a preset, e.g. `cmake --preset debug`.")
endif()

# When CreativeMotion is pulled into another project (the CreativeAlignment
# plugin uses it as a submodule for the kinematic libraries), tests and the
# Jolt physics layer default off so the host does not build them.
option(OPAS_BUILD_TESTS "Build the test runner" ${PROJECT_IS_TOP_LEVEL})
option(OPAS_BUILD_PHYSICS "Build physics/ (fetches Jolt)" ${PROJECT_IS_TOP_LEVEL})
if(OPAS_BUILD_TESTS AND NOT OPAS_BUILD_PHYSICS)
  message(FATAL_ERROR "OPAS_BUILD_TESTS needs OPAS_BUILD_PHYSICS (the test runner covers physics too).")
endif()
option(OPAS_SANITIZE "Build everything with AddressSanitizer (and UBSan where the compiler supports it)" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(OpasWarnings)
include(OpasSanitizers)

# Sanitizer flags must be set before dependencies are added so that Jolt and
# GoogleTest are instrumented the same way as our code.
opas_enable_sanitizers_globally()

include(OpasDependencies)

add_subdirectory(core)
if(OPAS_BUILD_PHYSICS)
  add_subdirectory(physics)
endif()
add_subdirectory(kinematic)
if(PROJECT_IS_TOP_LEVEL)
  add_subdirectory(tools/validate_markers)
endif()

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
  file(WRITE "${fixture}/allowed/build/Generated.cpp" "#include <Jolt/Jolt.h>\n")
  add_test(NAME JoltIncludeCheck.AllowsBackend
    COMMAND ${CMAKE_COMMAND} -DROOT=${fixture}/allowed -P ${jolt_check})
  # One fixture per way a leak could be written, each must be caught on its own.
  set(leaks
    "Spaced|core/Leak.hpp|  #  include \"Jolt/Physics/Body/Body.h\""
    "NoSpace|core/Leak.cpp|#include<Jolt/Jolt.h>"
    "Cxx|sandbox/Leak.cxx|#include <Jolt/Jolt.h>"
    "UpperCaseExtension|core/Leak.HPP|#include <Jolt/Jolt.h>"
    "ModuleInterface|core/Leak.ixx|#include <Jolt/Jolt.h>")
  foreach(leak IN LISTS leaks)
    string(REPLACE "|" ";" parts "${leak}")
    list(GET parts 0 name)
    list(GET parts 1 file)
    list(GET parts 2 line)
    file(WRITE "${fixture}/leak${name}/physics/jolt/Backend.cpp" "#include <Jolt/Jolt.h>\n")
    file(WRITE "${fixture}/leak${name}/${file}" "${line}\n")
    add_test(NAME JoltIncludeCheck.Rejects${name}
      COMMAND ${CMAKE_COMMAND} -DROOT=${fixture}/leak${name} -P ${jolt_check})
    set_tests_properties(JoltIncludeCheck.Rejects${name} PROPERTIES WILL_FAIL TRUE)
  endforeach()
endif()
