# One test runner for unit, property, replay and round-trip tests.
# Add new test files to the list below, grouped by folder.
add_executable(opas_tests
  unit/SmokeTest.cpp
  unit/physics_jolt/BackendInfoTest.cpp
)

target_link_libraries(opas_tests PRIVATE
  opas::physics_jolt
  GTest::gtest_main
)
opas_set_project_options(opas_tests)

include(GoogleTest)
# DISCOVERY_MODE PRE_TEST lists tests when ctest runs rather than at build time,
# which keeps sanitizer builds from failing during the build step.
gtest_discover_tests(opas_tests DISCOVERY_MODE PRE_TEST)
