# One test runner for unit, property, replay and round-trip tests.
# Add new test files to the list below, grouped by folder.
add_executable(opas_tests
  unit/SmokeTest.cpp
  unit/physics_jolt/BackendInfoTest.cpp
  unit/physics_jolt/BoxDropTest.cpp
  unit/physics_jolt/DeterminismTest.cpp
  unit/physics_jolt/DriveBodiesTest.cpp
  unit/physics_jolt/HandlesTest.cpp
  unit/physics_jolt/JointTest.cpp
  unit/physics_jolt/RaycastTest.cpp
  unit/physics_jolt/ReviewAmendmentsTest.cpp
  unit/physics_jolt/SeveralWorldsTest.cpp
  unit/physics_jolt/StepListenerTest.cpp
  unit/physics_jolt/ValidationTest.cpp
)

target_link_libraries(opas_tests PRIVATE
  opas::physics_jolt
  GTest::gtest_main
)
opas_set_project_options(opas_tests)

# Interface-only runner. It links nothing but the interface headers and
# GoogleTest, which proves physics/interface compiles without Jolt. Do not add
# opas::physics_jolt here.
add_executable(opas_interface_tests
  unit/physics_interface/HeadersCompileTest.cpp
  unit/physics_interface/ValueTypesTest.cpp
)

target_link_libraries(opas_interface_tests PRIVATE
  opas::physics_interface
  GTest::gtest_main
)
opas_set_project_options(opas_interface_tests)

include(GoogleTest)
# DISCOVERY_MODE PRE_TEST lists tests when ctest runs rather than at build time,
# which keeps sanitizer builds from failing during the build step.
gtest_discover_tests(opas_tests DISCOVERY_MODE PRE_TEST)
gtest_discover_tests(opas_interface_tests DISCOVERY_MODE PRE_TEST)
