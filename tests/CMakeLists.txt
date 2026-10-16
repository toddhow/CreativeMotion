# One test runner for unit, property, replay and round-trip tests.
# Add new test files to the list below, grouped by folder.
add_executable(opas_tests
  unit/SmokeTest.cpp
  unit/core_math/QuatTest.cpp
  unit/core_math/TransformTest.cpp
  unit/core_math/Vec3Test.cpp
  unit/core_rig/SkeletonTest.cpp
  unit/kinematic_ik/TwoBoneIkTest.cpp
  unit/kinematic_markers/BodyMeasurementsTest.cpp
  unit/kinematic_markers/MarkerLoadTest.cpp
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
  property/core_math/QuatNormalizationPropertyTest.cpp
  property/kinematic_ik/TwoBoneIkPropertyTest.cpp
  roundtrip/kinematic_markers/MarkerRoundTripTest.cpp
)

target_link_libraries(opas_tests PRIVATE
  opas::physics_jolt
  opas::math
  opas::rig
  opas::ik
  opas::markers
  GTest::gtest_main
)
# Helpers shared by the kinematic tests (seeded random numbers, comparisons,
# synthetic marker texts) live in tests/common.
target_include_directories(opas_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common)
# Small synthetic marker files for the file-loading tests.
target_compile_definitions(opas_tests PRIVATE
  OPAS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
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
