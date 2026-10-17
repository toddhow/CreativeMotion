# One test runner for unit, property, replay and round-trip tests.
# Add new test files to the list below, grouped by folder.
add_executable(opas_tests
  unit/SmokeTest.cpp
  unit/core_math/QuatTest.cpp
  unit/core_math/TransformTest.cpp
  unit/core_math/Vec3Test.cpp
  unit/core_rig/SkeletonTest.cpp
  unit/kinematic_ik/TwoBoneIkTest.cpp
  unit/kinematic_markers/AlignmentScenesTest.cpp
  unit/kinematic_markers/AlignmentTest.cpp
  unit/kinematic_markers/BodyMapLoadTest.cpp
  unit/kinematic_markers/ContactReviewTest.cpp
  unit/kinematic_markers/ContactRulesTest.cpp
  unit/kinematic_markers/ContactScenariosTest.cpp
  unit/kinematic_markers/DetectionProfileTest.cpp
  unit/kinematic_markers/BodyMeasurementsTest.cpp
  unit/kinematic_markers/MarkerLoadTest.cpp
  unit/kinematic_markers/MarkerValidationTest.cpp
  unit/kinematic_markers/PoseClipLoadTest.cpp
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
  roundtrip/kinematic_markers/BodyMapHumanoidRoundTripTest.cpp
  roundtrip/kinematic_markers/BodyMapRoundTripTest.cpp
  roundtrip/kinematic_markers/MarkerRoundTripTest.cpp
  roundtrip/kinematic_markers/NegativeZeroRoundTripTest.cpp
  roundtrip/kinematic_markers/PoseClipRoundTripTest.cpp
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

# --- Command-line tool tests (A4b) -------------------------------------------
# opas_validate_markers is run by tools/RunTool.cmake, which checks the exit code
# (0 all pass, 1 some fail, 2 bad arguments or files) and the output lines.
# ctest's WILL_FAIL cannot tell 1 from 2, hence the script. ARGS are joined by "|";
# R1.. are regexes the output must match, N1.. regexes it must not match.
if(TARGET opas_validate_markers)
  set(V "${CMAKE_CURRENT_SOURCE_DIR}/data/validation")
  set(D "${CMAKE_CURRENT_SOURCE_DIR}/data")
  function(opas_add_tool_test name exitCode args)
    set(defs "")
    foreach(extra IN LISTS ARGN)
      list(APPEND defs "-D${extra}")
    endforeach()
    add_test(NAME ValidateMarkersTool.${name}
      COMMAND ${CMAKE_COMMAND}
        "-DTOOL=$<TARGET_FILE:opas_validate_markers>"
        "-DARGS=${args}"
        "-DEXPECT_EXIT=${exitCode}"
        ${defs}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/tools/RunTool.cmake")
  endfunction()

  set(PASS_ARGS "--markers|${V}/markers_pass.markers.json|--clip|${V}/clip.poseclip.json|--bodymap|${V}/post.bodymap.json")
  set(FAIL_ARGS "--markers|${V}/markers_fail.markers.json|--clip|${V}/clip.poseclip.json|--bodymap|${V}/post.bodymap.json")

  # Exit 0: every marker passes. One line per marker, then a summary line.
  opas_add_tool_test(AllPass 0 "${PASS_ARGS}"
    "R1=PASS handOnPost worst=0[.]000m" "N1=FAIL" "EXPECT_LINES=2")
  # Exit 1: one marker fails at the pairings with s0 = 1.25 and s0 = 0.8 (residual |s0 - 1|, worst 0.25).
  opas_add_tool_test(SomeFail 1 "${FAIL_ARGS}"
    "R1=PASS handOnPost worst=0[.]000m"
    "R2=FAIL tipOnPost worst=0[.]250m at 0[.]8/0[.]8"
    "R3=0[.]8/1[.]25"
    "R4=1[.]25/0[.]8"
    "R5=1[.]25/1[.]25"
    "N1=(at|,) 1(/|[.]0/)"
    "EXPECT_LINES=3")
  # --scales: a sweep of one scale has one pairing, 1/1, where everything is as recorded.
  opas_add_tool_test(ScalesOptionAllPass 0 "${FAIL_ARGS}|--scales|1"
    "R1=PASS handOnPost" "R2=PASS tipOnPost" "N1=FAIL")
  opas_add_tool_test(ScalesOptionListsOnlyThePairingsOfTheSweep 1 "${FAIL_ARGS}|--scales|0.8,1.25"
    "R1=FAIL tipOnPost worst=0[.]250m at 0[.]8/0[.]8, 0[.]8/1[.]25, 1[.]25/0[.]8, 1[.]25/1[.]25")
  # --bodymap1: actor 1 uses its own map; this one has no "Post" region, so the marker cannot run.
  opas_add_tool_test(Bodymap1IsUsedForActorOne 1 "${PASS_ARGS}|--bodymap1|${V}/noregion.bodymap.json"
    "R1=FAIL handOnPost")
  opas_add_tool_test(Bodymap1SameAsBodymapGivesTheSameResult 0 "${PASS_ARGS}|--bodymap1|${V}/post.bodymap.json"
    "R1=PASS handOnPost" "EXPECT_LINES=2")

  # Exit 2: bad arguments or files.
  opas_add_tool_test(MissingMarkersFile 2 "--markers|${V}/does_not_exist.markers.json|--clip|${V}/clip.poseclip.json|--bodymap|${V}/post.bodymap.json"
    "R1=does_not_exist[.]markers[.]json")
  opas_add_tool_test(MissingClipFile 2 "--markers|${V}/markers_pass.markers.json|--clip|${V}/does_not_exist.poseclip.json|--bodymap|${V}/post.bodymap.json"
    "R1=does_not_exist[.]poseclip[.]json")
  opas_add_tool_test(MissingBodymapFile 2 "--markers|${V}/markers_pass.markers.json|--clip|${V}/clip.poseclip.json|--bodymap|${V}/does_not_exist.bodymap.json"
    "R1=does_not_exist[.]bodymap[.]json")
  opas_add_tool_test(MissingBodymap1File 2 "${PASS_ARGS}|--bodymap1|${V}/does_not_exist.bodymap.json"
    "R1=does_not_exist[.]bodymap[.]json")
  opas_add_tool_test(NoArguments 2 "")
  opas_add_tool_test(MissingRequiredOption 2 "--markers|${V}/markers_pass.markers.json|--bodymap|${V}/post.bodymap.json")
  opas_add_tool_test(UnknownOption 2 "${PASS_ARGS}|--bogus")
  opas_add_tool_test(OptionWithoutValue 2 "--markers|${V}/markers_pass.markers.json|--clip|${V}/clip.poseclip.json|--bodymap")
  opas_add_tool_test(UnparsableScales 2 "${PASS_ARGS}|--scales|0.8,abc")
  opas_add_tool_test(NonPositiveScale 2 "${PASS_ARGS}|--scales|0")
  # Load errors print file:line: field: message (the file as given on the command line).
  opas_add_tool_test(MarkersLoadErrorIsLocated 2
    "--markers|${V}/bad_tolerance.markers.json|--clip|${V}/clip.poseclip.json|--bodymap|${V}/post.bodymap.json"
    "R1=bad_tolerance[.]markers[.]json:15: /markers/0/toleranceM: ")
  opas_add_tool_test(ClipLoadErrorIsLocated 2
    "--markers|${V}/markers_pass.markers.json|--clip|${D}/poseclip/bad_quaternion.poseclip.json|--bodymap|${V}/post.bodymap.json"
    "R1=bad_quaternion[.]poseclip[.]json:18: /actors/1/frames/0/0: ")
  opas_add_tool_test(BodymapLoadErrorIsLocated 2
    "--markers|${V}/markers_pass.markers.json|--clip|${V}/clip.poseclip.json|--bodymap|${D}/bodymap/bad_radius.bodymap.json"
    "R1=bad_radius[.]bodymap[.]json:21: /ikChains/0/limbRadiusM: ")
endif()
