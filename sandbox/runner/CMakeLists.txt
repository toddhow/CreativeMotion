# The headless sandbox runner (M4, docs/tasks/0015): scenario and recording
# formats, the runner library, and the opas_run tool. Sandbox may depend on
# everything; nothing depends on sandbox (CLAUDE.md).
add_library(opas_sandbox
  src/ScenarioFile.cpp
  src/RecordingFile.cpp
  src/Runner.cpp
)
add_library(opas::sandbox ALIAS opas_sandbox)
target_include_directories(opas_sandbox PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(opas_sandbox
  PUBLIC opas::math opas::rig opas::motor opas::format
  PRIVATE opas::physics_jolt opas::format_json
)
opas_set_project_options(opas_sandbox)

add_executable(opas_run src/main.cpp)
target_link_libraries(opas_run PRIVATE opas::sandbox)
opas_set_project_options(opas_run)
opas_copy_sanitizer_runtime(opas_run)
