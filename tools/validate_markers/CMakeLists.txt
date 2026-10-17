# opas_validate_markers: the A4b validation pass as a command-line tool
# (docs/tasks/0007-a4b-marker-validation.md, rule 5).
add_executable(opas_validate_markers main.cpp)
target_link_libraries(opas_validate_markers PRIVATE opas::markers)
opas_set_project_options(opas_validate_markers)
opas_copy_sanitizer_runtime(opas_validate_markers)
