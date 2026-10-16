# Minimal skeleton model (M2 extends it).
add_library(opas_rig
  src/Skeleton.cpp
)
add_library(opas::rig ALIAS opas_rig)
target_include_directories(opas_rig PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(opas_rig PUBLIC opas::math)
opas_set_project_options(opas_rig)
