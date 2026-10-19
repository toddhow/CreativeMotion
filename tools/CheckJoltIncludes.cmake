# Fails if any source file outside physics/jolt/ includes a Jolt header.
# Enforces the architecture rule that only the Jolt backend sees Jolt.
#
# Usage: cmake -DROOT=<directory to scan> -P tools/CheckJoltIncludes.cmake
# ROOT defaults to the repository root. Build output (build/, out/), .git and
# .claude (agent worktrees are full checkouts of the repository, Jolt backend
# included) are skipped. Written as a CMake script so it runs anywhere CMake does.

if(NOT DEFINED ROOT)
  get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
endif()

# Every C/C++ source or header extension, compared case-insensitively so that
# Foo.HPP is caught on case-sensitive file systems too.
set(source_extension_regex "\\.(h|hh|hpp|hxx|h\\+\\+|c|cc|cpp|cxx|c\\+\\+|cppm|ixx|inl|ipp|tpp)$")

# Glob each top-level entry separately so build trees are never walked.
file(GLOB top_level RELATIVE "${ROOT}" "${ROOT}/*")
set(candidates "")
foreach(entry IN LISTS top_level)
  if(entry MATCHES "^(build|out|\\.git|\\.claude)$")
    continue()
  endif()
  if(IS_DIRECTORY "${ROOT}/${entry}")
    file(GLOB_RECURSE files RELATIVE "${ROOT}" "${ROOT}/${entry}/*")
    list(APPEND candidates ${files})
  else()
    list(APPEND candidates "${entry}")
  endif()
endforeach()

set(violations "")
foreach(path IN LISTS candidates)
  string(TOLOWER "${path}" lower_path)
  if(NOT lower_path MATCHES "${source_extension_regex}" OR path MATCHES "^physics/jolt/")
    continue()
  endif()
  file(STRINGS "${ROOT}/${path}" lines REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]Jolt/")
  foreach(line IN LISTS lines)
    string(STRIP "${line}" line)
    list(APPEND violations "${path}: ${line}")
  endforeach()
endforeach()

if(violations)
  list(JOIN violations "\n  " report)
  message(FATAL_ERROR "Jolt headers may only be included under physics/jolt/:\n  ${report}")
endif()
message(STATUS "Jolt include check passed")
