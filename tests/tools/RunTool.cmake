# Runs a command-line tool and checks its exit code and output (used by ctest).
# ctest's WILL_FAIL cannot tell exit code 1 from 2, so the tool tests go through this script:
#
#   cmake -DTOOL=<exe> "-DARGS=<arg>|<arg>|..." -DEXPECT_EXIT=<n>
#         ["-DR1=<regex>" ... "-DR8=<regex>"]   output (stdout + stderr) must match each
#         ["-DN1=<regex>" ... "-DN4=<regex>"]   output must not match any
#         [-DEXPECT_LINES=<n>]                  number of non-empty output lines
#         -P RunTool.cmake
#
# ARGS uses "|" between arguments so that paths and values survive being passed through ctest.

if(NOT DEFINED TOOL OR NOT DEFINED EXPECT_EXIT)
  message(FATAL_ERROR "RunTool.cmake needs -DTOOL=... and -DEXPECT_EXIT=...")
endif()

set(tool_args "")
if(DEFINED ARGS AND NOT "${ARGS}" STREQUAL "")
  string(REPLACE "|" ";" tool_args "${ARGS}")
endif()

execute_process(
  COMMAND "${TOOL}" ${tool_args}
  RESULT_VARIABLE exit_code
  OUTPUT_VARIABLE tool_out
  ERROR_VARIABLE tool_err
  TIMEOUT 60
)
set(output "${tool_out}${tool_err}")
set(failed FALSE)

if(NOT "${exit_code}" STREQUAL "${EXPECT_EXIT}")
  message(SEND_ERROR "expected exit code ${EXPECT_EXIT}, got '${exit_code}'")
  set(failed TRUE)
endif()

foreach(i RANGE 1 8)
  if(DEFINED R${i} AND NOT "${R${i}}" STREQUAL "")
    if(NOT output MATCHES "${R${i}}")
      message(SEND_ERROR "output does not match R${i} = ${R${i}}")
      set(failed TRUE)
    endif()
  endif()
endforeach()

foreach(i RANGE 1 4)
  if(DEFINED N${i} AND NOT "${N${i}}" STREQUAL "")
    if(output MATCHES "${N${i}}")
      message(SEND_ERROR "output unexpectedly matches N${i} = ${N${i}}")
      set(failed TRUE)
    endif()
  endif()
endforeach()

if(DEFINED EXPECT_LINES)
  string(REGEX MATCHALL "[^\r\n]+" lines "${output}")
  list(LENGTH lines line_count)
  if(NOT line_count EQUAL EXPECT_LINES)
    message(SEND_ERROR "expected ${EXPECT_LINES} output lines, got ${line_count}")
    set(failed TRUE)
  endif()
endif()

if(failed)
  message(FATAL_ERROR "tool output was:\n${output}\n(exit code ${exit_code})")
endif()
