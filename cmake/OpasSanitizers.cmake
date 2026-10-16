# Sanitizer support for the `asan` preset.
#
# Flags are added globally (add_compile_options / add_link_options) rather than
# per target because every library in the process, including Jolt and
# GoogleTest, has to be built the same way: mixing instrumented and
# uninstrumented code gives false reports, especially with MSVC.

function(opas_enable_sanitizers_globally)
  if(NOT OPAS_SANITIZE)
    return()
  endif()

  if(MSVC)
    # MSVC supports AddressSanitizer only; UBSan is checked on Linux in CI.
    add_compile_options(/fsanitize=address /Zi)
    # Incremental linking is incompatible with /fsanitize=address.
    add_link_options(/INCREMENTAL:NO)
  else()
    set(flags -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
    add_compile_options(${flags})
    add_link_options(${flags})
  endif()
endfunction()
