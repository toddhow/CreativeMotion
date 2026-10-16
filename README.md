# Open Procedural Animation System

Physics-driven procedural animation: a ragdoll that tracks animated poses, balances and reacts to hits. The first build is standalone (behavior core, Jolt backend, sandbox); it later moves into OpenMW. See `CLAUDE.md` for how the project is built and run.

## Build

Requirements: CMake ≥ 3.25, Ninja, and a C++20 compiler (MSVC 2022, GCC, or Clang). Dependencies are fetched automatically at pinned versions.

On Windows, run these from a "x64 Native Tools Command Prompt for VS 2022" (or a shell with the MSVC environment loaded):

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Presets: `debug`, `release`, `asan` (AddressSanitizer; plus UBSan on GCC/Clang).

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version. See [LICENSE](LICENSE) for the full text.

Third-party dependencies keep their own licenses; see [docs/dependencies.md](docs/dependencies.md).
