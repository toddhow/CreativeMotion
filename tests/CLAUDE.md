# tests — the single test runner

All tests build into one executable, `opas_tests`, run by `ctest --preset <preset>`.
Two exceptions: `opas_interface_tests` (`unit/physics_interface/`) links only `opas::physics_interface` to prove the interface headers build without Jolt (never link Jolt into it); `opas_alloc_tests` (`alloc/`) replaces the global `operator new`/`delete` with counting versions to check that per-step code never allocates, so it must stay a separate executable.

## Rules
- Folders: `unit/`, `property/`, `replay/`, `roundtrip/`. Inside, mirror the module path (`unit/physics_jolt/` tests `physics/jolt`).
- Register every new file in `tests/CMakeLists.txt`.
- Tests are written from the task spec, not from the implementation. The implementing agent may not edit or delete tests without toddhow's approval.
- No wall-clock time, no unseeded randomness, no dependence on test order.

## Pitfalls
- Tests are discovered when ctest runs (`DISCOVERY_MODE PRE_TEST`), so a test that crashes on load shows up as a ctest failure, not a build failure.
