# tests — the single test runner

All tests build into one executable, `opas_tests`, run by `ctest --preset <preset>`.

## Rules
- Folders: `unit/`, `property/`, `replay/`, `roundtrip/`. Inside, mirror the module path (`unit/physics_jolt/` tests `physics/jolt`).
- Register every new file in `tests/CMakeLists.txt`.
- Tests are written from the task spec, not from the implementation. The implementing agent may not edit or delete tests without toddhow's approval.
- No wall-clock time, no unseeded randomness, no dependence on test order.

## Pitfalls
- Tests are discovered when ctest runs (`DISCOVERY_MODE PRE_TEST`), so a test that crashes on load shows up as a ctest failure, not a build failure.
