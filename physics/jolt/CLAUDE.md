# physics/jolt — Jolt backend

The only module allowed to include Jolt headers or link the `Jolt` target.

## Rules
- Link Jolt `PRIVATE`. Public headers in `include/` must never include a Jolt header or expose a Jolt type; callers see only our own types (later: `physics/interface`).
- Include `<Jolt/Jolt.h>` before any other Jolt header (Jolt requirement, see `HelloWorld/HelloWorld.cpp` in the Jolt repo).
- Jolt keeps global state (`JPH::Factory::sInstance`, registered types, allocator hooks). Whoever sets it owns tearing it down; leave globals clean.
- Jolt build options are set in `cmake/OpasDependencies.cmake` and explained in `docs/dependencies.md`. `CROSS_PLATFORM_DETERMINISTIC` is ON — don't turn it off without updating golden replays and that doc.

## Pitfalls
- Jolt compiles with RTTI and exceptions off by default; that only applies inside Jolt's own targets, not ours.
- Jolt's public compile definitions (`JPH_*`) flow to us through the `Jolt` target. Never redefine them by hand, or Jolt's headers and library will disagree on layouts.
- Swing-twist constraint space is X = twist, Y = plane × twist, Z = plane axis (`SwingTwistConstraint.cpp` constructor). Targets passed to `SetTargetOrientationCS` use that frame; M1's interface comment had Y and Z swapped until M3.
- Collision groups: one `GroupFilterTable` per group, shared by reference count with its bodies. The Jolt group ID is a serial number that is never reused, so bodies of a destroyed group are never mistaken for members of a new group in a reused slot. `GroupFilterTable::DisableCollision` wants the smaller sub-group first.
