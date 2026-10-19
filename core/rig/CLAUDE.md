# core/rig — skeleton, rig and profile model, with their file formats

Depends on `core/math`, `core/format` (LoadError) and, privately, `opas::format_json` (nlohmann/json). No JSON type may appear in a public header.

## Rules
- Bones are ordered parents-first; `validateSkeleton` enforces it, and `computeWorldRest` relies on it.
- Rest transforms must be finite and rotations unit length (NaN is rejected explicitly: comparisons with NaN are always false, so checks are written as `!(x <= tol)`).
- Character data comes from files, never from code. Tests build synthetic skeletons only — no game-derived bone data.
- Formats: `docs/formats/rig.md`, `docs/formats/profile.md`. Change a format, its loader, writer and round-trip tests together.
- One set of rules: `detail::findRigProblem` / `findProfileProblem` (src/Rig.cpp, src/Profile.cpp) check a complete model and return a JSON pointer. `validateRig`/`validateProfile` print it; the loaders look the pointer up in the parsed document to get the line. Add a rule there, never only in a reader, so programmatic and loaded data follow the same rules. Readers check only types, finiteness and unit rotations.
- Joints are keyed by the child bone and connect to the nearest ancestor *body* (`parentBody`), skipping bones without bodies. Exactly one root body; every other body has a joint.
- Rig types mirror `opas::physics` shapes and `JointDesc`, but core/rig includes no physics header; M3 converts.

## Pitfalls
- `computeWorldRest` and `parentBody` assume a valid skeleton or rig; validate first.
- A rest rotation more than 1e-6 from unit length is rejected; authored data must be normalized.
- Bone parents are names in files and indices in `Skeleton`; the reader resolves them only after every bone is read, so "listed after its child" and "unknown" get different messages.
- A helper named `quoted` collides with `std::quoted` through argument-dependent lookup; these sources use `inQuotes`.
- Version 1 swing cones are symmetric about the rest pose, so asymmetric ranges (hip flexion vs extension) are only approximated (`data/rigs/README.md`).
