# core/rig — skeleton and (from M2) rig and profile model

## Rules
- Bones are ordered parents-first; `validateSkeleton` enforces it, and `computeWorldRest` relies on it.
- Rest transforms must be finite and rotations unit length (NaN is rejected explicitly: comparisons with NaN are always false, so checks are written as `!(x <= tol)`).
- Character data comes from files (M2), never from code. Tests build synthetic skeletons only — no game-derived bone data.

## Pitfalls
- `computeWorldRest` assumes a valid skeleton; validate first.
- A rest rotation more than 1e-6 from unit length is rejected; authored data must be normalized.
