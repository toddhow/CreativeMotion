# 0004 — A full fork of OpenMW, not an upstream component

Date: 2026-10-18 · Decided by: toddhow

## Decision
The animation system goes into a full fork of OpenMW. It no longer targets upstream OpenMW. The fork starts from the Jolt branch (`jolt-physics` from cykoder's fork, the work behind merge request !3919), which is checked out at `I:\Projects\openmw`.

## Why
- toddhow's words: the change is "so massive" that it "would deviate too much from the original feel of the game" for upstream OpenMW to take it.

## Consequences
- The Path to OpenMW gate no longer waits for !3919 to merge upstream or for the maintainers to answer a proposal. M7 becomes preparing the fork. No proposal is posted.
- The implementation doc's "upstream where possible" home is replaced: the fork is the home. Merging upstream changes into the fork stays desirable but optional.
- The fork is GPL-3.0-or-later, like OpenMW and this repository (decision 0001). Its name is still open.
- This repository keeps its module boundaries. `core` stays free of engine types, so the M0–M6 sandbox, its tests and its replays still work outside the fork.
