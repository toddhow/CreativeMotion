# 0002 — JSON for authored data files

Date: 2026-10-16 · Decided by: toddhow

## Decision
Authored data files (interaction markers now; rigs and profiles in M2) are JSON, the same as recordings. Parsing uses nlohmann/json v3.12.0 (MIT) with `JSON_Diagnostic_Positions`. Writing is our own canonical writer so round-trips are byte-for-byte.

## Why
- One format for everything; strict syntax; byte-for-byte round-trips are easy with our own writer.
- nlohmann/json's diagnostic positions give the line numbers the guide requires in loader errors.

## Consequences
- No comments in data files. Use descriptive ids and the format docs in `docs/formats/` instead.
- The YAML option in the guide's tech stack is dropped.
