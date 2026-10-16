#!/usr/bin/env python3
"""Run clang-tidy on this project's own sources (never on fetched dependencies).

Usage: python3 tools/run_clang_tidy.py <build-dir> [clang-tidy executable]

Reads <build-dir>/compile_commands.json, keeps the entries whose source file
lives in one of our module folders, and runs clang-tidy on each. Exits non-zero
if any file reports a problem (.clang-tidy turns every warning into an error).
"""

import json
import subprocess
import sys
from pathlib import Path

OUR_MODULES = ("core", "physics", "sandbox", "tests")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    repo_root = Path(__file__).resolve().parent.parent
    build_dir = Path(sys.argv[1]).resolve()
    clang_tidy = sys.argv[2] if len(sys.argv) > 2 else "clang-tidy"

    commands = json.loads((build_dir / "compile_commands.json").read_text(encoding="utf-8"))

    files = []
    for entry in commands:
        source = Path(entry["directory"], entry["file"]).resolve()
        try:
            relative = source.relative_to(repo_root)
        except ValueError:
            continue  # outside the repository, e.g. a fetched dependency
        if relative.parts and relative.parts[0] in OUR_MODULES:
            files.append(source)

    if not files:
        print("No project sources found in compile_commands.json", file=sys.stderr)
        return 1

    failed = []
    for source in sorted(set(files)):
        print(f"clang-tidy {source.relative_to(repo_root)}", flush=True)
        result = subprocess.run([clang_tidy, "-p", str(build_dir), "--quiet", str(source)])
        if result.returncode != 0:
            failed.append(source)

    if failed:
        print(f"\nclang-tidy reported problems in {len(failed)} file(s).", file=sys.stderr)
        return 1
    print(f"\nclang-tidy clean: {len(files)} file(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
