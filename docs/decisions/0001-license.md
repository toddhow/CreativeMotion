# 0001 — Project license: GPL-3.0-or-later

Date: 2026-10-16 · Decided by: toddhow

## Decision
The project is licensed under the GNU General Public License, version 3 or (at your option) any later version (SPDX: `GPL-3.0-or-later`). The full text is in `LICENSE`, taken unmodified from https://www.gnu.org/licenses/gpl-3.0.txt.

## Why
- The code is headed into OpenMW, which is GPLv3, so matching it removes any licensing question at integration time.
- "Or later" is the FSF's standard wording and lets the project follow a future GPL version without asking every contributor.

## Consequences
- Anyone who distributes the system or a modified version must release its source under the GPL.
- Dependencies must be GPLv3-compatible. The current ones (Jolt: MIT, GoogleTest: BSD-3-Clause) are.
- Options the guide listed earlier (MIT, zlib, MPL) are no longer in play.
- Per-file SPDX headers are not required yet; add them if a contributor or OpenMW asks.
