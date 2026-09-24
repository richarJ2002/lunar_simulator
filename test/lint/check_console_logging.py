"""!
@brief  CTest lint for WP-01's console rule: project C++ under src/ logs only
        through the LUNAR_LOG_* macros of src/common/console/console.h,
        which wrap every message at 40 characters of text per line. Direct
        RCLCPP_* logging calls outside that module fail the check.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Optional, Sequence

# rclcpp's logging macro family, including throttled/once/stream variants.
DIRECT_LOGGING_PATTERN = re.compile(r"\bRCLCPP_(DEBUG|INFO|WARN|ERROR|FATAL)\w*\s*\(")

# The one module allowed to call RCLCPP_* directly, relative to src/.
CONSOLE_MODULE = Path("common") / "console"

# C++ source and header suffixes this project uses.
CPP_SUFFIXES = (".h", ".hpp", ".cc", ".cpp", ".tpp")


def find_direct_logging_calls(source_root: Path) -> list[str]:
    """!
    @brief   Finds every direct RCLCPP_* logging call outside the console
             module.

    @param   source_root
             The project's src/ directory.

    @return  One "path:line: text" entry per offending line, sorted by
             path; empty when the rule holds.
    """
    # Collect offenders as readable, clickable locations.
    offenders: list[str] = []
    # Visit every C++ file under src/ in a stable order.
    for path in sorted(source_root.rglob("*")):
        # Skip directories and non-C++ files.
        if not path.is_file() or path.suffix not in CPP_SUFFIXES:
            continue
        # The console module itself is the sanctioned caller.
        relative_path = path.relative_to(source_root)
        if relative_path.parts[: len(CONSOLE_MODULE.parts)] == CONSOLE_MODULE.parts:
            continue
        # Report each matching line with its 1-based line number.
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1
        ):
            if DIRECT_LOGGING_PATTERN.search(line):
                offenders.append(f"{path}:{line_number}: {line.strip()}")
    return offenders


def main(argv: Optional[Sequence[str]] = None) -> int:
    """!
    @brief   Command-line entry point: checks one source tree.

    @param   argv
             Argument vector to parse; defaults to `sys.argv[1:]` when
             `None`.

    @return  `0` when no direct call exists, `1` when any does, `2` when
             the source root is missing.
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    # A missing tree would otherwise pass vacuously.
    if not args.source_root.is_dir():
        print(f"error: source root not found: {args.source_root}", file=sys.stderr)
        return 2
    offenders = find_direct_logging_calls(args.source_root)
    # Print every offender so the fix list is complete in one run.
    for offender in offenders:
        print(offender)
    if offenders:
        print(
            f"{len(offenders)} direct RCLCPP_* logging call(s); use the "
            "LUNAR_LOG_* macros from console/console.h instead",
            file=sys.stderr,
        )
        return 1
    print("console logging lint: no direct RCLCPP_* calls outside src/common/console")
    return 0


def _build_arg_parser() -> argparse.ArgumentParser:
    """!
    @brief   Builds the command-line argument parser for this lint.

    @return  A configured, not-yet-invoked `ArgumentParser`.
    """
    # Describe the rule so `-h` explains why the check exists.
    parser = argparse.ArgumentParser(
        description=(
            "Fail if project C++ logs with RCLCPP_* directly instead of the "
            "40-character-wrapping LUNAR_LOG_* macros."
        )
    )
    # The tree to scan, normally the repository's src/.
    parser.add_argument("source_root", type=Path, help="Project src/ directory to scan.")
    return parser


if __name__ == "__main__":
    raise SystemExit(main())
