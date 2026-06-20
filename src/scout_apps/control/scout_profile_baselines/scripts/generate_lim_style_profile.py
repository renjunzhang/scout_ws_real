#!/usr/bin/env python3
"""Stable rosrun entrypoint for the Lim-style profile baseline."""

import runpy
import sys
from pathlib import Path

TARGET = Path(__file__).resolve().parent / "lim" / "generate_profile.py"


def main():
    if not TARGET.is_file():
        raise RuntimeError(f"Lim-style implementation script is missing: {TARGET}")
    sys.argv[0] = str(TARGET)
    runpy.run_path(str(TARGET), run_name="__main__")


if __name__ == "__main__":
    main()
