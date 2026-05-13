#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke tests for OSCRS tail protection helpers."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, SCRIPT_DIR)

from oscrs.generators.tail_protect import (  # noqa: E402
    replace_raw_tail,
    tail_deviation,
    tail_heading_error_deg,
)


def main():
    base = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    candidate = [(0.0, 0.0), (1.0, 0.2), (2.0, 0.2), (3.0, 0.0)]
    protected = replace_raw_tail(candidate, base, 1.1)
    assert protected[-2:] == base[-2:]
    assert tail_deviation(protected, base, 1.1) < 1e-9
    assert tail_heading_error_deg(protected, base) < 1e-9
    print("OK")


if __name__ == "__main__":
    main()
