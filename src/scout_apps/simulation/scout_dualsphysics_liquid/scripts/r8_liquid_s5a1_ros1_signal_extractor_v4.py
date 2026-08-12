#!/usr/bin/env python3
"""Reader-v4 exact-file adapter for the frozen extractor-v3 semantics."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v4 as reader_v4
import r8_liquid_s5a1_ros1_signal_extractor_v1 as extractor_v1
import r8_liquid_s5a1_ros1_signal_extractor_v3 as extractor_v3

sys.dont_write_bytecode = True
SignalExtractionError = extractor_v3.SignalExtractionError
nearest_clock_alignment = extractor_v1.nearest_clock_alignment
tf_cross_check = extractor_v1.tf_cross_check
extract_bag_bytes = extractor_v3.extract_bag_bytes
build_motion_samples = extractor_v3.build_motion_samples


def read_exact_primary(path: Path, *, expected_size: int, expected_mode: int,
                       expected_sha256: str) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    data, before, after = reader_v4.read_regular_file(
        path, limits=reader_v4.ReaderLimits(), expected_sha256=expected_sha256,
        expected_size_bytes=expected_size, expected_mode=expected_mode,
    )
    return extractor_v3.extract_bag_bytes(data), before, after


__all__ = ["SignalExtractionError", "extract_bag_bytes", "build_motion_samples",
           "nearest_clock_alignment", "tf_cross_check", "read_exact_primary"]
