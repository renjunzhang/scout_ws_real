#!/usr/bin/env python3
"""QC v2 entry point paired with the exact damping-scheme inventory revision.

All numerical parsing, 17-metric thresholds and verdict semantics remain in
the frozen v1 QC implementation.  This entry point only activates the v2 exact
inventory reader and binds the reported QC script identity to this file.
"""

from __future__ import annotations

from pathlib import Path

import r8_liquid_u3_damped_restart_qc_v1 as base_qc
import r8_liquid_u3_gpu_damped_restart_runner_v2 as runner_v2


SCRIPT_PATH = Path(__file__).resolve()
BASE_QC_V1_SHA256 = "81f8997aed9581d7e0dcd4e5c7f04e1957171fdacfe47d6b386635842b07921f"


def configure() -> None:
    if runner_v2.legacy.sha256_file(base_qc.SCRIPT_PATH, maximum=4 * 1024 * 1024) != BASE_QC_V1_SHA256:
        raise base_qc.DampedRestartQcError("v1 QC dependency identity drifted")
    runner_v2.configure()
    base_qc.SCRIPT_PATH = SCRIPT_PATH
    base_qc.runner.SCRIPT_PATH = runner_v2.SCRIPT_PATH
    base_qc.runner.TEST_PATH = runner_v2.TEST_PATH
    base_qc.runner.QC_PATH = SCRIPT_PATH


def main(argv: list[str] | None = None) -> int:
    configure()
    return base_qc.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
