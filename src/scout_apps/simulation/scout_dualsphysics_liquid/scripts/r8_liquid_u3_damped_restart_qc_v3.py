#!/usr/bin/env python3
"""QC entry point paired with the explicit 3 GiB resource revision."""

from __future__ import annotations

from pathlib import Path

import r8_liquid_u3_damped_restart_qc_v1 as base_qc
import r8_liquid_u3_damped_restart_qc_v2 as qc_v2
import r8_liquid_u3_gpu_damped_restart_runner_v3 as runner_v3


SCRIPT_PATH = Path(__file__).resolve()
QC_V2_SHA256 = "4087ae64985af0e0454e48b0eed1566145244b85a18e1792498b9ce23ce5a402"


def configure() -> None:
    if runner_v3.legacy.sha256_file(qc_v2.SCRIPT_PATH, maximum=4 * 1024 * 1024) != QC_V2_SHA256:
        raise base_qc.DampedRestartQcError("v2 QC dependency identity drifted")
    runner_v3.configure()
    base_qc.SCRIPT_PATH = SCRIPT_PATH
    base_qc.runner.SCRIPT_PATH = runner_v3.SCRIPT_PATH
    base_qc.runner.SCHEMA_PATH = runner_v3.SCHEMA_PATH
    base_qc.runner.TEST_PATH = runner_v3.TEST_PATH
    base_qc.runner.QC_PATH = SCRIPT_PATH
    base_qc.runner.semantic_validate = runner_v3.semantic_validate_v3


def main(argv: list[str] | None = None) -> int:
    configure()
    return base_qc.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
