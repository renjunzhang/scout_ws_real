#!/usr/bin/env python3
"""Official 3D-example artificial-viscosity strength revision.

V4 proved the formulation change is effective but ``-viscoart:0.01`` still
misses the frozen particle-velocity limits.  The sealed upstream v5.4 3D
DamBreak example uses the same Artificial formulation with ``Visco=0.1``.
This revision changes only that scalar in both phases; all damping, CFL, DDT,
Shifting, dp, isolation, inventory, resources and 17 limits remain inherited.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_damped_restart_runner_v4 as prior_v4
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_damped_restart_policy_v2.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_damped_restart_runner_v5.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_damped_restart_qc_v5.py"
PRIOR_RUNNER_V4_SHA256 = "128b819db2f3cae120070010a0a0fc12f2264c157fbe1cf4f10105b304c3b40e"
PRIOR_FAILED_QC_SHA256 = "63ba5a62bfd0605e1a9fd1c9bbf7b1cc9ecf274b821d75fc6db52b30fdca2a82"
VISCO_ARG = "-viscoart:0.1"
UPSTREAM_COMMIT = prior_v4.UPSTREAM_COMMIT
UPSTREAM_EXAMPLE_PATH = "examples/main/01_DamBreak/CaseDambreak_Def.xml"
UPSTREAM_EXAMPLE_BLOB = "6c96b6898bdd198e7af037e1a4c9cc5f7d68b3c4"
UPSTREAM_EXAMPLE_SHA256 = "6457dace118a01efd8052e60d335d35242806f84b8d5991db6981c443f978943"


def exact_solver_argv_v5(phase_name: str) -> list[str]:
    argv = prior_v4.exact_solver_argv_v4(phase_name)
    index = argv.index(prior_v4.VISCO_ARG)
    argv[index] = VISCO_ARG
    return argv


def semantic_validate_v5(policy: dict[str, Any], policy_path: Path) -> dict[str, Any]:
    if policy["parents"]["stage4_failed_adjudication"]["sha256"] != PRIOR_FAILED_QC_SHA256:
        raise base.DampedRestartRunError("v5 does not parent the exact failed v4 viscosity QC")
    for phase_name in base.PHASE_KEYS:
        argv = policy["phases"][phase_name]["solver_argv"]
        if argv != exact_solver_argv_v5(phase_name) or argv.count(VISCO_ARG) != 1:
            raise base.DampedRestartRunError(f"{phase_name} artificial-viscosity strength differs")
        viscosity_flags = [item for item in argv if item.startswith(("-viscoart:", "-viscolam:", "-viscolamsps:"))]
        if viscosity_flags != [VISCO_ARG]:
            raise base.DampedRestartRunError(f"{phase_name} carries a second viscosity override")

    shadow = copy.deepcopy(policy)
    for phase_name in base.PHASE_KEYS:
        shadow["phases"][phase_name]["solver_argv"] = prior_v4.exact_solver_argv_v4(phase_name)
    shadow["parents"]["stage4_failed_adjudication"]["sha256"] = prior_v4.PRIOR_FAILED_QC_SHA256
    inherited = prior_v4.semantic_validate_v4(shadow, policy_path)
    inherited["viscosity_strength_revision"] = {
        "status": "PASS_SEALED_UPSTREAM_3D_EXAMPLE_VISCOSITY_STRENGTH_CONTRACT",
        "single_delta_from_v4": "ARTIFICIAL_0P01_TO_UPSTREAM_3D_EXAMPLE_0P1",
        "baseline_argument": prior_v4.VISCO_ARG,
        "candidate_argument": VISCO_ARG,
        "applied_to_phases": list(base.PHASE_KEYS),
        "upstream_commit": UPSTREAM_COMMIT,
        "upstream_example_path": UPSTREAM_EXAMPLE_PATH,
        "upstream_example_blob": UPSTREAM_EXAMPLE_BLOB,
        "upstream_example_sha256": UPSTREAM_EXAMPLE_SHA256,
        "other_numerical_contract_changed": False,
    }
    return inherited


def configure() -> None:
    if legacy.sha256_file(prior_v4.SCRIPT_PATH, maximum=4 * 1024 * 1024) != PRIOR_RUNNER_V4_SHA256:
        raise base.DampedRestartRunError("v4 runner dependency identity drifted")
    prior_v4.configure()
    base.SCRIPT_PATH = SCRIPT_PATH
    base.SCHEMA_PATH = SCHEMA_PATH
    base.TEST_PATH = TEST_PATH
    base.QC_PATH = QC_PATH
    base.semantic_validate = semantic_validate_v5


def main(argv: list[str] | None = None) -> int:
    configure()
    return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
