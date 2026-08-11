#!/usr/bin/env python3
"""Upstream-default artificial-viscosity revision of the damped restart run.

V3 proved that initialization-only damping safely creates and restarts the
expected state, but its undamped tail retained particle-scale velocity noise.
The sealed DualSPHysics v5.4 template documents Artificial viscosity=0.01 as
the default.  This revision adds exactly ``-viscoart:0.01`` to both phases;
all damping, CFL, DDT, Shifting, dp, isolation, inventory, resource and
17-metric contracts remain inherited unchanged from v3.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import r8_liquid_u3_gpu_damped_restart_runner_v1 as base
import r8_liquid_u3_gpu_damped_restart_runner_v3 as resource_v3
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_damped_restart_policy_v2.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_damped_restart_runner_v4.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_damped_restart_qc_v4.py"
RESOURCE_RUNNER_V3_SHA256 = "a91cb8ff47c3931c3cde60985a83d68462f7ca870ca80f10d9a4e2455386908c"
PRIOR_FAILED_QC_SHA256 = "e6737dcc0a3211fb9cf92c8839f19e663cd3de2b39a2cc4e63cde689f530ec38"
VISCO_ARG = "-viscoart:0.01"
UPSTREAM_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
UPSTREAM_PARAMETER_TEMPLATE_BLOB = "a340a1737199064ac667efd1cb0012b6a2f350a9"
UPSTREAM_PARAMETER_TEMPLATE_SHA256 = "0b3455763ac68d842f9202cab022638f183461ed18ab4259a15ef246ff97e3ea"


def exact_solver_argv_v4(phase_name: str) -> list[str]:
    argv = list(base._exact_solver_argv(phase_name))
    insertion = argv.index("-shifting:none") + 1
    argv.insert(insertion, VISCO_ARG)
    return argv


def _verify_viscosity_source(policy: dict[str, Any]) -> dict[str, Any]:
    parser = Path(policy["parents"]["restart_source"]["path"]).read_text(encoding="utf-8")
    required = (
        'printf("    -viscoart:<float>          Artificial viscosity [0-1]\\n")',
        'else if(txword=="VISCOART")',
        "TVisco=VISCO_Artificial",
        "if(Visco>10)ErrorParm",
    )
    if any(token not in parser for token in required):
        raise base.DampedRestartRunError("sealed artificial-viscosity CLI semantics differ")
    return {
        "status": "PASS_SEALED_ARTIFICIAL_VISCOSITY_CLI_SOURCE",
        "exact_solver_argument": VISCO_ARG,
        "upstream_commit": UPSTREAM_COMMIT,
        "upstream_parameter_template_blob": UPSTREAM_PARAMETER_TEMPLATE_BLOB,
        "upstream_parameter_template_sha256": UPSTREAM_PARAMETER_TEMPLATE_SHA256,
        "upstream_default": {"ViscoTreatment": 1, "Visco": 0.01},
    }


def semantic_validate_v4(policy: dict[str, Any], policy_path: Path) -> dict[str, Any]:
    if policy["parents"]["stage4_failed_adjudication"]["sha256"] != PRIOR_FAILED_QC_SHA256:
        raise base.DampedRestartRunError("v4 does not parent the exact failed damped-tail QC")
    for phase_name in base.PHASE_KEYS:
        argv = policy["phases"][phase_name]["solver_argv"]
        if argv != exact_solver_argv_v4(phase_name) or argv.count(VISCO_ARG) != 1:
            raise base.DampedRestartRunError(f"{phase_name} artificial-viscosity argv differs")
        if any(item.startswith(("-viscolam:", "-viscolamsps:")) for item in argv):
            raise base.DampedRestartRunError(f"{phase_name} carries a second viscosity override")

    shadow = copy.deepcopy(policy)
    for phase_name in base.PHASE_KEYS:
        shadow["phases"][phase_name]["solver_argv"] = base._exact_solver_argv(phase_name)
    inherited = resource_v3.semantic_validate_v3(shadow, policy_path)
    inherited["viscosity_revision"] = _verify_viscosity_source(policy)
    inherited["viscosity_revision"].update({
        "single_delta_from_v3": "LAMINAR_1E-6_TO_UPSTREAM_DEFAULT_ARTIFICIAL_0P01",
        "applied_to_phases": list(base.PHASE_KEYS),
        "other_numerical_contract_changed": False,
    })
    return inherited


def configure() -> None:
    if legacy.sha256_file(resource_v3.SCRIPT_PATH, maximum=4 * 1024 * 1024) != RESOURCE_RUNNER_V3_SHA256:
        raise base.DampedRestartRunError("v3 resource runner dependency identity drifted")
    resource_v3.configure()
    base.SCRIPT_PATH = SCRIPT_PATH
    base.SCHEMA_PATH = SCHEMA_PATH
    base.TEST_PATH = TEST_PATH
    base.QC_PATH = QC_PATH
    base.semantic_validate = semantic_validate_v4


def main(argv: list[str] | None = None) -> int:
    configure()
    return base.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
