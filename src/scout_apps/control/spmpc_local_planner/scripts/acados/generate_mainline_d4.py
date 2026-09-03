#!/usr/bin/env python3
"""Generate and atomically publish one Stage 3-D4 DEV_UNVALIDATED artifact."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPTS_ROOT = Path(__file__).resolve().parents[1]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from acados.mainline.constraints_oracle import ConstraintBounds
from acados.mainline.d4_generator import (
    D4GenerationError,
    D4GenerationRequest,
    generate_d4_development_artifact,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", required=True)
    parser.add_argument("--output-directory", required=True)
    parser.add_argument("--acados-install-root", required=True)
    parser.add_argument("--tera-executable", required=True)
    parser.add_argument("--q-issue-v-max", required=True, type=float)
    parser.add_argument("--q-issue-omega-max", required=True, type=float)
    parser.add_argument("--a-issue-max", required=True, type=float)
    parser.add_argument("--alpha-issue-max", required=True, type=float)
    parser.add_argument("--jerk-v-max", required=True, type=float)
    parser.add_argument("--jerk-omega-max", required=True, type=float)
    parser.add_argument("--v-s-max", required=True, type=float)
    parser.add_argument("--integer-snap-tolerance-sec", required=True, type=float)
    parser.add_argument("--duration-tolerance-sec", required=True, type=float)
    parser.add_argument("--ext-fun-compile-flags", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        bounds = ConstraintBounds(
            q_issue_v_max=args.q_issue_v_max,
            q_issue_omega_max=args.q_issue_omega_max,
            a_issue_max=args.a_issue_max,
            alpha_issue_max=args.alpha_issue_max,
            jerk_v_max=args.jerk_v_max,
            jerk_omega_max=args.jerk_omega_max,
            v_s_max=args.v_s_max,
        )
        request = D4GenerationRequest(
            repository_root=Path(args.repository_root),
            output_directory=Path(args.output_directory),
            acados_install_root=Path(args.acados_install_root),
            tera_executable=Path(args.tera_executable),
            constraint_bounds=bounds,
            integer_snap_tolerance_sec=args.integer_snap_tolerance_sec,
            duration_tolerance_sec=args.duration_tolerance_sec,
            ext_fun_compile_flags=args.ext_fun_compile_flags,
        )
        result = generate_d4_development_artifact(request)
    except (D4GenerationError, TypeError, ValueError) as exc:
        parser.exit(1, f"D4 generation failed: {exc}\n")
    print(json.dumps(result.to_dict(), ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
