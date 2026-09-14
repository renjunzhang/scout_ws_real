#!/usr/bin/env python3
"""Freeze one development candidate before four new Full/Smooth recordings."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import yaml

PROTOCOL = "SMPCC_C03_INTERNAL_SLOSH_DEV_V2"
PARAMETER_PROTOCOL = "SMPCC_C03_INTERNAL_SLOSH_DEV_V3"
SUPPORTED_PROTOCOLS = (PROTOCOL, PARAMETER_PROTOCOL)
ROWS = {"01": "full", "02": "smooth", "03": "smooth", "04": "full"}
ROOT = Path(__file__).resolve().parents[6]
PLANNER = Path("src/scout_apps/control/spmpc_local_planner")
LIQUID_KEYS = {
    "/spmpc_local_planner/variants/B_slosh/slosh_enable",
    "/spmpc_local_planner/variants/B_slosh/w_slosh",
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def digest_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def chain_identity(root=ROOT):
    """Freeze model/filter sources, loaded build artifacts and evaluator code."""
    directories = [PLANNER / "src", PLANNER / "include",
                   Path("src/scout_apps/control/slosh_models/src"),
                   Path("src/scout_apps/control/slosh_models/include")]
    files = [p for directory in directories for p in (root / directory).rglob("*") if p.is_file()]
    files += [root / p for p in (
        "devel/lib/spmpc_local_planner/spmpc_local_planner_node",
        "devel/lib/libspmpc_local_planner.so", "devel/lib/libslosh_models.so")]
    files += [root / PLANNER / "scripts/analysis" / name for name in (
        "analyze_internal_slosh_pair.py", "freeze_internal_slosh_evaluation.py",
        "validate_spmpc_comparison_recording.py", "validate_spmpc_ablation_smoke.py",
        "validate_explicit_actuator_runtime_smoke.py", "validate_i0_failclosed_fixed_abba_bag.py")]
    files += [root / PLANNER / "generated/acados" / variant / ("libacados_ocp_solver_" + variant + ".so")
              for variant in ("spmpc_b0", "spmpc_slosh")]
    manifest = {str(p.relative_to(root)): sha256(p) for p in sorted(files)}
    return {"sha256": digest_json(manifest), "files": manifest}


def _param(params, name):
    return params.get("/spmpc_local_planner/" + name, params.get(name))


def _effective_param(config, name):
    if not isinstance(config, dict):
        return None
    return config.get(name, config.get("variants/B_slosh/" + name,
                       config.get("/spmpc_local_planner/variants/B_slosh/" + name,
                                  config.get("ablation/" + name,
                                             config.get("/spmpc_local_planner/ablation/" + name)))))


def check_params(params, condition, weight, protocol=PROTOCOL, jerk_max=0.6, weights=None):
    if protocol not in SUPPORTED_PROTOCOLS:
        raise ValueError("unsupported parameter protocol")
    if not math.isfinite(float(jerk_max)) or float(jerk_max) not in (0.6, 1.0, 1.2):
        raise ValueError("invalid jerk_max")
    if protocol == PROTOCOL and float(jerk_max) != 0.6:
        raise ValueError("V2 freezes jerk_max=0.6")
    if condition not in ("full", "smooth") or not math.isfinite(weight) or not (0 < weight <= 20 if condition == "full" else weight >= 0):
        raise ValueError("invalid candidate condition/weight")
    prefix = "/spmpc_local_planner/"
    expected = {
        "planner_variant": "B_slosh", "slosh_observer/source": "processed_imu",
        "ablation/jerk_limit_enable": True, "ablation/jerk_max": float(jerk_max),
        "ablation/zero_liquid_initial_state": False,
        "variants/B_slosh/v_ref": 0.2,
        "variants/B_slosh/slosh_enable": condition == "full",
        "variants/B_slosh/w_slosh": weight if condition == "full" else 0,
    }
    for key, value in expected.items():
        if _param(params, key) != value:
            raise ValueError("unfrozen launch parameter: " + key)
    if protocol == PARAMETER_PROTOCOL:
        weights = weights or {}
        for key in ("w_v", "w_contour", "w_lag"):
            value = float(weights[key])
            if not math.isfinite(value) or not (0 < value <= 20) or _param(params, "variants/B_slosh/" + key) != value:
                raise ValueError("invalid or unfrozen " + key)


def read_candidate(path, condition, chain, protocol=None):
    report = json.loads(path.read_text())
    prereg = report.get("prereg", {})
    if not report.get("eligible_for_comparison") or report.get("status") != "PASS":
        raise ValueError("candidate must pass all recording/metric gates: " + str(path))
    candidate_protocol = prereg.get("protocol")
    if candidate_protocol == PARAMETER_PROTOCOL and prereg.get("evaluation_primary_monitor") != "imu":
        raise ValueError("V3 candidate must explicitly record imu primary monitor")
    requested = protocol or candidate_protocol or PROTOCOL
    if (candidate_protocol, prereg.get("phase"), prereg.get("condition")) != (requested, "screening", condition):
        raise ValueError("candidate is not a supported screening " + condition)
    if prereg.get("evaluation_chain_sha256") != chain["sha256"]:
        raise ValueError("candidate model/build/evaluator differs from current files")
    launch = Path(report["launch_params_file"])
    launch_sha = sha256(launch)
    if launch_sha != report.get("launch_params_sha256") or launch_sha != prereg.get("launch_params_sha256"):
        raise ValueError("candidate launch dump SHA mismatch")
    params = yaml.safe_load(launch.read_text())
    weight = float(prereg["w_slosh"])
    jerk_max = float(prereg.get("jerk_max", 0.6))
    weights = {key: float(prereg[key]) for key in ("w_v", "w_contour", "w_lag")} if requested == PARAMETER_PROTOCOL else None
    check_params(params, condition, weight, requested, jerk_max, weights)
    if requested == PARAMETER_PROTOCOL:
        config = report.get("effective_config_first", {})
        for key, expected in dict(weights, jerk_max=jerk_max).items():
            if not math.isclose(float(_effective_param(config, key)), expected,
                                rel_tol=1.0e-6, abs_tol=1.0e-6):
                raise ValueError("V3 effective_config mismatch: " + key)
    return report, params, launch_sha


def create_lock(full_path, smooth_path, primary, reason, output, chain=None, protocol=None):
    if output.exists():
        raise ValueError("preserve existing lock: " + str(output))
    if primary not in ("imu", "odom") or not reason.strip():
        raise ValueError("select imu/odom once and record a selection reason")
    chain = chain or chain_identity()
    protocol = protocol or PROTOCOL
    if protocol not in SUPPORTED_PROTOCOLS:
        raise ValueError("unsupported parameter protocol")
    if protocol == PARAMETER_PROTOCOL and primary != "imu":
        raise ValueError("V3 evaluation requires imu primary monitor")
    full, full_params, full_sha = read_candidate(full_path, "full", chain, protocol)
    smooth, smooth_params, smooth_sha = read_candidate(smooth_path, "smooth", chain, protocol)
    common = lambda params: {k: v for k, v in params.items() if k not in LIQUID_KEYS}
    if common(full_params) != common(smooth_params):
        raise ValueError("Full/Smooth non-liquid launch parameters differ")
    if any(full["prereg"].get(k) != smooth["prereg"].get(k) or not full["prereg"].get(k)
           for k in ("path_sha256", "map_sha256")):
        raise ValueError("candidate path/map mismatch or missing identity")
    # A candidate can be exploratory; freezing is not itself an efficacy claim.
    lock = {
        "schema_version": 1, "protocol": protocol, "created_at_epoch_sec": time.time(),
        "primary_monitor": primary, "selection_reason": reason, "rows": ROWS,
        "full_w_slosh": float(full["prereg"]["w_slosh"]),
        "jerk_max": float(full["prereg"].get("jerk_max", 0.6)),
        "launch_params_sha256": {"full": full_sha, "smooth": smooth_sha},
        "launch_params": {"full": full_params, "smooth": smooth_params},
        "evaluation_chain_sha256": chain["sha256"], "evaluation_chain_files": chain["files"],
        "path_sha256": full["prereg"]["path_sha256"], "map_sha256": full["prereg"]["map_sha256"],
        "candidates": [{"report": str(p.resolve()), "sha256": sha256(p), "bag": r["bag"]}
                       for p, r in ((full_path, full), (smooth_path, smooth))],
        "evaluation": {
            "signal": "modal_height_m * 1000; native state_stamp; sample RMS; no added filtering",
            "task": "first published nonzero command through first published GOAL_REACHED",
            "secondary_windows": "geometric path 10%-90%; GOAL_REACHED through +5 s",
            "primary_metric": "task height RMS mm; report both monitors and every attempt",
            "conclusion_scope": "frozen evaluation chain internal model metrics only",
        },
    }
    if protocol == PARAMETER_PROTOCOL:
        for key in ("w_v", "w_contour", "w_lag"):
            lock[key] = float(full["prereg"][key])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        json.dump(lock, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return lock


def check_lock(lock, condition, row, launch_text, path_sha, map_sha, chain=None, protocol=None):
    lock_protocol = lock.get("protocol")
    requested = protocol or lock_protocol or PROTOCOL
    if requested not in SUPPORTED_PROTOCOLS or lock.get("schema_version") != 1 or lock_protocol != requested or lock.get("rows") != ROWS:
        raise ValueError("invalid evaluation lock protocol")
    if ROWS.get(row) != condition or lock.get("primary_monitor") not in (("imu",) if requested == PARAMETER_PROTOCOL else ("imu", "odom")):
        raise ValueError("expected 01 full -> 02 smooth -> 03 smooth -> 04 full")
    if lock.get("evaluation_chain_sha256") != (chain or chain_identity())["sha256"]:
        raise ValueError("model/build/evaluator changed after freezing")
    if lock.get("path_sha256") != path_sha or lock.get("map_sha256") != map_sha:
        raise ValueError("path/map changed after freezing")
    if hashlib.sha256(launch_text.encode()).hexdigest() != lock["launch_params_sha256"][condition]:
        raise ValueError("launch parameters differ from frozen " + condition)
    weights = {key: lock[key] for key in ("w_v", "w_contour", "w_lag")} if requested == PARAMETER_PROTOCOL else None
    check_params(yaml.safe_load(launch_text), condition, lock["full_w_slosh"], requested,
                 lock.get("jerk_max", 0.6), weights)
    return lock["primary_monitor"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("fingerprint", help="print current evaluation chain SHA; no ROS access")
    create = sub.add_parser("create")
    create.add_argument("--full-report", type=Path, required=True)
    create.add_argument("--smooth-report", type=Path, required=True)
    create.add_argument("--primary-monitor", choices=("imu", "odom"), required=True)
    create.add_argument("--reason", required=True)
    create.add_argument("--output", type=Path, required=True)
    create.add_argument("--protocol", choices=SUPPORTED_PROTOCOLS, default=PROTOCOL)
    check = sub.add_parser("check", help="check current launch YAML from stdin before recording")
    check.add_argument("--lock", type=Path, required=True)
    check.add_argument("--condition", choices=("full", "smooth"), required=True)
    check.add_argument("--row", choices=tuple(ROWS), required=True)
    check.add_argument("--path-sha256", required=True)
    check.add_argument("--map-sha256", required=True)
    check.add_argument("--protocol", choices=SUPPORTED_PROTOCOLS)
    args = parser.parse_args(argv)
    try:
        if args.action == "fingerprint":
            print(chain_identity()["sha256"])
        elif args.action == "create":
            create_lock(args.full_report, args.smooth_report, args.primary_monitor, args.reason, args.output, protocol=args.protocol)
            print(str(args.output))
        else:
            print(check_lock(json.loads(args.lock.read_text()), args.condition, args.row,
                             sys.stdin.read(), args.path_sha256, args.map_sha256, protocol=args.protocol))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.exit(2, "evaluation freeze: {}\n".format(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
