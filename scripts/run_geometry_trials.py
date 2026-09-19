#!/usr/bin/env python3
"""Serial model-in-the-loop comparisons; no ROS imports or robot I/O.

Use the pinned code-generation Python environment. Reports include all failed
trials. A successful plan is numerical evidence, never a hardware certificate.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/scout_apps/control/spmpc_local_planner"
MODES = ("raw", "geometry", "planned", "planned_slosh")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command, log, timeout, environment):
    with log.open("w") as stream:
        try:
            return subprocess.run(list(map(str, command)), stdout=stream,
                                  stderr=subprocess.STDOUT, env=environment,
                                  timeout=timeout, check=False).returncode
        except subprocess.TimeoutExpired:
            return 124


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenarios", nargs="+", default=["straight", "corner", "double_corner"],
                        choices=["straight", "corner", "double_corner"])
    parser.add_argument("--actuator-scales", nargs="+", type=float, default=[1., 1.15])
    parser.add_argument("--curvature-weight", type=float, default=.05)
    parser.add_argument("--rti-iterations", type=int, default=5)
    parser.add_argument("--plan-timeout", type=float, default=600.)
    parser.add_argument("--suffix-reoptimize", action="store_true")
    parser.add_argument("--plans-dir", type=Path, help="reuse validated frozen plans from an earlier run")
    args = parser.parse_args()
    binary = args.native_dir.resolve() / "geometry_trial"
    if not binary.is_file():
        parser.error("build the real acados geometry_trial target first")
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    environment = os.environ.copy()
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        environment[key] = "1"
    report = {"evidence": "MODEL_IN_THE_LOOP", "hardware_verified": False,
              "liquid_state_source": "plant model state (ideal feedback)",
              "plant_mismatch": "actuator time constants scaled; no new liquid model",
              "binary_sha256": digest(binary), "trials": [], "plans": []}
    report["git_sha"] = subprocess.check_output(["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip()
    diff = subprocess.check_output(["git", "-C", ROOT, "diff", "HEAD"])
    (out / "source.diff").write_bytes(diff)
    report["source_diff_sha256"] = hashlib.sha256(diff).hexdigest()
    report["solver_library_sha256"] = {name: digest(PACKAGE / f"generated/acados/{name}/libacados_ocp_solver_{name}.so")
                                       for name in ("spmpc_b0", "spmpc_slosh")}
    for scenario in args.scenarios:
        task = out / f"{scenario}_task.json"
        shutil.copyfile(ROOT / "test/native/scenarios" / f"{scenario}.json", task)
        plan = out / f"{scenario}_plan.json"
        if args.plans_dir:
            shutil.copyfile(args.plans_dir / f"{scenario}_plan.json", plan)
            data = json.loads(plan.read_text())
            original = json.loads(task.read_text())
            if any(data["task"].get(key) != value for key, value in original.items()):
                raise ValueError(f"{scenario}: reused plan differs from the input task")
            command = [sys.executable, PACKAGE / "scripts/trajectory/generate_trajectory_plan.py", plan,
                       out / f"{scenario}_revalidation.json", "--validate-plan"]
        else:
            command = [sys.executable, PACKAGE / "scripts/trajectory/generate_trajectory_plan.py", task, plan]
        code = run(command, out / f"{scenario}_planning.log", args.plan_timeout, environment)
        plan_record = {"scenario": scenario, "task_sha256": digest(task), "exit_code": code}
        report["plans"].append(plan_record)
        if code:
            print(f"{scenario}: planning failed (exit {code}); see saved log", flush=True)
            continue
        data = json.loads(plan.read_text())
        plan_record.update(plan_id=data["plan_id"], plan_sha256=digest(plan), validation=data["validation"])
        # Both planned modes consume exactly the same full-task liquid plan;
        # only the lower liquid objective differs. This is not a 2x2 upper/lower ablation.
        for scale in args.actuator_scales:
            for mode in MODES:
                name = f"{scenario}_{mode}_tau{scale:g}"
                prefix = out / name
                contour = 1. if mode == "raw" else .02
                command = [binary, mode, prefix, plan, scale, args.curvature_weight,
                           contour, args.rti_iterations, 2.]
                print(f"running {name}", flush=True)
                code = run(command, out / f"{name}.log", 180., environment)
                trial = {"scenario": scenario, "mode": mode, "actuator_scale": scale,
                         "command": list(map(str, command)), "exit_code": code}
                # Names contain a decimal scale, so append rather than replace suffix.
                result = Path(str(prefix) + ".json")
                if result.exists():
                    trial["result"] = json.loads(result.read_text())
                predicted_checkpoint = Path(str(prefix) + "_checkpoint.json")
                if predicted_checkpoint.exists():
                    command = [sys.executable, PACKAGE / "scripts/trajectory/diagnose_trajectory_suffix.py",
                               plan, predicted_checkpoint, out / f"{name}_suffix.json"]
                    if not args.suffix_reoptimize:
                        command.append("--nominal-only")
                    trial["suffix_exit_code"] = run(command, out / f"{name}_suffix.log", args.plan_timeout, environment)
                report["trials"].append(trial)
                (out / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        # Preserve the complete predicted state at a nominal checkpoint.
        checkpoint = out / f"{scenario}_checkpoint.json"
        row = data["samples"][len(data["samples"]) // 3]
        checkpoint.write_text(json.dumps({"task_elapsed_sec": row["t"], "state": row["state"]}))
        plan_record["suffix_exit_code"] = run(
            [sys.executable, PACKAGE / "scripts/trajectory/diagnose_trajectory_suffix.py", plan, checkpoint,
             out / f"{scenario}_suffix.json", "--nominal-only"],
            out / f"{scenario}_suffix.log", 60., environment)
    report["all_trials_completed"] = (len(report["trials"]) == len(args.scenarios)*len(args.actuator_scales)*len(MODES)
                                       and all(t["exit_code"] == 0 for t in report["trials"]))
    (out / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(out / "report.json")
    return 0 if report["all_trials_completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
