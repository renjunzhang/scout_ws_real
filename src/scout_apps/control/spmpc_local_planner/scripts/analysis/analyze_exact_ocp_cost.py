#!/usr/bin/env python3
"""Freeze and evaluate C03 OCP costs offline, without solving or ROS publication.

Use the archived generated C functions, including their compiled normalization
and anti-creep terms. Weight isolation evaluates ONE fixed recorded trajectory;
it is not a weight sweep or an experiment in control sensitivity.
"""

import argparse
import ast
import csv
import ctypes as ct
import gzip
import json
import math
from pathlib import Path
import re
import shutil
import sys
import time

import numpy as np

from analyze_internal_slosh_pair import _motion_window, _read_env, scalar_message
from freeze_internal_slosh_evaluation import ROOT, PLANNER, chain_identity, digest_json, sha256


SCHEMA = 1
TOPICS = {
    "/spmpc/debug/control_cycle_audit": "audit",
    "/spmpc/debug/pre_solve_snapshot": "snapshot",
    "/spmpc/debug/predicted_horizon": "horizon",
}
GROUPS = {
    "J_contour": ("w_contour",), "J_lag": ("w_lag",),
    "J_progress": ("w_progress",),
    # w_v also multiplies both low-speed deficits in the generated expression.
    "J_v_with_anticreep": ("w_v",), "J_vs": ("w_vs",),
    "J_control": ("w_a", "w_omega", "w_alpha"),
    "J_smooth": ("w_du_a", "w_du_vs"),
    "J_slosh_eta": ("w_slosh_eta",),
    "J_slosh_eta_dot": ("w_slosh_eta_dot",),
}
STATE_FIELDS = {
    0: "x", 1: "y", 2: "yaw", 3: "v", 4: "s", 5: "omega",
    6: "v_cmd", 7: "omega_cmd", 23: "a_cmd_memory",
    24: "eta_x", 25: "eta_x_dot", 26: "eta_y", 27: "eta_y_dot",
}
SUFFIXES = ("_0", "", "_e")
ATOL, RTOL = 1e-11, 1e-10


def require(condition, message):
    if not condition:
        raise ValueError(message)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                     allow_nan=False) + "\n", encoding="utf-8")


def parameter_layout(source):
    """Read literal ABI names without importing/regenerating the model."""
    wanted = ("PARAM_NAMES", "SLOSH_EXTRA_NAMES", "SLOSH_HARD_EXTRA_NAMES")
    found = {}
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in wanted:
                    found[target.id] = ast.literal_eval(node.value)
    require(set(found) == set(wanted), "unsupported model parameter layout")
    return found["PARAM_NAMES"], sum((found[k] for k in wanted), [])


def freeze_bundle(output, root=ROOT):
    """Archive existing binaries and metadata, never generate a solver."""
    output, root = Path(output), Path(root)
    require(not output.exists(), "preserve existing bundle: " + str(output))
    planner = root / PLANNER
    chain = chain_identity(root)
    source = planner / "scripts/acados/spmpc_acados_model.py"
    layouts = parameter_layout(source.read_text())
    output.mkdir(parents=True)
    files, models = {}, {}

    def copy(src, relative):
        dest = output / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(src), str(dest))
        files[relative] = sha256(dest)
        return relative

    for name, nx, names in zip(("spmpc_b0", "spmpc_slosh"), (24, 28), layouts):
        folder = planner / "generated/acados" / name
        libname = "libacados_ocp_solver_" + name + ".so"
        library = copy(folder / libname, name + "/" + libname)
        json_name = "acados_ocp_" + name + ".json"
        config = json.loads((folder / json_name).read_text())
        copy(folder / json_name, name + "/" + json_name)
        solver_source = (folder / ("acados_solver_" + name + ".c")).read_text()
        copy(folder / ("acados_solver_" + name + ".c"), name + "/solver.c")
        dims, options = config["dims"], config["solver_options"]
        require((dims["nx"], dims["nu"], dims["np"], dims["N"]) ==
                (nx, 3, len(names), 60), "unsupported generated dimensions")
        require(all(config["cost"].get(k) == "EXTERNAL" for k in
                    ("cost_type", "cost_type_0", "cost_type_e")), "unsupported cost type")
        require(not any(v for k, v in dims.items() if k.startswith("ns")),
                "slack costs require a separate reconstruction contract")
        scaling = np.asarray(options["cost_scaling"], dtype=float)
        require(scaling.shape == (61,) and np.isfinite(scaling).all() and
                np.all(scaling > 0), "invalid cost scaling")
        declared = {int(k): float(v) for k, v in re.findall(
            r"cost_scaling\[(\d+)\]\s*=\s*([0-9.eE+-]+);", solver_source)}
        require(len(declared) == 61 and np.array_equal(
            scaling, [declared[k] for k in range(61)]), "JSON/C scaling mismatch")
        cost_sources = {}
        for suffix in SUFFIXES:
            filename = name + "_cost_ext_cost" + suffix + "_fun.c"
            src = folder / (name + "_cost") / filename
            text = src.read_text()
            require(re.search(r"#define casadi_int int\s", text) is not None and
                    "#define casadi_real double" in text, "unsupported C ABI")
            used = sorted(set(map(int, re.findall(r"arg\[0\]\[(\d+)\]", text))))
            require(used and set(used) <= set(STATE_FIELDS),
                    "cost consumes unrecorded state coordinates: " + filename)
            cost_sources[suffix] = {"file": copy(src, name + "/" + filename),
                                    "state_indices": used}
        expected_lib_sha = chain["files"][str(PLANNER / "generated/acados" / name / libname)]
        require(files[library] == expected_lib_sha, "library changed during freeze")
        models[name] = {"nx": nx, "np": len(names), "N": 60, "library": library,
                        "parameter_names": names, "cost_scaling": scaling.tolist(),
                        "time_steps": options["time_steps"], "functions": cost_sources}
        integration_file = folder / "integration_metadata.json"
        if integration_file.exists():
            integration = json.loads(integration_file.read_text())
            require(integration["solver_library_sha256"] == files[library],
                    "integration metadata does not match solver library")
            require(integration["model_source_sha256"] == sha256(source) and integration["model"] == name,
                    "integration metadata does not match current model source")
            models[name]["integration"] = integration
            copy(integration_file, name + "/integration_metadata.json")
    copy(source, "sources/spmpc_acados_model.py")
    copy(planner / "scripts/acados/spmpc_acados_cost.py", "sources/spmpc_acados_cost.py")
    for filename in (Path(__file__).name, "analyze_internal_slosh_pair.py",
                     "freeze_internal_slosh_evaluation.py"):
        copy(Path(__file__).with_name(filename), "tools/" + filename)
    manifest = {"schema_version": SCHEMA, "created_at_epoch_sec": time.time(),
                "evaluation_chain_sha256": chain["sha256"],
                "evaluation_chain_files": chain["files"], "files": files, "models": models,
                "scope": "generated EXTERNAL objective on recorded iterates; no solve/slack costs"}
    write_json(output / "manifest.json", manifest)
    return manifest


def load_bundle(path):
    path = Path(path)
    manifest = json.loads((path / "manifest.json").read_text())
    require(manifest.get("schema_version") == SCHEMA, "unsupported bundle schema")
    require(manifest.get("files") and manifest.get("models"), "empty bundle")
    require(digest_json(manifest["evaluation_chain_files"]) == manifest["evaluation_chain_sha256"],
            "bundle evaluation-chain digest mismatch")
    for relative, digest in manifest["files"].items():
        file = (path / relative).resolve()
        require(path.resolve() in file.parents, "bundle path escapes directory")
        require(sha256(file) == digest, "bundle hash mismatch: " + relative)
    # The recording fingerprint binds the actual shared libraries, not just
    # the human-readable manifest's own hashes.
    for name, model in manifest["models"].items():
        lib = "libacados_ocp_solver_" + name + ".so"
        relative = str(PLANNER / "generated/acados" / name / lib)
        require(manifest["files"][model["library"]] ==
                manifest["evaluation_chain_files"][relative], "bundle/library chain mismatch")
        config = json.loads((path / name / ("acados_ocp_" + name + ".json")).read_text())
        require(model["cost_scaling"] == config["solver_options"]["cost_scaling"] and
                model["time_steps"] == config["solver_options"]["time_steps"],
                "bundle scaling differs from archived config")
        for function in model["functions"].values():
            used = sorted(set(map(int, re.findall(r"arg\[0\]\[(\d+)\]",
                                                 (path / function["file"]).read_text()))))
            require(used == function["state_indices"] and set(used) <= set(STATE_FIELDS),
                    "bundle cost-state dependency mismatch")
    return manifest


class CostFunction:
    """The generated ABI is double/int32, unlike CasADi external's int64 ABI."""

    def __init__(self, library, name, nx, np_, terminal):
        self.ptr = ct.POINTER(ct.c_double)
        self.fn = getattr(library, name)
        self.fn.restype = ct.c_int
        self.fn.argtypes = [ct.POINTER(self.ptr), ct.POINTER(self.ptr),
                           ct.POINTER(ct.c_int), self.ptr, ct.c_int]
        sparse = getattr(library, name + "_sparsity_in")
        sparse.restype, sparse.argtypes = ct.POINTER(ct.c_int), [ct.c_int]
        require([sparse(i)[0] for i in range(4)] == [nx, 0 if terminal else 3, 0, np_],
                "library input dimensions mismatch: " + name)
        work = getattr(library, name + "_work")
        work.restype, work.argtypes = ct.c_int, [ct.POINTER(ct.c_int)] * 4
        sizes = [ct.c_int() for _ in range(4)]
        require(work(*[ct.byref(s) for s in sizes]) == 0, "cost work() failed")
        na, nr, ni, nw = [s.value for s in sizes]
        require((na, nr) == (4, 1) and 0 <= ni < 1000000 and 0 <= nw < 1000000,
                "unsupported cost workspace")
        self.iw = (ct.c_int * max(1, ni))()
        self.work = (ct.c_double * max(1, nw))()
        self.terminal = terminal
        self.empty = np.zeros(0, dtype=np.float64)

    def __call__(self, x, u, p):
        arrays = [np.ascontiguousarray(a, dtype=np.float64) for a in
                  (x, self.empty if self.terminal else u, self.empty, p)]
        args = (self.ptr * 4)(*[a.ctypes.data_as(self.ptr) for a in arrays])
        out = np.zeros(1)
        res = (self.ptr * 1)(out.ctypes.data_as(self.ptr))
        require(self.fn(args, res, self.iw, self.work, 0) == 0, "cost evaluation failed")
        require(np.isfinite(out[0]), "non-finite generated cost")
        return float(out[0])


class CostEvaluator:
    def __init__(self, bundle, manifest, model):
        self.meta = manifest["models"][model]
        self.library = ct.CDLL(str((Path(bundle) / self.meta["library"]).resolve()))
        self.functions = {s: CostFunction(self.library, model + "_cost_ext_cost" + s + "_fun",
                                         self.meta["nx"], self.meta["np"], s == "_e")
                          for s in SUFFIXES}
        self.index = {name: i for i, name in enumerate(self.meta["parameter_names"])}
        self.groups = {key: [self.index[w] for w in weights if w in self.index]
                       for key, weights in GROUPS.items()}
        self.weight_indices = [i for i, name in enumerate(self.meta["parameter_names"])
                               if name.startswith("w_")]
        require(set(sum(self.groups.values(), [])) == set(self.weight_indices),
                "unaccounted cost weight")

    def stage(self, x, u, p, k):
        require(np.asarray(p).shape == (self.meta["np"],) and np.isfinite(p).all(),
                "invalid stage parameter vector")
        suffix = "_e" if k == self.meta["N"] else "_0" if k == 0 else ""
        fn = self.functions[suffix]
        total = fn(x, u, p)
        zero = np.asarray(p, dtype=float).copy()
        zero[self.weight_indices] = 0
        require(abs(fn(x, u, zero)) <= ATOL, "unattributed constant/non-weight cost")
        parts = {}
        for key, indices in self.groups.items():
            masked = zero.copy()
            masked[indices] = p[indices]
            parts[key] = fn(x, u, masked) if indices else 0.0
        error = abs(sum(parts.values()) - total)
        require(error <= ATOL + RTOL * abs(total), "cost parts do not sum to original function")
        scale = self.meta["cost_scaling"][k]
        return {key: value * scale for key, value in parts.items()}, total * scale, error


def vector(row, key, size):
    value = np.asarray(row.get(key, []), dtype=float)
    require(value.shape == (size,) and np.isfinite(value).all(), "invalid/missing " + key)
    return value


def evaluate_cycle(snapshot, horizon, evaluator):
    meta, n = evaluator.meta, evaluator.meta["N"]
    require(snapshot.get("valid") and horizon.get("valid"), "invalid snapshot/horizon")
    require(snapshot.get("schema_version") == 5 and horizon.get("schema_version") == 5,
            "only explicit-actuator schema 5 is supported")
    require(snapshot["cycle_id"] == horizon["cycle_id"], "cycle_id mismatch")
    require(snapshot.get("control_semantics") == horizon.get("control_semantics") ==
            "a_cmd_alpha_cmd", "unsupported control semantics")
    require(snapshot.get("backend") == horizon.get("backend") ==
            "continuous_mpcc_acados_explicit_actuator", "backend mismatch")
    require(snapshot.get("variant") == horizon.get("variant") == "B_slosh" and
            snapshot.get("solver_status") == horizon.get("solver_status") == "B_slosh_ACADOS_OK",
            "variant/solver status mismatch")
    require((snapshot.get("horizon_steps"), horizon.get("horizon_steps"),
             snapshot.get("state_width"), snapshot.get("control_width"),
             snapshot.get("parameter_width")) == (n, n, meta["nx"], 3, meta["np"]),
            "snapshot/horizon dimensions mismatch")
    require(snapshot.get("parameter_names") == meta["parameter_names"], "parameter ABI mismatch")
    require(snapshot.get("slosh_enabled") == horizon.get("slosh_enabled") == (meta["nx"] == 28),
            "slosh model mismatch")
    for row in (snapshot, horizon):
        require(math.isclose(float(row["dt"]), meta["time_steps"][0], rel_tol=1e-8),
                "dt differs from archived scaling contract")
    require(abs(float(snapshot["solver_input_epoch"]) - float(horizon["solver_input_epoch"])) < 1e-6,
            "solver epoch mismatch")
    p = vector(snapshot, "stage_parameters", (n + 1) * meta["np"]).reshape(n + 1, meta["np"])
    used = set().union(*(set(f["state_indices"]) for f in meta["functions"].values()))
    # FIFO queues are not published at every horizon node, and this version's
    # cost does not consume them. Leave them NaN, never invent zero history.
    x = np.full((n + 1, meta["nx"]), np.nan)
    for i in used:
        x[:, i] = vector(horizon, STATE_FIELDS[i], n + 1)
    u = np.column_stack([vector(horizon, name, n) for name in ("a", "alpha_or_omega", "v_s")])
    stage_parts = {key: 0.0 for key in GROUPS}
    terminal_parts = dict(stage_parts)
    total, max_error = 0.0, 0.0
    for k in range(n + 1):
        parts, value, error = evaluator.stage(x[k], u[k] if k < n else np.zeros(0), p[k], k)
        destination = terminal_parts if k == n else stage_parts
        for key, amount in parts.items():
            destination[key] += amount
        total += value
        max_error = max(max_error, error)
    combined = {key: stage_parts[key] + terminal_parts[key] for key in GROUPS}
    require(abs(sum(combined.values()) - total) <= ATOL + RTOL * abs(total),
            "horizon sum mismatch")
    absolute = sum(abs(value) for value in combined.values())
    return {"total": total, "stage_total": sum(stage_parts.values()),
            "terminal_total": sum(terminal_parts.values()), "parts": combined,
            "stage_parts": stage_parts, "terminal_parts": terminal_parts,
            "abs_parts_sum": absolute, "max_unscaled_stage_sum_error": max_error,
            "weight_ranges": {name: [float(np.min(p[:, i])), float(np.max(p[:, i]))]
                              for name, i in evaluator.index.items() if name.startswith("w_")},
            "slosh_abs_percent": 100 * (abs(combined["J_slosh_eta"]) +
                abs(combined["J_slosh_eta_dot"])) / absolute if absolute > ATOL else None}


def load_inputs(bag, cache):
    bag, cache = Path(bag), Path(cache)
    identity = {"schema_version": SCHEMA, "bag": str(bag.resolve()),
                "size": bag.stat().st_size, "mtime_ns": bag.stat().st_mtime_ns,
                "topics": sorted(TOPICS)}
    if cache.exists():
        with gzip.open(str(cache), "rt", encoding="utf-8") as stream:
            saved = json.load(stream)
        if saved.get("identity") == identity:
            return saved["topics"], True
    import rosbag  # Offline file reader only; no rospy/ROS master.
    topics = {key: [] for key in TOPICS.values()}
    with rosbag.Bag(str(bag), "r") as stream:
        missing = set(TOPICS) - set(stream.get_type_and_topic_info().topics)
        require(not missing, "missing recorded topics: " + str(sorted(missing)))
        for topic, message, _ in stream.read_messages(topics=list(TOPICS)):
            topics[TOPICS[topic]].append(scalar_message(message))
    with gzip.open(str(cache), "wt", encoding="utf-8") as stream:
        json.dump({"identity": identity, "topics": topics}, stream)
    return topics, False


def unique_cycles(rows):
    result, duplicates = {}, set()
    for row in rows:
        cycle = int(row["cycle_id"])
        if cycle in result:
            duplicates.add(cycle)
        result[cycle] = row
    return result, duplicates


def analyze_topics(topics, evaluators, condition, parameter_expectations=None):
    audits, adup = unique_cycles(topics["audit"])
    snapshots, sdup = unique_cycles(topics["snapshot"])
    horizons, hdup = unique_cycles(topics["horizon"])
    start, end = _motion_window(topics["audit"])
    model = "spmpc_slosh" if condition == "full" else "spmpc_b0"
    rows, rejected, no_solve, outside = [], [], 0, 0
    for cycle, audit in sorted(audits.items()):
        stamp, timestamp_source = 0.0, ""
        # A successful OCP can be intercepted before command publication.
        # Preserve its objective, with explicit solve timing and intervention
        # flags; do not silently discard it or pretend it was executed.
        for source in ("command_publish_stamp", "solve_end_stamp", "horizon_available_stamp"):
            candidate = float(audit.get(source, 0))
            if math.isfinite(candidate) and candidate > 0:
                stamp, timestamp_source = candidate, source
                break
        if stamp <= 0 and audit.get("solve_attempted"):
            rejected.append({"cycle_id": cycle, "reason": "missing event timestamp; task membership unknown"})
            continue
        if not start <= stamp <= end:
            outside += 1
            continue
        if not audit.get("solve_attempted"):
            no_solve += 1
            continue
        try:
            require(cycle not in adup | sdup | hdup, "duplicate cycle_id")
            require(audit.get("solve_success"), "solve failed")
            require(cycle in snapshots and cycle in horizons, "missing snapshot/horizon")
            require(audit.get("variant") == horizons[cycle].get("variant") and
                    audit.get("solver_status") == horizons[cycle].get("solver_status"),
                    "audit variant/solver status mismatch")
            require(abs(float(audit["solver_input_epoch"]) -
                        float(snapshots[cycle]["solver_input_epoch"])) < 1e-6,
                    "audit solver epoch mismatch")
            value = evaluate_cycle(snapshots[cycle], horizons[cycle], evaluators[model])
            if parameter_expectations:
                for key in ("w_v", "w_contour", "w_lag"):
                    require(all(math.isclose(v, parameter_expectations[key], rel_tol=1e-9, abs_tol=1e-12)
                                for v in value["weight_ranges"][key]),
                            "recorded OCP parameter differs from prereg: " + key)
                for state in (snapshots[cycle], horizons[cycle]):
                    require(state.get("jerk_limit_enable") and math.isclose(
                        float(state["jerk_max"]), parameter_expectations["jerk_max"], rel_tol=1e-9),
                        "recorded jerk differs from prereg")
            value.update(cycle_id=cycle, event_stamp=stamp, timestamp_source=timestamp_source,
                         command_publish_stamp=float(audit.get("command_publish_stamp", 0)),
                         task_time_sec=stamp - start,
                         command_was_published=bool(audit.get("command_was_published")),
                         command_accepted=bool(audit.get("command_accepted")),
                         command_contract_violation=bool(audit.get("command_contract_violation")),
                         terminal_controller_intervened=bool(audit.get("terminal_controller_intervened")),
                         safety_gate_intervened=bool(audit.get("safety_gate_intervened")))
            rows.append(value)
        except (ValueError, KeyError, TypeError) as exc:
            rejected.append({"cycle_id": cycle, "reason": str(exc)})
    for cycle, horizon in horizons.items():
        stamp = float(horizon.get("solve_end_stamp", 0))
        if horizon.get("valid") and start <= stamp <= end and cycle not in audits:
            rejected.append({"cycle_id": cycle, "reason": "horizon without audit"})
    statistics = {}
    for key in GROUPS:
        values = np.asarray([row["parts"][key] for row in rows])
        if values.size:
            statistics[key] = {"mean": float(np.mean(values)), "median": float(np.median(values)),
                               "p95": float(np.percentile(values, 95)),
                               "max_abs": float(np.max(np.abs(values)))}
    return {"status": "PASS" if rows and not rejected else "FAIL", "condition": condition,
            "task_window": [start, end], "evaluated_cycles": len(rows),
            "no_solve_cycles_excluded": no_solve, "rejected_cycles": rejected,
            "outside_task_cycles_excluded": outside,
            "statistics": statistics, "cycles": rows,
            "scope": "objective on returned MPC iterates; not a time integral of executed motion",
            "notes": ["PASS checks numerical reconstruction only, not recording quality or efficacy",
                      "J_v_with_anticreep includes v tracking and both low-speed deficits",
                      "terminal is the OCP endpoint, not the terminal stopping controller",
                      "percent denominator is the sum of absolute component costs",
                      "weight isolation does not measure control sensitivity"]}


def save_outputs(output, report, plot=True):
    write_json(output / "report.json", report)
    rows = report.get("cycles", [])
    if not rows:
        return
    columns = ["cycle_id", "task_time_sec", "total", "stage_total", "terminal_total",
               "slosh_abs_percent", "max_unscaled_stage_sum_error", "command_accepted",
               "timestamp_source", "command_was_published", "command_contract_violation",
               "terminal_controller_intervened", "safety_gate_intervened"]
    for section in ("parts", "stage_parts", "terminal_parts"):
        columns += [section + "." + key for key in GROUPS]
    with (output / "cycles.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in columns if key in row}
            flat.update({section + "." + key: row[section][key]
                         for section in ("parts", "stage_parts", "terminal_parts") for key in GROUPS})
            writer.writerow(flat)
    if plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        t = [row["task_time_sec"] for row in rows]
        for key in GROUPS:
            axes[0].plot(t, [row["parts"][key] for row in rows], label=key, linewidth=0.8)
        axes[0].set_yscale("symlog", linthresh=1e-5)
        axes[0].set_ylabel("OCP component (signed)")
        axes[0].legend(fontsize=7, ncol=3)
        axes[1].plot(t, [row["stage_total"] for row in rows], label="running stages")
        axes[1].plot(t, [row["terminal_total"] for row in rows], label="OCP terminal")
        axes[1].set_xlabel("Time from first published motion command [s]")
        axes[1].set_ylabel("OCP objective")
        axes[1].legend()
        figure.suptitle("Generated cost on recorded iterates; " + report["status"])
        figure.tight_layout()
        figure.savefig(str(output / "cost_components.png"), dpi=140)
        plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    freeze = sub.add_parser("freeze", help="archive current artifacts before recording; no ROS")
    freeze.add_argument("--output-dir", type=Path, required=True)
    check = sub.add_parser("check", help="verify archive and current recording-chain identity")
    check.add_argument("--bundle", type=Path, required=True)
    analyze = sub.add_parser("analyze", help="read three topics of one new bag; no solve or publication")
    analyze.add_argument("--bag", type=Path, required=True)
    analyze.add_argument("--bundle", type=Path, required=True)
    analyze.add_argument("--output-dir", type=Path, required=True)
    analyze.add_argument("--cache", type=Path)
    analyze.add_argument("--no-plot", action="store_true")
    args = parser.parse_args(argv)
    created_output = False
    try:
        if args.action == "freeze":
            manifest = freeze_bundle(args.output_dir)
            print("Archived cost bundle: " + str(args.output_dir))
            print("Evaluation chain: " + manifest["evaluation_chain_sha256"])
            return 0
        manifest = load_bundle(args.bundle)
        if args.action == "check":
            require(chain_identity()["sha256"] == manifest["evaluation_chain_sha256"],
                    "current evaluation chain differs from bundle")
            print("Cost bundle/current chain PASS; no motion started")
            return 0
        require(not args.output_dir.exists(), "preserve existing analysis: " + str(args.output_dir))
        args.output_dir.mkdir(parents=True)
        created_output = True
        prereg_path = args.bag.with_name(args.bag.stem + "_runtime_smoke_prereg.env")
        prereg = _read_env(prereg_path)
        require(prereg.get("protocol") in ("SMPCC_C03_INTERNAL_SLOSH_DEV_V2",
                                           "SMPCC_C03_INTERNAL_SLOSH_DEV_V3"),
                "not a C03 V2/V3 bag")
        require(prereg.get("condition") in ("full", "smooth"), "invalid Full/Smooth condition")
        require(prereg.get("evaluation_chain_sha256") == manifest["evaluation_chain_sha256"],
                "recorded chain differs from archived artifacts")
        require(bool(prereg.get("git_revision")), "missing recording revision")
        parameter_expectations = None
        if prereg["protocol"] == "SMPCC_C03_INTERNAL_SLOSH_DEV_V3":
            parameter_expectations = {key: float(prereg[key])
                                      for key in ("w_v", "w_contour", "w_lag", "jerk_max")}
            require(all(math.isfinite(parameter_expectations[k]) and 0 < parameter_expectations[k] <= 20
                        for k in ("w_v", "w_contour", "w_lag")) and
                    parameter_expectations["jerk_max"] in (0.6, 1.0, 1.2),
                    "invalid V3 parameter prereg")
            require(prereg.get("evaluation_primary_monitor") == "imu", "V3 primary must be imu")
        require(manifest["created_at_epoch_sec"] <= args.bag.stat().st_mtime,
                "bundle was created after recording; require pre-record archive")
        cache = args.cache or args.bag.with_name(args.bag.stem + "_exact_cost_cache.json.gz")
        topics, hit = load_inputs(args.bag, cache)
        evaluators = {name: CostEvaluator(args.bundle, manifest, name) for name in manifest["models"]}
        report = analyze_topics(topics, evaluators, prereg["condition"], parameter_expectations)
        report.update(bag=str(args.bag.resolve()), prereg=prereg,
                      prereg_sha256=sha256(prereg_path), bundle=str(args.bundle.resolve()),
                      bundle_manifest_sha256=sha256(args.bundle / "manifest.json"),
                      analysis_tool_sha256=sha256(Path(__file__)), cache=str(cache), cache_hit=hit)
        require(manifest["created_at_epoch_sec"] < report["task_window"][0],
                "bundle must precede task start")
        save_outputs(args.output_dir, report, not args.no_plot)
        print(report["status"] + ": " + str(args.output_dir / "report.json"))
        return 0 if report["status"] == "PASS" else 1
    except (ValueError, KeyError, OSError, TypeError, ImportError) as exc:
        # Keep a reviewable failure, but never overwrite an existing report.
        out = getattr(args, "output_dir", None)
        if created_output and out and not (out / "report.json").exists():
            write_json(out / "report.json", {"status": "ERROR", "error": str(exc),
                                             "bag": str(args.bag), "bundle": str(args.bundle)})
        print("exact-cost ERROR: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
