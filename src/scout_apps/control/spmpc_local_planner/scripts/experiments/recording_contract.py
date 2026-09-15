#!/usr/bin/env python3
"""Freeze and verify the trajectory MPCC recording contract (stdlib + PyYAML only)."""
import argparse
import copy
import hashlib
import json
import math
import shutil
import subprocess
from pathlib import Path

SCHEMA = "trajectory_mpcc_recording_v2"

def merge(dst, src):
    for key, value in (src or {}).items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            merge(dst[key], value)
        else:
            dst[key] = copy.deepcopy(value)

def load(path):
    import yaml
    class UniqueLoader(yaml.SafeLoader):
        pass
    def construct_mapping(loader, node, deep=False):
        mapping = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if key in mapping:
                raise ValueError(f"duplicate YAML key: {key}")
            mapping[key] = loader.construct_object(value_node, deep=deep)
        return mapping
    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)
    with open(path, encoding="utf-8") as stream:
        return yaml.load(stream, Loader=UniqueLoader) or {}

def get(data, path):
    for key in path.split("."):
        data = data[key]
    return data

def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()

def git(repo, *args):
    try:
        return subprocess.check_output(["git", "-C", str(repo), *args], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""

def prepare(args):
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    if Path(args.name).name != args.name:
        raise SystemExit("recording name must be a basename")
    manifest_path = output / f"{args.name}_manifest.json"
    if manifest_path.exists():
        raise SystemExit("recording manifest already exists; choose a new NAME")
    effective = {}
    package=Path(__file__).resolve().parents[2]
    # Exact trajectory_mpcc.launch order, including calibrated platform/cup
    # files and all variant weights. Extra task is a test/CLI-only overlay.
    ordered=[("common",package/"config/planner/common.yaml"),
             ("variants",package/"config/planner/variants.yaml"),
             ("platform",package/"config/platforms/scout_mini.yaml"),
             ("container",package/"config/containers/tube_default.yaml"),
             ("profile",args.profile_config),("task",args.task_config)]
    for name,path in (("task_extra",args.task_config_extra),("task_overlay",args.task_overlay),
                      ("region",args.region),("planner_overlay",args.planner_overlay)):
        if path: ordered.append((name,path))
    artifacts = {}
    for label, path in ordered:
        if not path:
            continue
        path = str(path)
        if not Path(path).is_file():
            raise SystemExit(f"missing artifact: {path}")
        merge(effective, load(path))
        target = output / f"{args.name}_{label}_{Path(path).name}"
        shutil.copyfile(path, target)
        artifacts[label] = {"source": path, "copy": str(target), "sha256": sha256(path)}
    effective_plan = effective.get("planning", {}).get("reference", {}).get("plan_file", "")
    plan_path = args.plan_file or effective_plan
    if plan_path:
        plan = Path(plan_path)
        if not plan.is_file(): raise SystemExit(f"missing artifact: {plan}")
        effective.setdefault("planning", {}).setdefault("reference", {})["plan_file"] = str(plan)
        target = output / f"{args.name}_plan_{plan.name}"
        shutil.copyfile(plan, target)
        artifacts["plan"] = {"source": str(plan), "copy": str(target), "sha256": sha256(plan)}
        try:
            plan_data = json.loads(plan.read_text(encoding="utf-8"))
            plan_deadline = float(plan_data["deadline"])
            effective_deadline = float(get(effective, "planning.task_deadline_sec"))
            if not math.isfinite(plan_deadline) or abs(plan_deadline - effective_deadline) > 1e-6:
                raise SystemExit("plan deadline conflicts with effective planning.task_deadline_sec")
            plan_region = plan_data.get("region", {})
            effective_region = effective.get("planning", {}).get("region", {})
            for key in ("id", "frame_id", "footprint_radius", "margin", "cells"):
                if key in plan_region and key in effective_region and plan_region[key] != effective_region[key]:
                    raise SystemExit(f"plan region.{key} conflicts with effective planning/region/{key}")
        except SystemExit:
            raise
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            raise SystemExit("plan must declare a numeric deadline and valid JSON")
    identity = effective.get("planning", {}).get("experiment_profile_id")
    if identity != args.profile:
        raise SystemExit("effective profile identity differs from the requested recording profile")
    mode = effective.get("planning", {}).get("reference", {}).get("mode", "cruise")
    if mode not in ("cruise", "progress", "fixed_time"):
        raise SystemExit("unknown trajectory reference mode")
    if args.profile == "raw_mpcc" and (mode != "cruise" or plan_path):
        raise SystemExit("raw MPCC recording cannot consume a trajectory plan")
    if mode != "cruise" and not plan_path:
        raise SystemExit("non-cruise reference requires a trajectory plan")
    region = effective.get("planning", {}).get("region", {})
    if not region.get("enabled") or not region.get("cells"):
        raise SystemExit("effective planning.region must be enabled and contain cells")
    try:
        deadline = float(get(effective, "planning.task_deadline_sec"))
    except (KeyError, TypeError, ValueError):
        deadline = 0.0
    if not math.isfinite(deadline) or deadline <= 0: raise SystemExit("effective planning.task_deadline_sec must be finite and > 0")
    variant=args.planner_variant or ("B_slosh" if args.profile=="planned_slosh" else "B0")
    effective["planner_variant"]=variant
    flag = str(args.publish_cmd_vel).lower()
    if flag not in ("1", "0", "true", "false"):
        raise SystemExit("publish_cmd_vel must be true/false or 1/0")
    effective["publish_cmd_vel"] = flag in ("1", "true")
    manifest = {
        "schema": SCHEMA, "profile": args.profile,
        "planner_variant": variant,
        "recorded_launch_args": args.launch_args,
        "recorded_launch_args_status": "declared; launch is performed separately",
        "effective_config": effective, "artifacts": artifacts,
        "git": {"sha": git(args.repo, "rev-parse", "HEAD"),
                "status": git(args.repo, "status", "--short"),
                "diff": git(args.repo, "diff", "HEAD")},
    }
    manifest_path = output / f"{args.name}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    print(manifest_path)

def verify_live(args):
    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    if args.live_file:
        live = load(args.live_file)
    else:
        try:
            raw = subprocess.check_output(["rosparam", "get", "/spmpc_local_planner"], text=True, stderr=subprocess.STDOUT, timeout=5)
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            raise SystemExit(f"cannot read live ROS parameters: {exc}")
        import yaml
        live = yaml.safe_load(raw) or {}
    expected = manifest["effective_config"]
    def leaves(tree,prefix=""):
        for key,value in tree.items():
            path=f"{prefix}.{key}" if prefix else key
            if isinstance(value,dict): yield from leaves(value,path)
            else: yield path
    keys=list(leaves(expected))
    mismatches = []
    for key in keys:
        try: want, got = get(expected, key), get(live, key)
        except KeyError: mismatches.append(f"missing {key}"); continue
        if isinstance(want, float) or isinstance(got, float):
            try: equal = abs(float(want) - float(got)) <= 1e-9
            except (TypeError, ValueError): equal = False
        else: equal = want == got
        if not equal: mismatches.append(f"{key}: expected {want!r}, live {got!r}")
    if mismatches: raise SystemExit("live ROS parameter mismatch:\n" + "\n".join(mismatches))
    manifest["live_private_parameters"]=live
    manifest["live_verification"]="MATCHED_BEFORE_RECORDING"
    Path(args.manifest).write_text(json.dumps(manifest,indent=2,sort_keys=True,allow_nan=False)+"\n",encoding="utf-8")
    print("live ROS parameter contract PASS")

def parser():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="command", required=True)
    a = sub.add_parser("prepare")
    for name in ("profile_config", "task_config", "task_config_extra", "task_overlay", "region", "planner_overlay", "plan_file"):
        a.add_argument("--" + name.replace("_", "-"), default="")
    a.add_argument("--output-dir", required=True); a.add_argument("--name", required=True)
    a.add_argument("--profile", required=True); a.add_argument("--planner-variant", default="")
    a.add_argument("--publish-cmd-vel", default="true")
    a.add_argument("--launch-args", default=""); a.add_argument("--repo", required=True)
    a.set_defaults(func=prepare)
    a = sub.add_parser("verify-live"); a.add_argument("--manifest", required=True); a.add_argument("--live-file", default=""); a.set_defaults(func=verify_live)
    return p

if __name__ == "__main__":
    parsed = parser().parse_args()
    parsed.func(parsed)
