#!/usr/bin/env python3
"""Fail-closed admission wrapper for the separately linked simulation node.

The gate is intentionally outside the real controller package.  It validates
the complete SIMULATION_MECHANISM_ONLY contract from the ROS parameter server
before replacing itself with ``spmpc_sim_local_planner_node``.  A normal
real-robot launch never invokes this file and no parameter added here is read
by the real controller source.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any, Dict, Mapping


TARGET_ID = "SMPCC_SIM_LOCAL_PLANNER_TARGET_R8"
GATE_ID = "SMPCC_SIM_LOCAL_PLANNER_GATE_R8"
NODE_NAME = "/sim_spmpc_local_planner"
ENVIRONMENT_OWNER_PARAM = "/smpcc_sim_environment/owner_package"
ENVIRONMENT_OWNER_PACKAGE = "spmpc_sim_local_planner"
URI_RE = re.compile(r"^http://127\.0\.0\.1:([0-9]+)$")
# `common_sim.yaml` spells 1 / 30 as 0.0333333333.  Accept that textual
# representation while still refusing a materially different controller step.
FLOAT_TOLERANCE = 1e-9

CONTAINERS: Dict[str, Dict[str, Any]] = {
    "C1": {
        "config_id": "SIM_ONLY_C1_D37_H58_UNVALIDATED",
        "container_radius": 0.0185,
    },
    "C2": {
        "config_id": "SIM_ONLY_C2_D95_H58_UNVALIDATED",
        "container_radius": 0.0475,
    },
}

_COMMON = {
    "slosh_constraint_enable": False,
    "primitive_mode": "linear",
    "w_contour": 1.0,
    "w_lag": 0.2,
    "w_progress": 0.2,
    "w_v": 1.0,
    "w_vs": 0.3,
    "w_accel": 0.0,
}

CONDITIONS: Dict[str, Dict[str, Any]] = {
    "SIM_B0_R1": {
        "engine_variant": "B0",
        "variant": dict(_COMMON, slosh_enable=False, smooth_priority_enable=False,
                        v_ref=0.20, w_control=0.1, w_smooth=0.1,
                        w_alpha=0.1, w_du_a=0.1, w_du_vs=0.1, w_slosh=0.0),
    },
    "SIM_Bsmooth_R1": {
        "engine_variant": "B_smooth",
        "variant": dict(_COMMON, slosh_enable=False, smooth_priority_enable=True,
                        v_ref=0.20, w_control=0.3, w_smooth=1.0,
                        w_alpha=1.0, w_du_a=1.0, w_du_vs=1.0, w_slosh=0.0),
    },
    # H0 is a development-only runtime smoke.  It deliberately has a
    # different public identity from the R7/R8 matrix baseline because its
    # historical P2 smoke uses v_ref=0.25 m/s rather than the frozen matrix
    # value 0.20 m/s.
    "SIM_H0_Bsmooth_R1": {
        "engine_variant": "B_smooth",
        "variant": dict(_COMMON, slosh_enable=False, smooth_priority_enable=True,
                        v_ref=0.25, w_control=0.3, w_smooth=1.0,
                        w_alpha=1.0, w_du_a=1.0, w_du_vs=1.0, w_slosh=0.0),
    },
    "SIM_SmoothMatch_R1": {
        "engine_variant": "B_smooth",
        "variant": dict(_COMMON, slosh_enable=False, smooth_priority_enable=True,
                        v_ref=0.18, w_control=0.3, w_smooth=1.0,
                        w_alpha=1.0, w_du_a=1.0, w_du_vs=1.0, w_slosh=0.0),
    },
    "SIM_Bslosh_R1": {
        "engine_variant": "B_slosh",
        "variant": dict(_COMMON, slosh_enable=True, smooth_priority_enable=False,
                        v_ref=0.20, w_control=0.3, w_smooth=1.0,
                        w_alpha=1.0, w_du_a=1.0, w_du_vs=1.0, w_slosh=5.0),
    },
}


class GateError(RuntimeError):
    pass


def _require(value: bool, message: str) -> None:
    if not value:
        raise GateError(message)


def _equal(actual: Any, expected: Any) -> bool:
    if isinstance(expected, float):
        return (isinstance(actual, (int, float)) and not isinstance(actual, bool)
                and math.isfinite(float(actual))
                and math.isclose(float(actual), expected, rel_tol=FLOAT_TOLERANCE,
                                 abs_tol=FLOAT_TOLERANCE))
    return actual == expected


def canonical_uri(uri: str) -> str:
    match = URI_RE.fullmatch(uri.strip())
    _require(match is not None, "ROS master must be http://127.0.0.1:<port>")
    port = int(match.group(1))
    _require(1024 <= port <= 65535, "ROS master loopback port must be 1024..65535")
    return f"http://127.0.0.1:{port}"


def required_parameter_paths(condition: str, container: str) -> Dict[str, Any]:
    _require(condition in CONDITIONS, "unknown simulation controller condition")
    _require(container in CONTAINERS, "unknown simulation container condition")
    condition_spec = CONDITIONS[condition]
    container_spec = CONTAINERS[container]
    engine_variant = str(condition_spec["engine_variant"])
    expected: Dict[str, Any] = {
        "sim_adapter/target_id": TARGET_ID,
        "sim_adapter/gate_id": GATE_ID,
        "sim_adapter/launch_marker": True,
        "sim_adapter/release_ack": True,
        "sim_adapter/condition_id": condition,
        "sim_adapter/container_condition": container,
        "sim_adapter/container_config_id": container_spec["config_id"],
        "planner_variant": engine_variant,
        "solver_backend": "continuous_mpcc_acados",
        "frames/use_tf_pose": False,
        "slosh_observer/source": "odom",
        "slosh_observer/fallback_policy": "odom",
        "imu_shadow/enable": False,
        "delay_phase/mode": "off",
        "control_frequency": 30.0,
        "dt": 1.0 / 30.0,
        "horizon_steps": 60,
        "slosh/container_radius": container_spec["container_radius"],
        "slosh/liquid_height": 0.058,
        "slosh/liquid_density": 1000.0,
        "slosh/damping_ratio": 0.05,
        "slosh/mode_index": 1,
        "slosh/slosh_height_ref": 0.005,
        "slosh/slosh_height_max": 0.001,
        "slosh/slosh_eta_dot_ratio": 0.3,
        "slosh/use_linear_model": True,
        "slosh/use_parabola_term": False,
    }
    for key, value in dict(condition_spec["variant"]).items():
        expected[f"variants/{engine_variant}/{key}"] = value
    return expected


def validate_snapshot(snapshot: Mapping[str, Any]) -> Dict[str, Any]:
    """Validate a flattened, testable pre-exec snapshot without ROS imports."""
    uri = canonical_uri(str(snapshot.get("ros_master_uri", "")))
    _require(snapshot.get("use_sim_time") is True, "/use_sim_time must be true")
    _require(
        snapshot.get("environment_owner_package") == ENVIRONMENT_OWNER_PACKAGE,
        f"{ENVIRONMENT_OWNER_PARAM} must be {ENVIRONMENT_OWNER_PACKAGE}",
    )
    condition = snapshot.get("condition_id")
    container = snapshot.get("container_condition")
    _require(isinstance(condition, str) and condition in CONDITIONS,
             "sim_adapter/condition_id is invalid")
    _require(isinstance(container, str) and container in CONTAINERS,
             "sim_adapter/container_condition is invalid")
    parameters = snapshot.get("parameters")
    _require(isinstance(parameters, Mapping), "parameters must be an object")
    expected = required_parameter_paths(condition, container)
    drift: Dict[str, Dict[str, Any]] = {}
    for path, value in expected.items():
        actual = parameters.get(path)
        if not _equal(actual, value):
            drift[path] = {"expected": value, "actual": actual}
    _require(not drift, "simulation controller contract drift: " + json.dumps(drift, sort_keys=True))
    payload = {
        "gate_id": GATE_ID,
        "target_id": TARGET_ID,
        "ros_master_uri": uri,
        "environment_owner_package": ENVIRONMENT_OWNER_PACKAGE,
        "condition_id": condition,
        "container_condition": container,
        "parameters": {path: parameters[path] for path in sorted(expected)},
    }
    return dict(payload, gate_hash=hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest())


def _node_name() -> str:
    for argument in sys.argv[1:]:
        if argument.startswith("__name:="):
            raw = argument.split(":=", 1)[1].strip("/")
            if raw:
                return "/" + raw
    return NODE_NAME


def _read_live_snapshot() -> Dict[str, Any]:
    try:
        import rosgraph  # type: ignore
    except ImportError as exc:
        raise GateError("rosgraph is required for the simulation controller gate") from exc
    node = _node_name()
    master = rosgraph.Master("/smpcc_sim_controller_gate")

    def get(path: str) -> Any:
        try:
            return master.getParam(node + "/" + path)
        except Exception as exc:
            raise GateError(f"missing or unreadable ROS parameter {node}/{path}: {exc}") from exc

    condition = get("sim_adapter/condition_id")
    container = get("sim_adapter/container_condition")
    _require(isinstance(condition, str), "condition ID must be text")
    _require(isinstance(container, str), "container condition must be text")
    expected = required_parameter_paths(condition, container)
    parameters = {path: get(path) for path in expected}
    try:
        use_sim_time = master.getParam("/use_sim_time")
    except Exception as exc:
        raise GateError("/use_sim_time is required") from exc
    try:
        environment_owner_package = master.getParam(ENVIRONMENT_OWNER_PARAM)
    except Exception as exc:
        raise GateError(f"{ENVIRONMENT_OWNER_PARAM} is required") from exc
    return {
        "ros_master_uri": os.environ.get("ROS_MASTER_URI", ""),
        "use_sim_time": use_sim_time,
        "environment_owner_package": environment_owner_package,
        "condition_id": condition,
        "container_condition": container,
        "parameters": parameters,
    }


def _write_receipt(receipt: Mapping[str, Any]) -> None:
    raw = os.environ.get("SMPCC_SIM_GATE_RECEIPT_PATH", "").strip()
    if not raw:
        return
    path = Path(raw)
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    try:
        descriptor = os.open(str(path), flags, 0o444)
    except FileExistsError as exc:
        raise GateError(f"gate receipt already exists: {path}") from exc
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(dict(receipt), stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def _bind_gate_hash_to_node(receipt: Mapping[str, Any]) -> None:
    """Bind the validated handoff token to the exact exec target namespace.

    The independently linked C++ node requires both this private parameter and
    the inherited environment token before it creates a controller object.
    Therefore a direct ``rosrun`` cannot silently bypass this Python gate.
    """

    token = receipt.get("gate_hash")
    _require(isinstance(token, str) and re.fullmatch(r"[0-9a-f]{64}", token),
             "validated gate receipt lacks a canonical SHA-256 token")
    try:
        import rosgraph  # type: ignore
    except ImportError as exc:
        raise GateError("rosgraph is required to bind the simulation gate token") from exc
    node = _node_name()
    path = node + "/sim_adapter/gate_hash"
    master = rosgraph.Master("/smpcc_sim_controller_gate")
    try:
        existing = master.getParam(path)
    except Exception:
        existing = None
    _require(existing in (None, token),
             "sim_adapter/gate_hash is already bound to a different gate token")
    try:
        master.setParam(path, token)
        readback = master.getParam(path)
    except Exception as exc:
        raise GateError(f"cannot bind sim_adapter/gate_hash before controller exec: {exc}") from exc
    _require(readback == token,
             "sim_adapter/gate_hash write/readback mismatch before controller exec")


def run_gate() -> int:
    receipt = validate_snapshot(_read_live_snapshot())
    _bind_gate_hash_to_node(receipt)
    _write_receipt(receipt)
    os.environ["SMPCC_SIM_CONTROLLER_GATE_HASH"] = str(receipt["gate_hash"])
    os.execvp(
        "rosrun",
        ["rosrun", "spmpc_sim_local_planner", "spmpc_sim_local_planner_node", *sys.argv[1:]],
    )
    return 127


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-json", type=Path,
                        help="offline test hook: validate a flattened gate snapshot")
    # roslaunch appends ROS remapping arguments such as ``__name:=...`` and
    # ``__log:=...``.  They are not gate CLI options and must remain in
    # ``sys.argv`` for the later exec'd simulation node.
    args, _unknown_ros_arguments = parser.parse_known_args()
    try:
        if args.validate_json is not None:
            with args.validate_json.open(encoding="utf-8") as stream:
                print(json.dumps(validate_snapshot(json.load(stream)), sort_keys=True))
            return 0
        return run_gate()
    except (GateError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[smpcc_sim_controller_gate] REFUSED: {exc}", file=sys.stderr)
        return 64


if __name__ == "__main__":
    raise SystemExit(main())
