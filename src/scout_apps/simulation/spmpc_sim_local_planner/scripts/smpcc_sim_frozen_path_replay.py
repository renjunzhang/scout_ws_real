#!/usr/bin/env python3
"""Fail-closed immutable H1/L1 JSON path replay publisher.

This adapter has one deliberately narrow job: take an already frozen H1 or
L1 JSON replay asset, prove its file hashes and rigid-transform/world-fit
contract, and publish *that exact transformed point list* as a ROS
``nav_msgs/Path``.  It never generates a path, interpolates it, smooths it,
or applies a runtime transform.

The Gazebo ``.world``/``.sdf`` and the JSON clearance geometry are deliberately
different frozen assets.  The former is provenance for what Gazebo launches;
the latter is the only object passed to the analytic fit/clearance validator.
Treating an SDF file as JSON used to make a schema-valid formal freeze fail at
this preflight boundary, so this distinction is enforced here as well as by
the shared formal-freeze validator.

The offline :func:`preflight_frozen_path_replay` helper is the important
entrypoint for launch contracts and unit tests.  ROS imports are deliberately
lazy, so preflight is usable without ROS or Gazebo.  A successful preflight
is input evidence only; it is not authorization to execute a formal row.

Formal callers must supply a SHA-256-bound freeze file and pass
``--require-formal``.  Development tests can exercise the same immutable
asset protocol with a clearly non-formal fixture, but H0/H0b/H0s,
``runtime_s_curve``, and the rejected W5/W5_S10 lineage are always refused.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple, Type


def _load_toolchain():
    """Load the sibling protocol module without importing ROS."""
    existing = sys.modules.get("smpcc_sim_toolchain")
    if existing is not None:
        return existing
    module_path = Path(__file__).with_name("smpcc_sim_toolchain.py")
    spec = importlib.util.spec_from_file_location("smpcc_sim_toolchain", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load SMPCC-SIM toolchain: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["smpcc_sim_toolchain"] = module
    spec.loader.exec_module(module)
    return module


toolchain = _load_toolchain()


ADAPTER_ID = "SMPCC-SIM-FROZEN-PATH-REPLAY-v1"
ADAPTER_SCHEMA_VERSION = "smpcc-sim-frozen-path-replay-v1"
ALLOWED_PATH_IDS = frozenset(("H1", "L1"))
_H0_IDENTITY_PATTERN = re.compile(r"(?<![a-z0-9])h0(?:b|s)?(?![a-z0-9])", re.IGNORECASE)
_RUNTIME_SCURVE_PATTERN = re.compile(r"runtime[ _-]*s[ _-]*curve", re.IGNORECASE)


class ReplayContractError(RuntimeError):
    """Raised when an immutable path replay contract is not satisfied."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ReplayContractError(message)


def _toolchain_call(callback, *args, **kwargs):
    try:
        return callback(*args, **kwargs)
    except toolchain.ContractError as exc:
        raise ReplayContractError(str(exc)) from exc


def _iter_strings(value: Any) -> Iterable[str]:
    """Yield both keys and values so hidden selector names cannot bypass checks."""
    if isinstance(value, str):
        yield value
    elif isinstance(value, Mapping):
        for key, item in value.items():
            yield str(key)
            yield from _iter_strings(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from _iter_strings(item)


def _reject_disallowed_lineage(value: Any, label: str) -> None:
    """Reject development/runtime/rejected identities before any replay read."""
    for text in _iter_strings(value):
        normalized = text.strip().lower()
        require(
            not _H0_IDENTITY_PATTERN.search(normalized),
            f"{label} contains development H0/H0b/H0s identity",
        )
        require(
            _RUNTIME_SCURVE_PATTERN.search(normalized) is None,
            f"{label} contains runtime_s_curve; frozen JSON replay is required",
        )
        # ``has_forbidden_w5`` is the protocol's canonical spelling matcher.
        require(
            not _toolchain_call(toolchain.has_forbidden_w5, text),
            f"{label} attempts to revive rejected W5/W5_S10",
        )


def _require_sha256(value: Any, label: str) -> str:
    require(_toolchain_call(toolchain.is_sha256, value), f"{label} must be a SHA-256")
    return str(value)


def _absolute_file(value: Any, label: str) -> Path:
    require(isinstance(value, str) and value, f"{label} path is missing")
    path = Path(value)
    require(path.is_absolute(), f"{label} path must be absolute")
    require(path.is_file(), f"{label} file is missing: {path}")
    return path.resolve()


def _bound_file(owner: Mapping[str, Any], path_key: str, hash_key: str, label: str) -> Path:
    """Require an absolute existing file whose content still matches freeze."""
    path = _absolute_file(owner.get(path_key), label)
    expected = _require_sha256(owner.get(hash_key), f"{label} {hash_key}")
    actual = _toolchain_call(toolchain.sha256_file, path)
    require(actual == expected, f"{label} hash mismatch for {path}")
    return path


def _read_bound_object(owner: Mapping[str, Any], path_key: str, hash_key: str, label: str) -> Tuple[Path, Mapping[str, Any]]:
    path = _bound_file(owner, path_key, hash_key, label)
    document = _toolchain_call(toolchain.read_json, path)
    try:
        mapping = _toolchain_call(toolchain.require_mapping, document, label)
    except ReplayContractError:
        raise
    _reject_disallowed_lineage(mapping, label)
    # Defend against a change between the first digest and JSON read.  The
    # final snapshot below is made only from this second verified read.
    require(
        _toolchain_call(toolchain.sha256_file, path) == owner.get(hash_key),
        f"{label} changed while being read",
    )
    return path, mapping


def _validate_path_id(path_id: Any) -> str:
    require(isinstance(path_id, str), "path_id must be a string")
    require(path_id in ALLOWED_PATH_IDS, "frozen replay accepts only H1 or L1; H0/runtime paths are refused")
    return path_id


def _validate_frame_id(value: Any) -> str:
    require(isinstance(value, str) and value == value.strip() and value, "frozen replay JSON needs a non-empty frame_id")
    require(value[0] != "/", "frozen replay frame_id must not start with '/'")
    require(re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]*", value) is not None, "frozen replay frame_id is not a valid ROS frame")
    return value


def _validate_topic(value: Any) -> str:
    require(isinstance(value, str) and value == value.strip() and value, "frozen replay JSON needs ros_path_topic")
    require(re.fullmatch(r"/[A-Za-z][A-Za-z0-9_/]*", value) is not None, "frozen replay ros_path_topic is not an absolute ROS topic")
    return value


def _finite_positive(value: Any, label: str) -> float:
    require(isinstance(value, (int, float)) and not isinstance(value, bool), f"{label} must be numeric")
    numeric = float(value)
    require(math.isfinite(numeric) and numeric > 0.0, f"{label} must be finite and positive")
    return numeric


@dataclass(frozen=True)
class FrozenPathPoint:
    """One prevalidated transformed replay point; no mutable input survives."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FrozenPathReplay:
    """Immutable result of a successful hash-bound replay preflight."""

    freeze_path: str
    freeze_file_hash: str
    path_id: str
    source_path: str
    source_path_hash: str
    sim_path: str
    sim_path_hash: str
    transform_path: str
    transform_file_hash: str
    fit_clearance_report_path: str
    fit_clearance_report_hash: str
    world_path: str
    world_file_hash: str
    world_geometry_path: str
    world_geometry_file_hash: str
    frame_id: str
    ros_path_topic: str
    points: Tuple[FrozenPathPoint, ...]
    snapshot_hash: str
    toolchain_validation_hash: str
    arc_length_m: float
    minimum_clearance_m: float
    required_clearance_m: float
    formal_freeze_validated: bool

    def report(self) -> Dict[str, Any]:
        """JSON-safe provenance report.  This does not create an experiment record."""
        return {
            "adapter_id": ADAPTER_ID,
            "schema_version": ADAPTER_SCHEMA_VERSION,
            "status": "PASS",
            "evidence_class": "FROZEN_PATH_REPLAY_PREFLIGHT_ONLY_NOT_EXPERIMENT",
            "formal_execution_authorized": False,
            "formal_freeze_validated": self.formal_freeze_validated,
            "freeze_path": self.freeze_path,
            "freeze_file_hash": self.freeze_file_hash,
            "path_id": self.path_id,
            "source_path": self.source_path,
            "source_path_hash": self.source_path_hash,
            "sim_path": self.sim_path,
            "sim_path_hash": self.sim_path_hash,
            "transform_path": self.transform_path,
            "transform_file_hash": self.transform_file_hash,
            "fit_clearance_report_path": self.fit_clearance_report_path,
            "fit_clearance_report_hash": self.fit_clearance_report_hash,
            "world_path": self.world_path,
            "world_file_hash": self.world_file_hash,
            "world_geometry_path": self.world_geometry_path,
            "world_geometry_file_hash": self.world_geometry_file_hash,
            "frame_id": self.frame_id,
            "ros_path_topic": self.ros_path_topic,
            "point_count": len(self.points),
            "snapshot_hash": self.snapshot_hash,
            "toolchain_validation_hash": self.toolchain_validation_hash,
            "arc_length_m": self.arc_length_m,
            "minimum_clearance_m": self.minimum_clearance_m,
            "required_clearance_m": self.required_clearance_m,
        }


def _validate_fit_report(
    report: Mapping[str, Any],
    validation: Mapping[str, Any],
    path_id: str,
    required_clearance_m: float,
) -> None:
    """Bind the archived PASS report to the fresh toolchain computation.

    The toolchain result remains authoritative: this merely prevents an
    unrelated or stale report from being carried as the frozen evidence file.
    """
    require(report.get("status") == "PASS", "frozen fit/clearance report is not PASS")
    require(report.get("path_id") == path_id, "fit/clearance report path_id differs from replay")
    for key in ("source_path_hash", "sim_path_hash", "transform_hash", "world_hash"):
        require(report.get(key) == validation.get(key), f"fit/clearance report {key} differs from fresh toolchain validation")
    report_clearance = _finite_positive(report.get("clearance_m"), "fit/clearance report clearance_m")
    require(abs(report_clearance - required_clearance_m) <= 1e-12, "fit/clearance report clearance_m differs from frozen path entry")
    minimum = report.get("minimum_clearance_m")
    require(isinstance(minimum, (int, float)) and not isinstance(minimum, bool) and math.isfinite(float(minimum)), "fit/clearance report minimum_clearance_m is invalid")
    require(float(minimum) + 1e-9 >= required_clearance_m, "fit/clearance report says required clearance was not met")
    require(abs(float(minimum) - float(validation["minimum_clearance_m"])) <= 1e-6, "fit/clearance report minimum clearance differs from fresh validation")
    require(report.get("clearance_method") == validation.get("clearance_method"), "fit/clearance report clearance method differs from fresh validation")


def preflight_frozen_path_replay(
    freeze_path: Path,
    freeze_file_hash: str,
    path_id: str,
    *,
    require_formal: bool = False,
) -> FrozenPathReplay:
    """Read and validate one immutable H1/L1 replay without ROS/Gazebo.

    The function deliberately has no output path and no file-writing branch.
    It verifies the freeze itself, source/transformed JSON, rigid transform,
    fit/clearance report, and world asset before returning an immutable point
    snapshot.  ``require_formal`` performs the full existing toolchain freeze
    gate as an additional guard; even then this helper does not authorize an
    experiment or launch a runtime.
    """
    selected_path_id = _validate_path_id(path_id)
    freeze_path = _absolute_file(str(freeze_path), "formal/development freeze")
    expected_freeze_hash = _require_sha256(freeze_file_hash, "freeze_file_hash")
    require(
        _toolchain_call(toolchain.sha256_file, freeze_path) == expected_freeze_hash,
        "freeze file hash mismatch",
    )
    raw_freeze = _toolchain_call(toolchain.read_json, freeze_path)
    freeze = _toolchain_call(toolchain.require_mapping, raw_freeze, "freeze")
    require(
        _toolchain_call(toolchain.sha256_file, freeze_path) == expected_freeze_hash,
        "freeze file changed while being read",
    )

    paths = freeze.get("paths")
    require(isinstance(paths, Mapping), "freeze lacks frozen JSON path registry")
    # A path registry containing a development or rejected path is not a
    # valid input to this adapter, even if the caller nominally selects H1.
    _reject_disallowed_lineage(paths, "freeze path registry")
    require(set(paths).issubset(ALLOWED_PATH_IDS), "freeze path registry may contain only H1/L1 replay entries")
    entry = paths.get(selected_path_id)
    require(isinstance(entry, Mapping), f"freeze lacks {selected_path_id} path entry")
    _reject_disallowed_lineage(entry, f"freeze path {selected_path_id}")
    require(entry.get("source_mode") == "frozen_json_replay", "frozen replay requires source_mode=frozen_json_replay")
    required_clearance_m = _finite_positive(entry.get("clearance_m"), "freeze path clearance_m")

    source_path, source_document = _read_bound_object(entry, "source_path", "source_path_hash", "frozen source JSON")
    sim_path, sim_document = _read_bound_object(entry, "sim_path", "sim_path_hash", "frozen transformed replay JSON")
    transform_path, transform = _read_bound_object(entry, "transform_path", "transform_hash", "frozen rigid transform JSON")
    fit_path, fit_report = _read_bound_object(entry, "fit_clearance_report_path", "fit_clearance_report_hash", "frozen fit/clearance report")

    require(source_document.get("path_id") == selected_path_id, "frozen source JSON path_id differs from selected H1/L1")
    require(sim_document.get("path_id") == selected_path_id, "frozen transformed replay JSON path_id differs from selected H1/L1")
    require(source_document.get("source_mode") == "frozen_json_replay", "frozen source JSON is not immutable replay")
    require(sim_document.get("source_mode") == "frozen_json_replay", "transformed replay JSON is not immutable replay")
    frame_id = _validate_frame_id(sim_document.get("frame_id"))
    topic = _validate_topic(sim_document.get("ros_path_topic"))

    simulator_assets = freeze.get("simulator_assets")
    require(isinstance(simulator_assets, Mapping), "freeze lacks simulator_assets for path/world fit")
    _reject_disallowed_lineage(simulator_assets.get("world_file"), "frozen world asset")
    # ``world_file`` is the actual launched SDF/XML artifact.  It must be
    # hash-bound even though the clearance math below intentionally never
    # parses it.  ``world_geometry_file`` is the reviewed JSON representation
    # whose geometry is used by validate_path_replay.  Keeping them separate
    # closes an integration gap between this publisher and
    # validate_formal_simulator_assets().
    world_path = _bound_file(simulator_assets, "world_file", "world_hash", "frozen launched Gazebo world")
    geometry_path = _bound_file(
        simulator_assets,
        "world_geometry_file",
        "world_geometry_hash",
        "frozen world_geometry_file clearance geometry JSON",
    )
    require(world_path != geometry_path, "frozen launched world and clearance geometry must be distinct files")
    require(
        simulator_assets.get("world_hash") != simulator_assets.get("world_geometry_hash"),
        "frozen launched world and clearance geometry must not alias hashes",
    )
    geometry_document = _toolchain_call(toolchain.read_json, geometry_path)
    world = _toolchain_call(
        toolchain.require_mapping,
        geometry_document,
        "frozen world_geometry_file clearance geometry JSON",
    )
    _reject_disallowed_lineage(world, "frozen world_geometry_file clearance geometry JSON")
    require(
        _toolchain_call(toolchain.sha256_file, geometry_path) == simulator_assets.get("world_geometry_hash"),
        "frozen world_geometry_file clearance geometry JSON changed while being read",
    )

    validation = _toolchain_call(
        toolchain.validate_path_replay,
        source_path,
        sim_path,
        transform,
        world,
        required_clearance_m,
    )
    require(validation.get("path_id") == selected_path_id, "toolchain replay validation returned another path_id")
    require(validation.get("source_path_hash") == entry.get("source_path_hash"), "toolchain source JSON hash differs from freeze")
    require(validation.get("sim_path_hash") == entry.get("sim_path_hash"), "toolchain transformed replay JSON hash differs from freeze")
    _validate_fit_report(fit_report, validation, selected_path_id, required_clearance_m)

    points_raw = _toolchain_call(toolchain.points_from_path, sim_document)
    points: Tuple[FrozenPathPoint, ...] = tuple(
        FrozenPathPoint(x=x, y=y, yaw=yaw)
        for x, y, yaw in points_raw
        if yaw is not None
    )
    require(len(points) == len(points_raw), "transformed replay JSON requires an explicit finite yaw for every point; runtime heading reconstruction is forbidden")
    require(len(points) >= 2, "transformed replay JSON needs at least two immutable points")

    # Verify every replay-relevant file again immediately before snapshotting.
    for owner, path_key, hash_key, path, label in (
        (entry, "source_path", "source_path_hash", source_path, "frozen source JSON"),
        (entry, "sim_path", "sim_path_hash", sim_path, "frozen transformed replay JSON"),
        (entry, "transform_path", "transform_hash", transform_path, "frozen rigid transform JSON"),
        (entry, "fit_clearance_report_path", "fit_clearance_report_hash", fit_path, "frozen fit/clearance report"),
        (simulator_assets, "world_file", "world_hash", world_path, "frozen launched Gazebo world"),
        (
            simulator_assets,
            "world_geometry_file",
            "world_geometry_hash",
            geometry_path,
            "frozen world_geometry_file clearance geometry JSON",
        ),
    ):
        require(_toolchain_call(toolchain.sha256_file, path) == owner.get(hash_key), f"{label} changed before immutable snapshot")
    require(
        _toolchain_call(toolchain.sha256_file, freeze_path) == expected_freeze_hash,
        "freeze file changed before immutable snapshot",
    )

    formal_freeze_validated = False
    if require_formal:
        gate = _toolchain_call(toolchain.validate_formal_freeze, freeze)
        require(gate.get("status") == "PASS", "formal freeze gate did not PASS: " + "; ".join(str(item) for item in gate.get("errors", [])))
        formal_freeze_validated = True

    snapshot_core = {
        "adapter_id": ADAPTER_ID,
        "freeze_file_hash": expected_freeze_hash,
        "path_id": selected_path_id,
        "source_path_hash": entry["source_path_hash"],
        "sim_path_hash": entry["sim_path_hash"],
        "transform_file_hash": entry["transform_hash"],
        "fit_clearance_report_hash": entry["fit_clearance_report_hash"],
        "world_file_hash": simulator_assets["world_hash"],
        "world_geometry_file_hash": simulator_assets["world_geometry_hash"],
        "frame_id": frame_id,
        "ros_path_topic": topic,
        "points": [{"x": point.x, "y": point.y, "yaw": point.yaw} for point in points],
    }
    return FrozenPathReplay(
        freeze_path=str(freeze_path),
        freeze_file_hash=expected_freeze_hash,
        path_id=selected_path_id,
        source_path=str(source_path),
        source_path_hash=str(entry["source_path_hash"]),
        sim_path=str(sim_path),
        sim_path_hash=str(entry["sim_path_hash"]),
        transform_path=str(transform_path),
        transform_file_hash=str(entry["transform_hash"]),
        fit_clearance_report_path=str(fit_path),
        fit_clearance_report_hash=str(entry["fit_clearance_report_hash"]),
        world_path=str(world_path),
        world_file_hash=str(simulator_assets["world_hash"]),
        world_geometry_path=str(geometry_path),
        world_geometry_file_hash=str(simulator_assets["world_geometry_hash"]),
        frame_id=frame_id,
        ros_path_topic=topic,
        points=points,
        snapshot_hash=_toolchain_call(toolchain.canonical_hash, snapshot_core),
        toolchain_validation_hash=_toolchain_call(toolchain.canonical_hash, validation),
        arc_length_m=float(validation["arc_length"]),
        minimum_clearance_m=float(validation["minimum_clearance_m"]),
        required_clearance_m=required_clearance_m,
        formal_freeze_validated=formal_freeze_validated,
    )


def build_ros_path_message(
    replay: FrozenPathReplay,
    ros_path_type: Type[Any],
    pose_stamped_type: Type[Any],
    stamp: Any,
) -> Any:
    """Materialize a ROS Path from the already immutable snapshot only.

    This is intentionally a representation conversion, not path processing:
    there is no file access, transform, interpolation, resampling, smoothing,
    or heading estimation in this function.
    """
    message = ros_path_type()
    message.header.frame_id = replay.frame_id
    message.header.stamp = stamp
    for point in replay.points:
        pose = pose_stamped_type()
        pose.header.frame_id = replay.frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(point.yaw / 2.0)
        pose.pose.orientation.w = math.cos(point.yaw / 2.0)
        message.poses.append(pose)
    return message


def publish_immutable_replay(
    replay: FrozenPathReplay,
    publisher: Any,
    ros_path_type: Type[Any],
    pose_stamped_type: Type[Any],
    stamp: Any,
) -> Any:
    """Publish exactly one immutable message through an already-created publisher."""
    message = build_ros_path_message(replay, ros_path_type, pose_stamped_type, stamp)
    publisher.publish(message)
    return message


def run_ros_publisher(replay: FrozenPathReplay) -> None:
    """Run the long-lived ROS1 publisher after all offline checks have passed."""
    try:
        import rospy  # type: ignore[import-not-found]
        from geometry_msgs.msg import PoseStamped  # type: ignore[import-not-found]
        from nav_msgs.msg import Path as RosPath  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - depends on runtime installation
        raise ReplayContractError("ROS1 rospy/nav_msgs/geometry_msgs are required only for publish") from exc

    rospy.init_node("smpcc_frozen_path_replay", anonymous=False)
    publisher = rospy.Publisher(replay.ros_path_topic, RosPath, queue_size=1, latch=True)
    # A latched publisher must remain alive for controllers/RViz that connect
    # after process startup.  The single-row runner owns this PID and stops it
    # using its tracked process group; this adapter never kills other nodes.
    publish_immutable_replay(replay, publisher, RosPath, PoseStamped, rospy.Time.now())
    rospy.spin()


def _emit(value: Any) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))


def _command_preflight(args: argparse.Namespace) -> int:
    try:
        replay = preflight_frozen_path_replay(
            args.freeze,
            args.freeze_sha256,
            args.path_id,
            require_formal=args.require_formal,
        )
    except ReplayContractError as exc:
        _emit({"adapter_id": ADAPTER_ID, "status": "FAIL", "error": str(exc), "formal_execution_authorized": False})
        return 2
    _emit(replay.report())
    return 0


def _command_publish(args: argparse.Namespace) -> int:
    try:
        replay = preflight_frozen_path_replay(
            args.freeze,
            args.freeze_sha256,
            args.path_id,
            require_formal=args.require_formal,
        )
        run_ros_publisher(replay)
    except ReplayContractError as exc:
        _emit({"adapter_id": ADAPTER_ID, "status": "FAIL", "error": str(exc), "formal_execution_authorized": False})
        return 2
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Strict immutable H1/L1 frozen JSON Path replay")
    sub = parser.add_subparsers(dest="command", required=True)
    for name, handler, help_text in (
        ("preflight", _command_preflight, "offline hash/transform/fit/clearance preflight; never starts ROS"),
        ("publish", _command_publish, "publish the preflighted immutable JSON points as a latched ROS Path"),
    ):
        item = sub.add_parser(name, help=help_text)
        item.add_argument("--freeze", type=Path, required=True, help="absolute frozen path registry JSON")
        item.add_argument("--freeze-sha256", required=True, help="expected SHA-256 of --freeze")
        item.add_argument("--path-id", choices=sorted(ALLOWED_PATH_IDS), required=True)
        item.add_argument("--require-formal", action="store_true", help="also require the complete toolchain formal-freeze gate")
        item.set_defaults(func=handler)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
