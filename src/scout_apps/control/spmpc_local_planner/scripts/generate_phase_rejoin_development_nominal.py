#!/usr/bin/env python3
"""Extract a frozen ROS Path and delegate development nominal generation to C++."""

import argparse
import csv
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Iterable, NamedTuple, Sequence, Tuple


class GenerationError(RuntimeError):
    pass


class Point(NamedTuple):
    x: float
    y: float


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_path_from_bag(path: Path, topic: str) -> Tuple[Point, ...]:
    """Keep rosbag extraction in Python; C++ owns every domain check."""
    try:
        import rosbag
    except ImportError as exc:
        raise GenerationError("rosbag is required to read --bag") from exc
    try:
        with rosbag.Bag(str(path), "r") as bag:
            for _topic, message, _stamp in bag.read_messages(topics=[topic]):
                poses = list(getattr(message, "poses", ()))
                if len(poses) >= 3:
                    return tuple(
                        Point(item.pose.position.x, item.pose.position.y)
                        for item in poses
                    )
    except GenerationError:
        raise
    except Exception as exc:
        raise GenerationError(f"failed to read path from bag: {path}") from exc
    raise GenerationError(f"no nav_msgs/Path with at least three poses found on {topic}")


def _generator_path() -> Path:
    configured = os.environ.get(
        "SPMPC_PHASE_REJOIN_DEVELOPMENT_NOMINAL", ""
    ).strip()
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    executable = shutil.which("spmpc_phase_rejoin_development_nominal")
    if executable:
        candidates.append(Path(executable))
    for parent in Path(__file__).resolve().parents:
        candidates.append(
            parent
            / "devel/lib/spmpc_local_planner/"
            "spmpc_phase_rejoin_development_nominal"
        )
    for candidate in candidates:
        if candidate.is_file() and os.access(str(candidate), os.X_OK):
            return candidate.resolve()
    raise GenerationError(
        "C++ development nominal generator is unavailable; build "
        "spmpc_phase_rejoin_development_nominal or set "
        "SPMPC_PHASE_REJOIN_DEVELOPMENT_NOMINAL"
    )


def _format(value: float) -> str:
    return format(float(value), ".17g")


def _write_path_csv(path: Path, points: Iterable[Point]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("x", "y"))
        for point in points:
            writer.writerow((_format(point.x), _format(point.y)))
        stream.flush()
        os.fsync(stream.fileno())


def generate_from_points(
    points: Sequence[Point],
    args: argparse.Namespace,
    source_bag_sha256: str,
) -> str:
    """Pass extracted points and provenance to the fail-closed C++ generator."""
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = ""
    try:
        with tempfile.NamedTemporaryFile(
            prefix=output.name + ".path.",
            suffix=".csv",
            dir=str(output.parent),
            delete=False,
        ) as stream:
            temporary = stream.name
        _write_path_csv(Path(temporary), points)
        command = [
            str(_generator_path()),
            "--path-csv", temporary,
            "--output", str(output),
            "--contract-id", args.contract_id,
            "--frame-id", args.frame_id,
            "--dt", _format(args.dt),
            "--cruise-speed", _format(args.cruise_speed),
            "--ramp-sec", _format(args.ramp_sec),
            "--lookahead", _format(args.lookahead),
            "--heading-gain", _format(args.heading_gain),
            "--omega-max", _format(args.omega_max),
            "--alpha-max", _format(args.alpha_max),
            "--omega-n", _format(args.omega_n),
            "--damping-ratio", _format(args.damping_ratio),
            "--kappa-x", _format(args.kappa_x),
            "--kappa-y", _format(args.kappa_y),
            "--zero-hold-sec", _format(args.zero_hold_sec),
            "--terminal-eta-norm-max", _format(args.terminal_eta_norm_max),
            "--terminal-eta-dot-norm-max",
            _format(args.terminal_eta_dot_norm_max),
            "--gate-radii",
            *(_format(value) for value in args.gate_radii),
            "--source-bag-sha256", source_bag_sha256,
            "--path-topic", args.path_topic,
        ]
        if args.overwrite:
            command.append("--overwrite")
        try:
            completed = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as exc:
            raise GenerationError("failed to execute C++ nominal generator") from exc
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip()
            raise GenerationError(
                detail or f"C++ nominal generator exited {completed.returncode}"
            )
        return completed.stdout
    finally:
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Extract a ROS Path and generate a DEVELOPMENT-ONLY Phase-Rejoin "
            "v2 artifact through the shared C++ dynamics and artifact core."
        )
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--path-topic", default="/scout/global_path_fixed")
    parser.add_argument("--output", required=True)
    parser.add_argument("--contract-id", required=True)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--dt", type=float, default=0.0333333333)
    parser.add_argument("--cruise-speed", type=float, default=0.34)
    parser.add_argument("--ramp-sec", type=float, default=2.5)
    parser.add_argument("--lookahead", type=float, default=0.30)
    parser.add_argument("--heading-gain", type=float, default=3.0)
    parser.add_argument("--omega-max", type=float, default=1.0)
    parser.add_argument("--alpha-max", type=float, default=1.0)
    parser.add_argument("--omega-n", type=float, required=True)
    parser.add_argument("--damping-ratio", type=float, default=0.05)
    parser.add_argument("--kappa-x", type=float, default=1.0)
    parser.add_argument("--kappa-y", type=float, default=1.0)
    parser.add_argument("--zero-hold-sec", type=float, default=2.0)
    parser.add_argument("--terminal-eta-norm-max", type=float, default=2.0e-6)
    parser.add_argument(
        "--terminal-eta-dot-norm-max", type=float, default=1.0e-4
    )
    parser.add_argument(
        "--gate-radii",
        type=float,
        nargs=9,
        required=True,
        metavar=("RX", "RY", "RYAW", "RV", "ROMEGA", "REX", "REXD", "REY", "REYD"),
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] = None) -> int:
    args = build_parser().parse_args(argv)
    bag = Path(args.bag).expanduser().resolve()
    if not bag.is_file():
        raise GenerationError(f"bag does not exist: {bag}")
    points = load_path_from_bag(bag, args.path_topic)
    output = generate_from_points(points, args, sha256(bag))
    print(output, end="" if output.endswith("\n") else "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
