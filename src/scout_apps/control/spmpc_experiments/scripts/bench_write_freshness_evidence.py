#!/usr/bin/env python3
"""Write strict fresh-sim evidence in bench_check_freshness.py schema.

This utility bridges existing /data/a/scout_sim_replacement strict manifests to the
explicit evidence fields required by bench_check_freshness.py. It does not start
or stop ROS/Gazebo and does not classify an already-running simulator as strict.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised only on missing dep
    raise SystemExit("PyYAML missing; install python3-yaml or pip3 install pyyaml") from exc


class EvidenceWriter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.report: Dict[str, Any] = {
            "tool": "bench_write_freshness_evidence.py",
            "schema_version": 1,
            "runtime_actions": {
                "starts_ros": False,
                "starts_gazebo": False,
                "kills_processes": False,
                "writes_files": bool(args.out_file),
                "changes_cmd_vel_chain": False,
                "changes_spmpc_ocp_inputs": False,
            },
            "errors": [],
            "warnings": [],
            "infos": [],
        }

    def run(self) -> Dict[str, Any]:
        batch_meta = self.load_batch_meta(self.args.batch_meta) if self.args.batch_meta else {}
        manifest_row = self.load_manifest_row(self.args.manifest) if self.args.manifest else {}
        freshness = self.build_freshness(batch_meta, manifest_row)
        evidence = {
            "schema_version": 1,
            "source_schema_version": "scout_sim_replacement_manifest_v1",
            "freshness": freshness,
            "source": {
                "manifest": str(self.args.manifest) if self.args.manifest else None,
                "batch_meta": str(self.args.batch_meta) if self.args.batch_meta else None,
                "case_label": freshness.get("case_label"),
                "planner": freshness.get("planner"),
                "run_index": freshness.get("run_index"),
            },
            "runtime_actions": self.report["runtime_actions"],
        }
        self.report["evidence"] = evidence

        if self.args.out_file:
            self.write_evidence(evidence, self.args.out_file)
        else:
            self.warn("NO_OUT_FILE", "No --out-file supplied; evidence was printed only.")

        self.report["ok"] = not self.report["errors"]
        self.report["status"] = "PASS" if self.report["ok"] else "FAIL"
        return self.report

    def load_batch_meta(self, path: Path) -> Dict[str, Any]:
        if not path.is_file():
            self.error("BATCH_META_MISSING", f"batch_meta file not found: {path}", path)
            return {}
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception as exc:  # noqa: BLE001
            self.error("BATCH_META_PARSE_FAILED", f"Failed to parse {path}: {exc}", path)
            return {}
        if not isinstance(data, dict):
            self.error("BATCH_META_INVALID", f"batch_meta must be a mapping: {path}", path)
            return {}
        self.info("BATCH_META_LOADED", f"Loaded batch_meta: {path}", path)
        return data

    def load_manifest_row(self, path: Path) -> Dict[str, str]:
        if not path.is_file():
            self.error("MANIFEST_MISSING", f"manifest file not found: {path}", path)
            return {}
        rows = []
        try:
            with path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                rows = list(reader)
        except Exception as exc:  # noqa: BLE001
            self.error("MANIFEST_PARSE_FAILED", f"Failed to parse CSV manifest {path}: {exc}", path)
            return {}
        if not rows:
            self.error("MANIFEST_EMPTY", f"manifest contains no rows: {path}", path)
            return {}

        candidates = rows
        if self.args.case_label:
            candidates = [row for row in candidates if row.get("case_label") == self.args.case_label]
        if self.args.planner:
            candidates = [row for row in candidates if row.get("planner") == self.args.planner]
        if self.args.run_index is not None:
            candidates = [row for row in candidates if str(row.get("run_index")) == str(self.args.run_index)]

        if len(candidates) != 1:
            self.error(
                "MANIFEST_ROW_SELECTION_FAILED",
                f"Expected exactly one manifest row after filters, got {len(candidates)} "
                f"(case_label={self.args.case_label!r}, planner={self.args.planner!r}, run_index={self.args.run_index!r})",
                path,
            )
            return {}
        self.info("MANIFEST_ROW_SELECTED", f"Selected manifest row case_label={candidates[0].get('case_label')}", path)
        return candidates[0]

    def build_freshness(self, batch_meta: Dict[str, Any], row: Dict[str, str]) -> Dict[str, Any]:
        strict_requested = self.args.strict_requested
        one_case = self.args.one_case_per_fresh_sim
        settle_completed = self.args.settle_completed
        shutdown_after_case = self.args.shutdown_after_case
        cooldown_completed = self.args.cooldown_completed

        if batch_meta:
            strict_requested = strict_requested or parse_bool(batch_meta.get("strict_fresh_sim"), default=False)
            one_case = one_case or int_or_zero(batch_meta.get("n")) >= 1
            settle_completed = settle_completed or float_or_zero(batch_meta.get("strict_pre_control_settle_sec")) > 0.0
            shutdown_after_case = shutdown_after_case or True
            cooldown_completed = cooldown_completed or float_or_zero(batch_meta.get("strict_post_shutdown_sec")) > 0.0

        if row:
            valid_strict_case = parse_bool(row.get("valid_strict_case"), default=False)
            if not valid_strict_case:
                self.warn("SOURCE_CASE_NOT_VALID_STRICT", "Source manifest row valid_strict_case is not true; converted evidence will remain blocked from strict fresh acceptance.")
                one_case = False
            pre_ros_alive = parse_bool(row.get("pre_ros_reachable"), default=True)
            pre_gazebo_alive = parse_bool(row.get("pre_gazebo_reachable"), default=True)
            post_ros_alive = parse_bool(row.get("post_ros_reachable"), default=True)
            post_gazebo_alive = parse_bool(row.get("post_gazebo_reachable"), default=True)
            exit_status = int_or_none(row.get("exit_status"))
        else:
            pre_ros_alive = self.args.pre_ros_alive
            pre_gazebo_alive = self.args.pre_gazebo_alive
            post_ros_alive = self.args.post_ros_alive
            post_gazebo_alive = self.args.post_gazebo_alive
            valid_strict_case = None
            exit_status = None

        freshness = {
            "strict_requested": bool(strict_requested),
            "one_case_per_fresh_sim": bool(one_case),
            "settle_completed": bool(settle_completed),
            "shutdown_after_case": bool(shutdown_after_case),
            "cooldown_completed": bool(cooldown_completed),
            "pre_ros_alive": bool(pre_ros_alive),
            "pre_gazebo_alive": bool(pre_gazebo_alive),
            "post_ros_alive": bool(post_ros_alive),
            "post_gazebo_alive": bool(post_gazebo_alive),
            "source_valid_strict_case": valid_strict_case,
            "source_exit_status": exit_status,
            "case_label": row.get("case_label") if row else self.args.case_label,
            "planner": row.get("planner") if row else self.args.planner,
            "run_index": int_or_none(row.get("run_index")) if row else self.args.run_index,
            "summary_path": row.get("summary_path") if row else None,
            "bag_path": row.get("bag_path") if row else None,
            "result_dir": row.get("result_dir") if row else None,
            "bag_dir": row.get("bag_dir") if row else None,
        }
        return freshness

    def write_evidence(self, evidence: Dict[str, Any], path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        text: str
        if path.suffix.lower() == ".json" or self.args.format == "json":
            text = json.dumps(evidence, indent=2, ensure_ascii=False, sort_keys=False) + "\n"
        else:
            text = yaml.safe_dump(evidence, allow_unicode=True, sort_keys=False)
        path.write_text(text, encoding="utf-8")
        self.info("EVIDENCE_WRITTEN", f"Wrote freshness evidence: {path}", path)

    def error(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["errors"].append(make_item(code, message, path))

    def warn(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["warnings"].append(make_item(code, message, path))

    def info(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["infos"].append(make_item(code, message, path))


def parse_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    return default


def float_or_zero(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def int_or_zero(value: Any) -> int:
    parsed = int_or_none(value)
    return 0 if parsed is None else parsed


def int_or_none(value: Any) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def make_item(code: str, message: str, path: Optional[Path] = None) -> Dict[str, Any]:
    item: Dict[str, Any] = {"code": code, "message": message}
    if path is not None:
        item["path"] = str(path)
    return item


def emit_report(report: Dict[str, Any], fmt: str) -> None:
    if fmt == "yaml":
        print(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    else:
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=False))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert strict fresh-sim manifest rows into bench_check_freshness.py evidence schema.")
    parser.add_argument("--manifest", type=Path, default=None, help="strict_fresh_manifest.csv from /data/a/scout_sim_replacement")
    parser.add_argument("--batch-meta", type=Path, default=None, help="batch_meta.yaml from the same strict fresh run")
    parser.add_argument("--case-label", default=None, help="Manifest case_label selector")
    parser.add_argument("--planner", default=None, help="Manifest planner selector")
    parser.add_argument("--run-index", type=int, default=None, help="Manifest run_index selector")
    parser.add_argument("--out-file", type=Path, default=None, help="Output YAML/JSON evidence path")
    parser.add_argument("--format", choices=("json", "yaml"), default="yaml", help="Report/evidence format preference")
    parser.add_argument("--strict-requested", action="store_true", help="Set strict_requested=true when writing direct evidence")
    parser.add_argument("--one-case-per-fresh-sim", action="store_true")
    parser.add_argument("--settle-completed", action="store_true")
    parser.add_argument("--shutdown-after-case", action="store_true")
    parser.add_argument("--cooldown-completed", action="store_true")
    parser.add_argument("--pre-ros-alive", action="store_true", help="Direct evidence fallback; manifest mode maps pre_ros_reachable instead")
    parser.add_argument("--pre-gazebo-alive", action="store_true")
    parser.add_argument("--post-ros-alive", action="store_true")
    parser.add_argument("--post-gazebo-alive", action="store_true")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    writer = EvidenceWriter(args)
    report = writer.run()
    emit_report(report, args.format)
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
