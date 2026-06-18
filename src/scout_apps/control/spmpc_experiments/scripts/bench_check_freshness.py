#!/usr/bin/env python3
"""Dry-run freshness classifier for benchmark evidence.

Phase 0 scope: classify whether a run has explicit strict-fresh evidence. The
script does not start or stop ROS/Gazebo, does not kill processes, and does not
infer strict freshness from an already-running simulator.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

try:
    import yaml
except ImportError as exc:  # pragma: no cover - exercised only on missing dep
    raise SystemExit(
        "PyYAML missing; install python3-yaml or pip3 install pyyaml"
    ) from exc


STRICT_REQUIRED_TRUE = (
    "strict_requested",
    "one_case_per_fresh_sim",
    "settle_completed",
    "shutdown_after_case",
    "cooldown_completed",
)

STRICT_REQUIRED_FALSE = (
    "pre_ros_alive",
    "pre_gazebo_alive",
    "post_ros_alive",
    "post_gazebo_alive",
)


class FreshnessCheck:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.repo_root = args.repo_root.resolve() if args.repo_root else infer_repo_root()
        self.report: Dict[str, Any] = {
            "tool": "bench_check_freshness.py",
            "schema_version": 1,
            "mode": args.mode,
            "strict_requested": bool(args.strict),
            "dry_run": bool(args.dry_run),
            "repo_root": str(self.repo_root),
            "runtime_actions": {
                "starts_ros": False,
                "starts_gazebo": False,
                "kills_processes": False,
                "writes_files": False,
                "checks_only": True,
            },
            "freshness": {
                "strict_fresh": False,
                "run_type": "current_sim_diagnostics",
                "passed": False,
                "pre_ros_alive": "unknown",
                "pre_gazebo_alive": "unknown",
                "post_ros_alive": "unknown",
                "post_gazebo_alive": "unknown",
            },
            "errors": [],
            "warnings": [],
            "infos": [],
        }

    def check(self) -> Dict[str, Any]:
        if not self.args.dry_run:
            self.error(
                "DRY_RUN_REQUIRED",
                "Phase 0 freshness checker is dry-run/report-only; rerun with --dry-run.",
            )

        evidence = self.load_evidence() if self.args.evidence_file else None
        if evidence is None:
            self.classify_without_evidence()
        else:
            self.classify_with_evidence(evidence)

        tool_ok = not self.report["errors"]
        self.report["ok"] = tool_ok
        self.report["status"] = "PASS" if tool_ok else "FAIL"
        return self.report

    def load_evidence(self) -> Optional[Dict[str, Any]]:
        path = self.args.evidence_file
        if path is None:
            return None
        if not path.is_file():
            self.error("EVIDENCE_FILE_MISSING", f"Freshness evidence file not found: {path}", path)
            return None
        try:
            text = path.read_text(encoding="utf-8")
            if path.suffix.lower() == ".json":
                data = json.loads(text)
            else:
                data = yaml.safe_load(text)
        except Exception as exc:  # noqa: BLE001 - report parse failures clearly
            self.error("EVIDENCE_PARSE_FAILED", f"Failed to parse evidence file {path}: {exc}", path)
            return None
        if not isinstance(data, dict):
            self.error("EVIDENCE_NOT_MAPPING", f"Freshness evidence must be a mapping: {path}", path)
            return None
        return data

    def classify_without_evidence(self) -> None:
        freshness = self.report["freshness"]
        if self.args.strict:
            freshness.update(
                {
                    "strict_fresh": False,
                    "run_type": "current_sim_diagnostics",
                    "passed": False,
                    "stop_main_table": True,
                    "reason": "NO_STRICT_FRESH_EVIDENCE",
                }
            )
            message = "No explicit lifecycle evidence was supplied; do not enter strict fresh-sim main table."
            if self.args.strict_main_table:
                self.error("NON_STRICT_FRESH_SIM", message)
            else:
                self.warn("NO_STRICT_FRESH_EVIDENCE", message)
        else:
            freshness.update(
                {
                    "strict_fresh": False,
                    "run_type": "current_sim_diagnostics",
                    "passed": True,
                    "stop_main_table": True,
                    "reason": "STRICT_NOT_REQUESTED",
                }
            )
            self.info(
                "CURRENT_SIM_DIAGNOSTICS_ONLY",
                "No strict freshness requested; classify as current-sim diagnostics only.",
            )

    def classify_with_evidence(self, evidence: Dict[str, Any]) -> None:
        freshness = evidence.get("freshness", evidence)
        if not isinstance(freshness, dict):
            self.error("FRESHNESS_EVIDENCE_INVALID", "freshness evidence must be a mapping")
            return

        missing_true = [key for key in STRICT_REQUIRED_TRUE if freshness.get(key) is not True]
        missing_false = [key for key in STRICT_REQUIRED_FALSE if freshness.get(key) is not False]
        strict_fresh = not missing_true and not missing_false

        self.report["freshness"].update(
            {
                "strict_fresh": strict_fresh,
                "run_type": "strict_fresh" if strict_fresh else "current_sim_diagnostics",
                "passed": strict_fresh if self.args.strict else True,
                "stop_main_table": not strict_fresh,
                "pre_ros_alive": freshness.get("pre_ros_alive", "unknown"),
                "pre_gazebo_alive": freshness.get("pre_gazebo_alive", "unknown"),
                "post_ros_alive": freshness.get("post_ros_alive", "unknown"),
                "post_gazebo_alive": freshness.get("post_gazebo_alive", "unknown"),
                "evidence_file": str(self.args.evidence_file) if self.args.evidence_file else None,
            }
        )
        if strict_fresh:
            self.info("STRICT_FRESH_EVIDENCE_ACCEPTED", "Freshness evidence satisfies strict lifecycle fields.")
        else:
            reason = {
                "missing_true_fields": missing_true,
                "missing_false_fields": missing_false,
            }
            self.report["freshness"]["reason"] = reason
            if self.args.strict_main_table:
                self.error("NON_STRICT_FRESH_SIM", f"Strict freshness evidence incomplete: {reason}")
            else:
                self.warn("NON_STRICT_FRESH_SIM", f"Strict freshness evidence incomplete: {reason}")

    def error(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["errors"].append(make_item(code, message, path))

    def warn(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["warnings"].append(make_item(code, message, path))

    def info(self, code: str, message: str, path: Optional[Path] = None) -> None:
        self.report["infos"].append(make_item(code, message, path))


def make_item(code: str, message: str, path: Optional[Path] = None) -> Dict[str, Any]:
    item: Dict[str, Any] = {"code": code, "message": message}
    if path is not None:
        item["path"] = str(path)
    return item


def infer_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "src/scout_apps/control/spmpc_experiments").is_dir():
            return parent
    return Path.cwd().resolve()


def emit_report(report: Dict[str, Any], fmt: str) -> None:
    if fmt == "yaml":
        print(yaml.safe_dump(report, allow_unicode=True, sort_keys=False))
    else:
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=False))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dry-run strict fresh-sim classifier for benchmark evidence."
    )
    parser.add_argument("--mode", choices=("sim", "real"), default="sim")
    parser.add_argument("--strict", action="store_true", help="Request strict fresh-sim classification.")
    parser.add_argument("--dry-run", action="store_true", help="Required in Phase 0; no runtime actions are taken.")
    parser.add_argument("--strict-main-table", action="store_true", help="Return failure if strict evidence is incomplete.")
    parser.add_argument("--evidence-file", type=Path, default=None, help="Optional YAML/JSON freshness evidence file.")
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--format", choices=("json", "yaml"), default="json")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    checker = FreshnessCheck(args)
    report = checker.check()
    emit_report(report, args.format)
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
