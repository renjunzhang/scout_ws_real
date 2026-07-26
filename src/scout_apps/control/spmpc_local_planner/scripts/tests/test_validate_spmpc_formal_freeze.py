#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "validate_spmpc_formal_freeze.py"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FormalFreezeValidatorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.root = Path(self.tempdir.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        subprocess.run(["git", "init", "-q", str(self.repo)], check=True)
        subprocess.run(
            ["git", "-C", str(self.repo), "config", "user.email", "test@example.com"],
            check=True,
        )
        subprocess.run(
            ["git", "-C", str(self.repo), "config", "user.name", "Freeze Test"],
            check=True,
        )

        def artifact(relative: str, content: str = "frozen artifact\n") -> Path:
            path = self.repo / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
            return path

        self.artifact = artifact
        self.upstream_files = {
            "experimental_design": artifact("docs/experimental_design.md"),
            "matrix_index": artifact("docs/matrix_index.md"),
            "k6_protocol": artifact("docs/k6_protocol.md"),
            "field_matrix": artifact("docs/field_matrix.md"),
            "field_commands": artifact("docs/field_commands.md"),
        }
        self.release_files = {
            "dynamics": artifact("src/dynamics.cpp"),
            "state_propagation": artifact("src/state_propagation.cpp"),
            "cost_structure": artifact("src/cost_structure.py"),
        }
        self.build_log = artifact("freeze/build.log")
        self.method_files = {
            "baseline": artifact("freeze/configs/B0.yaml"),
            "smooth": artifact("freeze/configs/B_smooth.yaml"),
            "spmpc": artifact("freeze/configs/B_slosh.yaml"),
            "smooth_match": artifact("freeze/configs/SmoothMatch.yaml"),
        }
        self.pilot_evidence_files = {
            "standalone_monitor_config": artifact("freeze/configs/slosh_monitor.yaml"),
            "delta_model_weight_decision_file": artifact("freeze/reports/weight_decision.md"),
            "p3a_endpoint_acceptance_report": artifact("freeze/reports/p3a_acceptance.md"),
            "p4_completion_match_report": artifact("freeze/reports/p4_completion_match.md"),
        }
        self.path_files = {
            "h0": artifact("freeze/paths/H0.json", '{"frame_id":"map","poses":[]}'),
            "h1": artifact("freeze/paths/H1.json", '{"frame_id":"map","poses":[]}'),
            "l1": artifact("freeze/paths/L1.json", '{"frame_id":"map","poses":[]}'),
            "geometry": artifact("freeze/paths/geometry.csv", "path,length\nH1,1.0\n"),
        }
        self.path_file = self.path_files["h1"]
        self.container_yaml = artifact(
            "config/tube_default.yaml",
            "slosh:\n"
            "  container_radius: 0.0185\n"
            "  liquid_height: 0.058\n"
            "  damping_ratio: 0.05\n",
        )
        self.container_c2_yaml = artifact(
            "config/tube_c2.yaml",
            "slosh:\n"
            "  container_radius: 0.025\n"
            "  liquid_height: 0.050\n"
            "  damping_ratio: 0.05\n",
        )
        self.vision_files = {
            "camera": artifact("freeze/calibration/camera.yaml"),
            "calibration": artifact("freeze/calibration/liquid.yaml"),
            "k6_manifest": artifact("freeze/calibration/k6_fidelity.yaml"),
            "k6_smoke": artifact("freeze/reports/k6_smoke.md"),
            "nominal_replay": artifact("freeze/reports/nominal_replay.md"),
        }
        self.randomization_files = {
            "parameter_pilot": artifact("freeze/randomization/parameter.csv"),
            "smooth_match_pilot": artifact("freeze/randomization/smooth_match.csv"),
            "s1_order": artifact("freeze/randomization/s1.csv"),
            "e1_order": artifact("freeze/randomization/e1.csv"),
            "e2_c2_order": artifact("freeze/randomization/e2_c2.csv"),
        }
        self.analysis_files = {
            "rgb": artifact("scripts/rgb.py"),
            "actual_zero_replay": artifact("scripts/actual_zero.py"),
            "k6": artifact("scripts/k6.py"),
            "runtime": artifact("scripts/runtime.py"),
        }
        self.execution_smoke_files = {
            "recorder_warm_start_report": artifact("freeze/reports/warm_start_smoke.md"),
            "actual_zero_replay_report": artifact("freeze/reports/actual_zero_smoke.md"),
        }
        self.validator_script = self.repo / "scripts" / "validate_spmpc_formal_freeze.py"
        self.validator_script.parent.mkdir(parents=True, exist_ok=True)
        self.validator_script.write_bytes(SCRIPT.read_bytes())
        subprocess.run(["git", "-C", str(self.repo), "add", "."], check=True)
        subprocess.run(
            ["git", "-C", str(self.repo), "commit", "-q", "-m", "freeze artifacts"],
            check=True,
        )
        self.head = subprocess.run(
            ["git", "-C", str(self.repo), "rev-parse", "HEAD"],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        ).stdout.strip()
        self.manifest_path = self.root / "freeze_manifest.yaml"

    def manifest(self) -> dict:
        def rel(path: Path) -> str:
            return str(path.relative_to(self.repo))

        randomization = {}
        for key, path in self.randomization_files.items():
            randomization[f"{key}_file"] = rel(path)
            randomization[f"{key}_sha256"] = sha256(path)
        pilot_evidence = {}
        for key, path in self.pilot_evidence_files.items():
            pilot_evidence[key] = rel(path)
            pilot_evidence[f"{key}_sha256"] = sha256(path)
        execution_smoke = {}
        for key, path in self.execution_smoke_files.items():
            execution_smoke[key] = rel(path)
            execution_smoke[f"{key}_sha256"] = sha256(path)

        gates = {
            "parameter_pilot_pass": True,
            "smooth_match_pilot_pass": True,
            "h0_h1_l1_freeze_pass": True,
            "c1_c2_freeze_pass": True,
            "recorder_warm_start_smoke_pass": True,
            "actual_zero_replay_smoke_pass": True,
            "nominal_replay_reproduction_pass": True,
            "visual_sync_smoke_pass": True,
            "k6_fid_v1_0_no_go_check_pass": True,
            "all_formal_prerequisites_pass": True,
        }
        return {
            "protocol_id": "SMPCC-REAL-40-88-v1.0",
            "experimental_design_version": "v1.2",
            "k6_protocol_id": "K6-FID-v1.0",
            "freeze_id": "SMPCC-FREEZE-TEST-001",
            "status": "GO",
            "e4_enabled": False,
            "t_settle_sec": 10.0,
            "upstream_protocols": {
                "experimental_design_file": rel(self.upstream_files["experimental_design"]),
                "experimental_design_sha256": sha256(self.upstream_files["experimental_design"]),
                "matrix_index_file": rel(self.upstream_files["matrix_index"]),
                "matrix_index_sha256": sha256(self.upstream_files["matrix_index"]),
                "k6_protocol_file": rel(self.upstream_files["k6_protocol"]),
                "k6_protocol_sha256": sha256(self.upstream_files["k6_protocol"]),
                "field_matrix_file": rel(self.upstream_files["field_matrix"]),
                "field_matrix_sha256": sha256(self.upstream_files["field_matrix"]),
                "field_commands_file": rel(self.upstream_files["field_commands"]),
                "field_commands_sha256": sha256(self.upstream_files["field_commands"]),
            },
            "method_release": {
                "release_id": "SMPCC-METHOD-v1",
                "dynamics_source": rel(self.release_files["dynamics"]),
                "dynamics_sha256": sha256(self.release_files["dynamics"]),
                "state_propagation_source": rel(self.release_files["state_propagation"]),
                "state_propagation_sha256": sha256(self.release_files["state_propagation"]),
                "cost_structure_source": rel(self.release_files["cost_structure"]),
                "cost_structure_sha256": sha256(self.release_files["cost_structure"]),
                "rotation_consistent_enabled": False,
                "phase_energy_cost_enabled": False,
                "signed_power_enabled": False,
            },
            "software": {
                "git_revision": self.head,
                "git_clean": True,
                "acados_version": "test-acados",
                "codegen_hash": "a" * 64,
                "build_log": rel(self.build_log),
                "build_log_sha256": sha256(self.build_log),
            },
            "methods": {
                "baseline_config": rel(self.method_files["baseline"]),
                "baseline_sha256": sha256(self.method_files["baseline"]),
                "smooth_config": rel(self.method_files["smooth"]),
                "smooth_sha256": sha256(self.method_files["smooth"]),
                "spmpc_config": rel(self.method_files["spmpc"]),
                "spmpc_sha256": sha256(self.method_files["spmpc"]),
                "smooth_match_config": rel(self.method_files["smooth_match"]),
                "smooth_match_sha256": sha256(self.method_files["smooth_match"]),
                "final_w_slosh": 5.0,
                "smooth_match_v_ref": 0.18,
                "smooth_match_safe_v_ref_min": 0.16,
                "smooth_match_safe_v_ref_max": 0.22,
            },
            "pilot_evidence": pilot_evidence,
            "paths": {
                "h0_file": rel(self.path_files["h0"]),
                "h0_sha256": sha256(self.path_files["h0"]),
                "h1_file": rel(self.path_files["h1"]),
                "h1_sha256": sha256(self.path_file),
                "l1_file": rel(self.path_files["l1"]),
                "l1_sha256": sha256(self.path_files["l1"]),
                "geometry_summary_file": rel(self.path_files["geometry"]),
                "geometry_summary_sha256": sha256(self.path_files["geometry"]),
            },
            "containers": {
                "c1_config": "tube_default",
                "c1_config_file": rel(self.container_yaml),
                "c1_config_sha256": sha256(self.container_yaml),
                "c1_radius_m": 0.0185,
                "c1_liquid_height_m": 0.058,
                "c1_damping_ratio": 0.05,
                "c1_freeboard_m": 0.02,
                "c1_f1_hz": 2.0,
                "c1_camera_frames_per_cycle": 15.0,
                "c2_config": "tube_c2",
                "c2_config_file": rel(self.container_c2_yaml),
                "c2_config_sha256": sha256(self.container_c2_yaml),
                "c2_radius_m": 0.025,
                "c2_liquid_height_m": 0.050,
                "c2_damping_ratio": 0.05,
                "c2_freeboard_m": 0.02,
                "c2_f1_hz": 3.0,
                "c2_camera_frames_per_cycle": 10.0,
                "lambda_h": 0.7,
            },
            "vision_and_sync": {
                "camera_serial": "TEST-CAMERA",
                "camera_config_file": rel(self.vision_files["camera"]),
                "camera_config_sha256": sha256(self.vision_files["camera"]),
                "calibration_file": rel(self.vision_files["calibration"]),
                "calibration_sha256": sha256(self.vision_files["calibration"]),
                "tau_cal_sec": 0.01,
                "k6_fidelity_manifest": rel(self.vision_files["k6_manifest"]),
                "k6_fidelity_manifest_sha256": sha256(self.vision_files["k6_manifest"]),
                "k6_protocol_smoke_report": rel(self.vision_files["k6_smoke"]),
                "k6_protocol_smoke_report_sha256": sha256(self.vision_files["k6_smoke"]),
                "nominal_replay_report": rel(self.vision_files["nominal_replay"]),
                "nominal_replay_report_sha256": sha256(self.vision_files["nominal_replay"]),
            },
            "randomization": randomization,
            "analysis_tools": {
                "rgb_script": rel(self.analysis_files["rgb"]),
                "rgb_script_sha256": sha256(self.analysis_files["rgb"]),
                "actual_zero_replay_script": rel(self.analysis_files["actual_zero_replay"]),
                "actual_zero_replay_script_sha256": sha256(
                    self.analysis_files["actual_zero_replay"]
                ),
                "k6_script": rel(self.analysis_files["k6"]),
                "k6_script_sha256": sha256(self.analysis_files["k6"]),
                "runtime_script": rel(self.analysis_files["runtime"]),
                "runtime_script_sha256": sha256(self.analysis_files["runtime"]),
            },
            "runtime_rules": {
                "solve_budget_ms": 33.3333333,
                "solve_budget_metric": "solver_time_ms_overrun_rate",
                "interarrival_metric": "observed_command_intervention_inter_arrival_gap_rate",
                "interarrival_gap_threshold_sec": 0.05,
                "interarrival_window": "full_motion",
                "interarrival_denominator": "expected_control_cycles",
                "strict_control_cycle_deadline_claim_enabled": False,
            },
            "execution_smoke": execution_smoke,
            "manifest_validation": {
                "validator_script": rel(self.validator_script),
                "validator_script_sha256": sha256(self.validator_script),
            },
            "gates": gates,
        }

    def run_validator(
        self,
        manifest: dict,
        *,
        stage: str = "S1",
        group: str = "E2",
        method: str = "Bslosh",
        variant: str = "B_slosh",
        v_ref: str = "0.20",
        w_slosh: str = "5.0",
        read_only: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        if self.manifest_path.exists():
            self.manifest_path.chmod(0o600)
        self.manifest_path.write_text(
            yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
        )
        self.manifest_path.chmod(0o444 if read_only else 0o644)
        return subprocess.run(
            [
                sys.executable,
                str(self.validator_script),
                "--manifest",
                str(self.manifest_path),
                "--repo-root",
                str(self.repo),
                "--stage",
                stage,
                "--group",
                group,
                "--method",
                method,
                "--variant",
                variant,
                "--v-ref",
                v_ref,
                "--w-slosh",
                w_slosh,
                "--path-id",
                "H1",
                "--path-file",
                str(self.path_file),
                "--container-id",
                "C1",
                "--container-config",
                "tube_default",
                "--container-yaml",
                str(self.container_yaml),
                "--container-radius",
                "0.0185",
                "--liquid-height",
                "0.058",
                "--damping-ratio",
                "0.05",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )

    def test_valid_manifest_passes(self) -> None:
        completed = self.run_validator(self.manifest())
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("FORMAL_FREEZE_VALIDATION=PASS", completed.stdout)
        self.assertIn("FREEZE_ID=SMPCC-FREEZE-TEST-001", completed.stdout)
        self.assertIn("T_SETTLE=10", completed.stdout)

    def test_false_gate_fails_closed(self) -> None:
        manifest = self.manifest()
        manifest["gates"]["visual_sync_smoke_pass"] = False
        completed = self.run_validator(manifest)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("FORMAL_FREEZE_VALIDATION=FAIL", completed.stderr)
        self.assertIn("gates.visual_sync_smoke_pass must be boolean true", completed.stderr)

    def test_degenerate_smooth_match_safe_interval_fails(self) -> None:
        manifest = self.manifest()
        manifest["methods"]["smooth_match_safe_v_ref_min"] = 0.18
        manifest["methods"]["smooth_match_safe_v_ref_max"] = 0.18
        completed = self.run_validator(manifest)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must be strictly less than safe_v_ref_max", completed.stderr)

    def test_illegal_stage_path_combination_fails(self) -> None:
        completed = self.run_validator(self.manifest(), stage="S2A", group="E1")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("illegal path for S2A/E1: expected L1, got H1", completed.stderr)

    def test_critical_artifact_hash_mismatch_fails(self) -> None:
        manifest = self.manifest()
        manifest["paths"]["h1_sha256"] = "0" * 64
        completed = self.run_validator(manifest)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("paths.h1_file sha256 mismatch", completed.stderr)

    def test_runtime_w_slosh_mismatch_fails(self) -> None:
        completed = self.run_validator(self.manifest(), w_slosh="2.0")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("runtime w_slosh mismatch", completed.stderr)

    def test_non_candidate_final_w_slosh_fails(self) -> None:
        manifest = self.manifest()
        manifest["methods"]["final_w_slosh"] = 3.0
        completed = self.run_validator(manifest, w_slosh="3.0")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "methods.final_w_slosh must equal one of the frozen v1.0 candidates {1, 2, 5}",
            completed.stderr,
        )

    def test_smooth_match_v_ref_mismatch_fails(self) -> None:
        completed = self.run_validator(
            self.manifest(),
            stage="S1",
            group="E3",
            method="SmoothMatch",
            variant="B_smooth",
            v_ref="0.19",
            w_slosh="0.0",
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("runtime v_ref mismatch", completed.stderr)

    def test_dirty_git_worktree_fails(self) -> None:
        (self.repo / "untracked.txt").write_text("dirty\n", encoding="utf-8")
        completed = self.run_validator(self.manifest())
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Git worktree is not clean", completed.stderr)

    def test_writable_manifest_fails(self) -> None:
        completed = self.run_validator(self.manifest(), read_only=False)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("formal manifest must be read-only", completed.stderr)


if __name__ == "__main__":
    unittest.main()
