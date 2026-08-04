#!/usr/bin/env python3
"""Offline tests for the non-executing formal runtime adapter scaffold.

The fixtures intentionally patch only the already-tested full-freeze/master
validators.  The adapter-specific tests still exercise real file hashes,
H1 replay geometry, C1 parameters, full effective config binding, seed/case
identity and the eight-command ABI.  Nothing in this file starts ROS/Gazebo.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from contextlib import contextmanager
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "smpcc_sim_formal_runtime_adapter.py"


def load_adapter():
    module_name = "smpcc_sim_formal_runtime_adapter_test_target"
    spec = importlib.util.spec_from_file_location(module_name, MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot dynamically import formal runtime adapter")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class FormalRuntimeAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adapter = load_adapter()

    @staticmethod
    def _write_json(path: Path, value) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True), encoding="utf-8")

    @staticmethod
    def _write_text(path: Path, text: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    @staticmethod
    def _sha_file(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    @staticmethod
    def _sha_text(text: str) -> str:
        return hashlib.sha256(text.encode("utf-8")).hexdigest()

    def _bound(self, path: Path, value, path_key: str, hash_key: str):
        self._write_json(path, value)
        return {path_key: str(path.resolve()), hash_key: self._sha_file(path)}

    def _effective_config(self, condition: str, *, legacy_variant: str | None = None):
        config = {
            "condition_id": condition,
            "w_control": 0.3,
            "w_smooth": 1.0,
            "w_alpha": 1.0,
            "w_du_a": 1.0,
            "w_du_vs": 1.0,
            "w_slosh": 0.0,
            "v_ref": 0.2,
            "observer": {"source": "odom", "model": "frozen-nominal"},
            "delay": {"mode": "off", "linear_sec": 0.0, "angular_sec": 0.0},
        }
        if legacy_variant is not None:
            config["runtime_variant"] = legacy_variant
        return config

    def _fixture(self, root: Path, *, condition: str = "Bsmooth", legacy_variant: str | None = None):
        """Create a small, real-hash asset graph for one S1/C1 row.

        It is not a real formal freeze.  The two global validators are mocked
        below so these adapter tests can focus on adapter-owned binding checks;
        all per-asset checks remain the production toolchain functions.
        """
        sim_root = root / "sim"
        ledger_root = sim_root / "formal_ledger"
        ledger_root.mkdir(parents=True)
        assets = root / "assets"

        # The actual launched Gazebo artifact is SDF/XML.  Clearance uses a
        # separate, hash-bound JSON geometry document; the test intentionally
        # refuses the old JSON-as-world aliasing loophole.
        world_path = assets / "open_field.world"
        self._write_text(world_path, "<?xml version='1.0'?><sdf version='1.6'><world name='open_field'/></sdf>\n")
        world_geometry_path = assets / "open_field_clearance_geometry.json"
        self._write_json(
            world_geometry_path,
            {
                "document_type": "SMPCC_SIM_WORLD_CLEARANCE_GEOMETRY",
                "status": "FROZEN",
                "launched_world_hash": self._sha_file(world_path),
                "bounds": [-5.0, 5.0, -5.0, 5.0],
                "obstacles": [],
            },
        )
        map_path = assets / "map.json"
        self._write_json(map_path, {"map_id": "frozen-map"})
        robot_path = assets / "robot.urdf"
        self._write_text(robot_path, "<robot name='frozen'/>")

        zones = {"Z1": [0.0, 0.2], "Z2": [0.2, 0.4], "Z3": [0.4, 0.6], "Z4": [0.6, 0.8], "Z5": [0.8, 1.0]}
        source = {
            "path_id": "H1",
            "source_mode": "frozen_json_replay",
            "points": [{"x": -1.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.0, "yaw": 0.0}],
            "zones": zones,
        }
        source_path = assets / "H1_source.json"
        sim_path = assets / "H1_sim.json"
        self._write_json(source_path, source)
        self._write_json(sim_path, source)
        transform_path = assets / "H1_transform.json"
        self._write_json(transform_path, {"rotation_rad": 0.0, "tx": 0.0, "ty": 0.0, "yaw_offset_rad": 0.0})
        fit_path = assets / "H1_fit_clearance.json"
        self._write_json(fit_path, {"status": "PASS", "report_type": "fixture-only-adapter-test"})

        physical_path = assets / "C1_parameters.json"
        self._write_json(physical_path, {"container_id": "C1", "radius": 0.02})
        container_manifest_path = assets / "C1_manifest.json"
        self._write_json(
            container_manifest_path,
            {
                "report_type": "SMPCC_SIM_CONTAINER_MANIFEST",
                "status": "FROZEN",
                "container_id": "C1",
                "physical_parameter_hash": self._sha_file(physical_path),
            },
        )

        config = self._effective_config(condition, legacy_variant=legacy_variant)
        config_path = assets / f"{condition}_effective_config.json"
        self._write_json(config_path, config)

        plant_code = assets / "independent_plant.py"
        plant_params = assets / "plant_params.json"
        plant_input = assets / "plant_input_schema.json"
        plant_output = assets / "plant_output_schema.json"
        fidelity = assets / "plant_fidelity.json"
        self._write_text(plant_code, "# independent test fixture plant\n")
        self._write_json(plant_params, {"plant": "fixture"})
        self._write_json(plant_input, {"input": "executed_base_motion"})
        self._write_json(plant_output, {"output": "liquid_height"})
        plant_hashes = {
            "plant_code_hash": self._sha_file(plant_code),
            "plant_parameter_hash": self._sha_file(plant_params),
            "plant_input_schema_hash": self._sha_file(plant_input),
            "plant_output_schema_hash": self._sha_file(plant_output),
        }
        self._write_json(
            fidelity,
            {
                "report_type": "SMPCC_SIM_LIQUID_PLANT_FIDELITY_VALIDATION",
                "status": "PASS",
                "formal": True,
                "development_only": False,
                "fidelity_validation_status": "PASS",
                "truth_topic": "/sim_truth/liquid_height",
                "validation_dimensions": {
                    "amplitude": "PASS",
                    "frequency": "PASS",
                    "damping": "PASS",
                    "phase": "PASS",
                    "ranking": "PASS",
                },
                **plant_hashes,
            },
        )
        capability_report = assets / "plant_capability_report.json"
        self._write_json(
            capability_report,
            {
                "report_type": "SMPCC_SIM_INDEPENDENT_LIQUID_PLANT_CAPABILITY",
                "status": "PASS",
                "formal": True,
                "development_only": False,
                "physical_primary_eligible": True,
                "independent_plant": True,
                "implementation_isolated_from_controller": True,
                "controller_hidden_state_access": False,
                "driven_by": "executed_simulated_base_motion",
                "truth_topic": "/sim_truth/liquid_height",
                "fidelity_validation_status": "PASS",
                "fidelity_report_hash": self._sha_file(fidelity),
                **plant_hashes,
            },
        )
        capability = {
            "independent_plant": True,
            "implementation_isolated_from_controller": True,
            "controller_hidden_state_access": False,
            "driven_by": "executed_simulated_base_motion",
            "truth_topic": "/sim_truth/liquid_height",
            "fidelity_validation_status": "PASS",
            "formal": True,
            "development_only": False,
            "physical_primary_eligible": True,
            "plant_code_path": str(plant_code.resolve()),
            "plant_code_hash": plant_hashes["plant_code_hash"],
            "plant_parameter_path": str(plant_params.resolve()),
            "plant_parameter_hash": plant_hashes["plant_parameter_hash"],
            "plant_input_schema_path": str(plant_input.resolve()),
            "plant_input_schema_hash": plant_hashes["plant_input_schema_hash"],
            "plant_output_schema_path": str(plant_output.resolve()),
            "plant_output_schema_hash": plant_hashes["plant_output_schema_hash"],
            "fidelity_report_path": str(fidelity.resolve()),
            "fidelity_report_hash": self._sha_file(fidelity),
            "plant_capability_report_path": str(capability_report.resolve()),
            "plant_capability_report_hash": self._sha_file(capability_report),
        }

        runtime_contract = self._runtime_contract()
        ledger = {
            "ledger_id": "SIM-LEDGER-TEST",
            "ledger_root": str(ledger_root.resolve()),
            "ledger_identity_hash": self._sha_text("ledger identity"),
        }
        freeze = {
            "protocol_id": self.adapter.toolchain.FORMAL_PROTOCOL_ID,
            "sim_freeze_id": "SIM-FREEZE-TEST",
            "git_revision": "0123456789abcdef",
            "build_id": "formal-backend-test-build",
            "paths": {
                "H1": {
                    "source_mode": "frozen_json_replay",
                    "source_path": str(source_path.resolve()),
                    "source_path_hash": self._sha_file(source_path),
                    "sim_path": str(sim_path.resolve()),
                    "sim_path_hash": self._sha_file(sim_path),
                    "transform_path": str(transform_path.resolve()),
                    "transform_hash": self._sha_file(transform_path),
                    "fit_clearance_report_path": str(fit_path.resolve()),
                    "fit_clearance_report_hash": self._sha_file(fit_path),
                    "clearance_m": 0.2,
                }
            },
            "containers": {
                "C1": {
                    "physical_parameter_file": str(physical_path.resolve()),
                    "physical_parameter_hash": self._sha_file(physical_path),
                    "container_manifest_path": str(container_manifest_path.resolve()),
                    "container_manifest_hash": self._sha_file(container_manifest_path),
                }
            },
            "effective_configs": {
                condition: {
                    "effective_config_path": str(config_path.resolve()),
                    "effective_config_file_hash": self._sha_file(config_path),
                    "effective_config": config,
                    "effective_config_hash": self.adapter.toolchain.canonical_hash(config),
                }
            },
            "simulator_assets": {
                "map_file": str(map_path.resolve()),
                "map_hash": self._sha_file(map_path),
                "world_file": str(world_path.resolve()),
                "world_hash": self._sha_file(world_path),
                "world_runtime_format": "gazebo_sdf",
                "world_geometry_file": str(world_geometry_path.resolve()),
                "world_geometry_hash": self._sha_file(world_geometry_path),
                "robot_model_file": str(robot_path.resolve()),
                "robot_model_hash": self._sha_file(robot_path),
            },
            "liquid_plant_capability": capability,
            "dataset_ledger": ledger,
            "runtime_launch_contract": {"fixture": "validated by mock below"},
            "recording_policy": {
                "settle_sec": 30.0,
                "goal_timeout_sec": 60.0,
                "tail_sec": 5.0,
                "post_shutdown_sec": 30.0,
                "recorder_ready_timeout_sec": 15.0,
            },
            "controller_firewall": {"controller_nodes": ["/formal_controller"]},
        }
        freeze["formal_runtime_backend"] = self._runtime_backend(assets, runtime_contract)
        # R8 does not accept a name-only source claim.  The fixture uses the
        # live sim-package registry so the adapter's execution gate exercises
        # the same source/binary/model/codegen binding as a real release.
        source_binding = self.adapter.source_separation.make_source_separation_binding()
        freeze["release_id"] = self.adapter.source_separation.SOURCE_SEPARATED_RELEASE_ID
        freeze["source_separation"] = source_binding

        if condition == "FixedProfile":
            profile_path = assets / "H1_C1_fixed_profile.csv"
            self._write_text(profile_path, "s_normalized,v_ref_m_s\n0.0,0.2\n1.0,0.0\n")
            profile_path.chmod(0o444)
            freeze["fixed_profiles"] = {
                "H1_C1": {
                    "profile_path": str(profile_path.resolve()),
                    "profile_hash": self._sha_file(profile_path),
                    "generator_hash": self._sha_text("frozen generator"),
                    "tracker_config_hash": self._sha_text("frozen tracker"),
                    "constraint_audit_hash": self._sha_text("constraint audit"),
                    "generated_before_run": True,
                    "read_only_replay": True,
                    "runtime_regeneration_forbidden": True,
                }
            }

        seed_bundle = self.adapter.toolchain.make_seed_bundle("formal-adapter-test", "SIM-S1_CORE", 1)
        row = {
            "schema_version": self.adapter.toolchain.SCHEMA_VERSION,
            "protocol_id": self.adapter.toolchain.FORMAL_PROTOCOL_ID,
            "formal": True,
            "evidence_class": "FORMAL_PLANNED_ROWS_NOT_EXECUTED",
            "stage": "SIM-S1_CORE",
            "stage_alias": "S1",
            "planned_row_id": f"SIM-S1_CORE_H1_C1_{condition}_b01",
            "block_id": "b01",
            "order_position": 1,
            "condition_id": condition,
            "method_backend": self.adapter.toolchain.CONDITION_BACKENDS[condition],
            "path_id": "H1",
            "container_id": "C1",
            "planned_block_segment_id": "SIM-S1_CORE_b01_seg01",
            "randomization_table_id": "fixture-table",
            "randomization_hash": self._sha_text("randomization"),
            "seed_bundle_id": seed_bundle["seed_bundle_id"],
            "seed_bundle_hash": seed_bundle["seed_bundle_hash"],
            "fixed_denominator": {"n_plan_stage": 40, "n_plan_condition": 8, "n_block_plan": 8, "n_plan_total": 88},
        }
        row["frozen_asset_hashes"] = self.adapter.toolchain.expected_row_frozen_asset_hashes(freeze, "H1", "C1", condition)

        freeze_path = assets / "formal_freeze.json"
        self._write_json(freeze_path, freeze)
        freeze_hash = self.adapter.toolchain.canonical_hash(freeze)
        master = {
            "formal": True,
            "release_id": self.adapter.source_separation.SOURCE_SEPARATED_RELEASE_ID,
            "source_separation_hash": self.adapter.source_separation.canonical_hash(source_binding),
            "execution_artifact_registry_hash": source_binding[
                "execution_artifact_registry_hash"
            ],
            "master_hash": self._sha_text("formal master identity"),
            "freeze_hash": freeze_hash,
            "formal_freeze_path": str(freeze_path.resolve()),
            "formal_freeze_file_hash": self._sha_file(freeze_path),
            "planned_rows": [row],
            "seed_bundles": {"SIM-S1_CORE:b01": seed_bundle},
        }
        checks = {}
        r8_evidence_root = assets / "r8_source_separation_go_evidence"
        r8_evidence_root.mkdir()
        for name in sorted(self.adapter.source_separation.R8_GO_CHECKS):
            expected = self.adapter.source_separation._expected_go_check(name)
            evidence = r8_evidence_root / f"{name}.log"
            transcript = "returncode=0\n"
            if expected["test_count"]:
                transcript += f"Ran {expected['test_count']} tests in 0.001s\n\nOK\n"
            else:
                transcript += "sim package build passed\n"
            self._write_text(evidence, transcript)
            evidence.chmod(0o444)
            checks[name] = {
                "status": "PASS",
                "command": expected["command"],
                "returncode": 0,
                "test_count": expected["test_count"],
                "test_source": expected["test_source"],
                "evidence": {"path": str(evidence.resolve()), "sha256": self._sha_file(evidence)},
            }
        r8_receipt = self.adapter.source_separation.build_r8_go_receipt(
            freeze, master, checks
        )
        r8_receipt_path = assets / "r8_source_separation_go_receipt.json"
        self._write_json(r8_receipt_path, r8_receipt)
        r8_receipt_path.chmod(0o444)
        freeze["source_separation_go_receipt"] = {
            "path": str(r8_receipt_path.resolve()),
            "sha256": self._sha_file(r8_receipt_path),
        }
        master["source_separation_go_receipt_hash"] = r8_receipt[
            "go_receipt_hash"
        ]
        # The generic formal adapter binds the finalized freeze-file hash,
        # while the R8 GO receipt intentionally binds the pre-GO payload (it
        # excludes its own descriptor to avoid a circular hash).
        self._write_json(freeze_path, freeze)
        freeze_hash = self.adapter.toolchain.canonical_hash(freeze)
        master["freeze_hash"] = freeze_hash
        master["formal_freeze_file_hash"] = self._sha_file(freeze_path)
        master_path = assets / "formal_master.json"
        self._write_json(master_path, master)

        case_dir = ledger_root / row["stage"] / row["block_id"] / f"p01_{condition}" / "r01"
        case_dir.mkdir(parents=True)
        seed_path = case_dir / "seed_bundle.json"
        self._write_json(seed_path, seed_bundle)
        seed_path.chmod(0o444)
        hashes = {
            "map_hash": row["frozen_asset_hashes"]["map_hash"],
            "world_hash": row["frozen_asset_hashes"]["world_hash"],
            "world_geometry_hash": row["frozen_asset_hashes"]["world_geometry_hash"],
            "robot_model_hash": row["frozen_asset_hashes"]["robot_model_hash"],
            "path_hash": row["frozen_asset_hashes"]["sim_path_hash"],
            "physical_parameter_hash": row["frozen_asset_hashes"]["physical_parameter_hash"],
            "effective_config_hash": row["frozen_asset_hashes"]["effective_config_hash"],
            "observer_policy_hash": row["frozen_asset_hashes"]["observer_policy_hash"],
            "delay_policy_hash": row["frozen_asset_hashes"]["delay_policy_hash"],
            "liquid_plant_code_hash": row["frozen_asset_hashes"]["liquid_plant_code_hash"],
            "plant_parameter_hash": row["frozen_asset_hashes"]["plant_parameter_hash"],
            "plant_input_schema_hash": row["frozen_asset_hashes"]["plant_input_schema_hash"],
            "plant_output_schema_hash": row["frozen_asset_hashes"]["plant_output_schema_hash"],
        }
        if condition == "FixedProfile":
            hashes["profile_hash"] = row["frozen_asset_hashes"]["profile_hash"]
            hashes["tracker_config_hash"] = row["frozen_asset_hashes"]["tracker_config_hash"]
        manifest = {
            "formal": True,
            "attempt_id": row["planned_row_id"] + "_r01",
            "planned_row_id": row["planned_row_id"],
            "formal_master_hash": master["master_hash"],
            "formal_freeze_hash": freeze_hash,
            "dataset_root": str(ledger_root.resolve()),
            "dataset_ledger_id": ledger["ledger_id"],
            "dataset_ledger_identity_hash": ledger["ledger_identity_hash"],
            "runtime_launch_contract_id": runtime_contract["contract_id"],
            "runtime_launch_contract_hash": runtime_contract["contract_hash"],
            "seed_bundle_path": str(seed_path.resolve()),
            "seed_bundle_hash": seed_bundle["seed_bundle_hash"],
            "hashes": hashes,
        }
        manifest_path = case_dir / "case_launch_manifest.json"
        self._write_json(manifest_path, manifest)
        environment = {
            "SMPCC_CASE_LAUNCH_MANIFEST_PATH": str(manifest_path.resolve()),
            "SMPCC_CASE_LAUNCH_MANIFEST_SHA256": self._sha_file(manifest_path),
            "SMPCC_SEED_BUNDLE_PATH": str(seed_path.resolve()),
            "SMPCC_SEED_BUNDLE_SHA256": seed_bundle["seed_bundle_hash"],
            "SMPCC_FORMAL_FREEZE_PATH": str(freeze_path.resolve()),
            "SMPCC_FORMAL_FREEZE_FILE_SHA256": self._sha_file(freeze_path),
            "SMPCC_FORMAL_MASTER_PATH": str(master_path.resolve()),
            "SMPCC_FORMAL_MASTER_FILE_SHA256": self._sha_file(master_path),
        }
        return {
            "sim_root": sim_root,
            "ledger_root": ledger_root,
            "environment": environment,
            "freeze": freeze,
            "freeze_path": freeze_path,
            "master": master,
            "master_path": master_path,
            "row": row,
            "manifest": manifest,
            "manifest_path": manifest_path,
            "case_dir": case_dir,
            "seed_path": seed_path,
            "config_path": config_path,
            "runtime_contract": runtime_contract,
            "profile_path": (freeze.get("fixed_profiles", {}).get("H1_C1", {}) or {}).get("profile_path"),
        }

    def _runtime_contract(self):
        commands = {
            field: [sys.executable, str(MODULE_PATH.resolve()), subcommand]
            for field, subcommand in self.adapter.ABI_SUBCOMMANDS.items()
        }
        return {
            "contract_id": "FORMAL-ADAPTER-TEST-CONTRACT",
            "contract_hash": self._sha_text("formal adapter test contract"),
            "commands": commands,
            "ros_master_uri": "127.0.0.1:19530",
            "gazebo_master_uri": "127.0.0.1:19564",
            "startup_timeout_sec": 60.0,
            "command_timeout_sec": 15.0,
            "runtime_ack_schema_hash": self._sha_text("runtime ack schema"),
            "motion_release_ack_schema_hash": self._sha_text("release ack schema"),
            "motion_stop_ack_schema_hash": self._sha_text("stop ack schema"),
            "goal_reached_rule_hash": self._sha_text("goal reached rule"),
        }

    def _runtime_backend(self, assets: Path, runtime_contract):
        """A real-hash delegate release used only for offline adapter tests."""
        delegate = assets / "formal_backend_delegate.py"
        self._write_text(delegate, "#!/usr/bin/env python3\n# offline formal-backend fixture; never run by this test\n")
        delegate.chmod(0o555)
        replay = MODULE_PATH.with_name("smpcc_sim_frozen_path_replay.py").resolve()
        commands = {
            field: [str(delegate.resolve()), subcommand]
            for field, subcommand in self.adapter.ABI_SUBCOMMANDS.items()
        }
        command_file_hashes = {
            field: [{"path": str(delegate.resolve()), "sha256": self._sha_file(delegate)}]
            for field in self.adapter.ABI_SUBCOMMANDS
        }
        backend = {
            "document_type": self.adapter.BACKEND_DOCUMENT_TYPE,
            "status": self.adapter.BACKEND_STATUS,
            "protocol_id": self.adapter.toolchain.FORMAL_PROTOCOL_ID,
            "formal": True,
            "development_only": False,
            "runtime_backend_implemented": True,
            "delegate_via_execve": True,
            "legacy_wrappers_forbidden": True,
            "backend_id": "FORMAL-BACKEND-TEST-R1",
            "sim_freeze_id": "SIM-FREEZE-TEST",
            "git_revision": "0123456789abcdef",
            "build_id": "formal-backend-test-build",
            "runtime_launch_contract_id": runtime_contract["contract_id"],
            "runtime_launch_contract_hash": runtime_contract["contract_hash"],
            "frozen_path_replay": {
                "source_mode": "frozen_json_replay",
                "runtime_generation_forbidden": True,
                "entrypoint_path": str(replay),
                "entrypoint_hash": self._sha_file(replay),
            },
            "effective_config_readback": {
                "required": True,
                "consumed_fields": list(self.adapter.toolchain.REQUIRED_EFFECTIVE_CONFIG_FIELDS),
                "runtime_ack_schema_hash": runtime_contract["runtime_ack_schema_hash"],
            },
            "goal_probe_policy": {
                "exact_terminal_status": "GOAL_REACHED",
                "after_motion_release_required": True,
                "goal_reached_rule_hash": runtime_contract["goal_reached_rule_hash"],
            },
            "motion_stop_policy": {
                "dedicated_cmd_gate": True,
                "zero_hold_required": True,
                "motion_stop_ack_schema_hash": runtime_contract["motion_stop_ack_schema_hash"],
            },
            "lifecycle": {
                "fresh_master_required": True,
                "owned_process_groups_only": True,
                "recorder_before_motion": True,
                "goal_status_exact": "GOAL_REACHED",
                "success_tail_required": True,
                "timeout_tail_required": True,
                "runtime_ack_required": True,
                "motion_release_ack_required": True,
                "motion_stop_ack_required": True,
                "controller_firewall_required": True,
                "broad_process_control_forbidden": True,
                "settle_sec": 30.0,
                "effective_motion_window_sec": 60.0,
                "tail_sec": 5.0,
                "controller_firewall_checkpoints": ["ready", "pre_motion", "postflight"],
            },
            "required_environment": list(self.adapter.BACKEND_REQUIRED_ENVIRONMENT),
            "commands": commands,
            "command_file_hashes": command_file_hashes,
            "case_artifacts": {
                "recorder_artifact": "formal_runtime.bag.active",
                "runtime_ack": "formal_runtime_ack.json",
                "motion_release_ack": "formal_motion_release_ack.json",
                "motion_stop_ack": "formal_motion_stop_ack.json",
            },
        }
        backend["backend_hash"] = self.adapter.toolchain.canonical_hash(backend)
        backend_path = assets / "formal_runtime_backend.json"
        self._write_json(backend_path, backend)
        return {
            "backend_manifest_path": str(backend_path.resolve()),
            "backend_manifest_hash": self._sha_file(backend_path),
            "backend_id": backend["backend_id"],
            "backend_hash": backend["backend_hash"],
        }

    @contextmanager
    def _validating_fixture(self, fixture):
        """Keep the test fixture small while retaining adapter-specific checks."""
        with mock.patch.object(
            self.adapter.toolchain,
            "validate_formal_freeze",
            return_value={"status": "PASS", "errors": []},
        ), mock.patch.object(
            self.adapter.toolchain,
            "validate_master",
            return_value={"status": "PASS", "errors": []},
        ), mock.patch.object(
            self.adapter.toolchain,
            "validate_frozen_runtime_launch_contract",
            return_value=fixture["runtime_contract"],
        ), mock.patch.object(
            self.adapter.toolchain,
            "validate_formal_liquid_plant_capability",
            return_value={
                "eligible": True,
                "status": "PASS",
                "errors": [],
                "truth_topic": "/sim_truth/liquid_height",
            },
        ):
            yield

    def _preflight(self, fixture):
        with self._validating_fixture(fixture):
            return self.adapter.preflight_case(
                environment=fixture["environment"],
                sim_root=fixture["sim_root"],
            )

    def _refresh_r8_source_receipt(self, fixture) -> None:
        """Make a new immutable fixture receipt after a deliberate edit.

        Production evidence is append-only too: a changed freeze cannot reuse
        its old GO receipt.  The test keeps the prior receipt untouched and
        attaches a new hash-bound receipt solely to exercise later adapter
        validation branches.
        """
        previous = Path(fixture["freeze"]["source_separation_go_receipt"]["path"])
        evidence_root = previous.parent / "r8_source_separation_go_evidence"
        checks = {}
        for name in sorted(self.adapter.source_separation.R8_GO_CHECKS):
            expected = self.adapter.source_separation._expected_go_check(name)
            evidence = evidence_root / f"{name}.log"
            checks[name] = {
                "status": "PASS",
                "command": expected["command"],
                "returncode": 0,
                "test_count": expected["test_count"],
                "test_source": expected["test_source"],
                "evidence": {"path": str(evidence), "sha256": self._sha_file(evidence)},
            }
        receipt = self.adapter.source_separation.build_r8_go_receipt(
            fixture["freeze"], fixture["master"], checks
        )
        receipt_path = previous.parent / f"r8_source_separation_go_receipt_{len(list(previous.parent.glob('r8_source_separation_go_receipt*.json'))):02d}.json"
        self._write_json(receipt_path, receipt)
        receipt_path.chmod(0o444)
        fixture["freeze"]["source_separation_go_receipt"] = {
            "path": str(receipt_path),
            "sha256": self._sha_file(receipt_path),
        }
        fixture["master"]["source_separation_go_receipt_hash"] = receipt[
            "go_receipt_hash"
        ]

    def _refresh_freeze_master_case_binding(self, fixture) -> None:
        """Rewrite only fixture bindings after an intentional negative edit."""
        self._refresh_r8_source_receipt(fixture)
        self._write_json(fixture["freeze_path"], fixture["freeze"])
        fixture["environment"]["SMPCC_FORMAL_FREEZE_FILE_SHA256"] = self._sha_file(fixture["freeze_path"])
        freeze_hash = self.adapter.toolchain.canonical_hash(fixture["freeze"])
        fixture["master"]["freeze_hash"] = freeze_hash
        fixture["master"]["formal_freeze_file_hash"] = self._sha_file(fixture["freeze_path"])
        self._write_json(fixture["master_path"], fixture["master"])
        fixture["environment"]["SMPCC_FORMAL_MASTER_FILE_SHA256"] = self._sha_file(fixture["master_path"])
        fixture["manifest"]["formal_freeze_hash"] = freeze_hash
        self._write_json(fixture["manifest_path"], fixture["manifest"])
        fixture["environment"]["SMPCC_CASE_LAUNCH_MANIFEST_SHA256"] = self._sha_file(fixture["manifest_path"])

    def test_describe_declares_all_eight_commands_but_no_implicit_execution(self):
        descriptor = self.adapter.describe()
        self.assertEqual("HASH_BOUND_BACKEND_DISPATCH_IMPLEMENTED_FAIL_CLOSED", descriptor["status"])
        self.assertFalse(descriptor["formal_execution_authorized"])
        self.assertTrue(descriptor["runtime_backend_implemented"])
        self.assertEqual(set(self.adapter.ABI_SUBCOMMANDS), set(descriptor["abi"]))
        self.assertEqual(8, len(descriptor["abi"]))

    def test_incomplete_formal_freeze_is_rejected_without_mocking_the_protocol_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            with self.assertRaisesRegex(self.adapter.AdapterError, "FORMAL_SIM_NO_GO"):
                self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

    def test_preflight_binds_case_master_freeze_seed_assets_and_full_config(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            context = self._preflight(fixture)
            report = self.adapter.preflight_report(context)

        self.assertEqual("FORMAL_RUNTIME_BACKEND_BOUND_NOT_EXECUTED", report["status"])
        self.assertFalse(report["formal_execution_authorized"])
        self.assertTrue(report["can_dispatch_hash_bound_backend"])
        self.assertEqual("Bsmooth", report["condition_id"])
        self.assertIn("w_control", report["effective_config_contract"]["canonical_field_paths"])
        self.assertIn("observer.source", report["effective_config_contract"]["canonical_field_paths"])
        self.assertIn("delay.mode", report["effective_config_contract"]["canonical_field_paths"])
        self.assertIn("sim_path", report["assets"])
        self.assertIn("plant_code", report["assets"])

    def test_prepared_formal_spec_is_deterministic_and_execution_needs_explicit_authority(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            # ``prepare_formal_row`` happens before toolchain creates the case
            # directory/manifest/seed file.  Remove only this temporary test
            # case; no repository or simulation artifact is touched.
            fixture["manifest_path"].unlink()
            fixture["seed_path"].unlink()
            fixture["case_dir"].rmdir()
            with self._validating_fixture(fixture), mock.patch.object(
                self.adapter,
                "DEFAULT_SIM_ROOT",
                fixture["sim_root"],
            ), mock.patch.object(
                self.adapter.toolchain,
                "validate_formal_dataset_ledger",
                return_value={"ledger_root": str(fixture["ledger_root"].resolve())},
            ):
                preparation = self.adapter.prepare_formal_row(
                    formal_freeze_path=fixture["freeze_path"],
                    formal_master_path=fixture["master_path"],
                    planned_row_id=fixture["row"]["planned_row_id"],
                    output_root=fixture["ledger_root"],
                    sim_root=fixture["sim_root"],
                )
                spec = self.adapter.formal_runner_spec(preparation)
                self.assertEqual(30.0, spec["settle_sec"])
                self.assertEqual(60.0, spec["goal_timeout_sec"])
                self.assertGreater(spec["tail_sec"], 0.0)
                self.assertIn("world_geometry_file", spec["assets"])
                self.assertEqual(set(self.adapter.ABI_SUBCOMMANDS), {key for key in spec if key in self.adapter.ABI_SUBCOMMANDS})
                self.assertTrue(spec["runtime_ack_path"].endswith("formal_runtime_ack.json"))
                with self.assertRaisesRegex(self.adapter.AdapterError, "authorize_execution"):
                    self.adapter.execute_prepared_formal_row(preparation)
                with mock.patch.object(self.adapter.toolchain, "run_single_row", return_value={"status": "PASS", "formal": True}) as runner:
                    result = self.adapter.execute_prepared_formal_row(preparation, authorize_execution=True)
                self.assertEqual("PASS", result["status"])
                runner.assert_called_once()

    def test_preflight_rejects_h0_runtime_identity_before_any_runtime_action(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["row"]["path_id"] = "H0"
            self._write_json(fixture["master_path"], fixture["master"])
            fixture["environment"]["SMPCC_FORMAL_MASTER_FILE_SHA256"] = self._sha_file(fixture["master_path"])
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "H0"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

    def test_preflight_rejects_legacy_default_bslosh_config(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary), legacy_variant="B_slosh")
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "legacy default"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

    def test_preflight_rejects_runtime_scurve_and_w5_lineage(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["freeze"]["paths"]["H1"]["source_mode"] = "runtime_s_curve"
            self._refresh_freeze_master_case_binding(fixture)
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "runtime_s_curve"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["freeze"]["rejected_lineage"] = "W5_S10"
            self._refresh_freeze_master_case_binding(fixture)
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "W5/W5_S10"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

    def test_preflight_rejects_tampered_config_file_hash(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            fixture["config_path"].write_text("{}", encoding="utf-8")
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "hash mismatch"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

    def test_fixed_profile_is_resolved_only_from_frozen_read_only_stratum(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary), condition="FixedProfile")
            context = self._preflight(fixture)
            self.assertIn("fixed_profile", context.assets)
            profile_path = Path(str(fixture["profile_path"]))
            profile_path.chmod(0o644)
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "read-only"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

    def test_adapter_abi_rejects_mutable_asset_selector(self):
        contract = self._runtime_contract()
        contract["commands"] = {key: list(value) for key, value in contract["commands"].items()}
        contract["commands"]["launch_command"].extend(["--condition", "Bslosh"])
        with self.assertRaisesRegex(self.adapter.AdapterError, "mutable command-line arguments"):
            self.adapter.validate_adapter_command_abi(contract)

    def test_active_abi_only_execs_the_bound_delegate_and_owned_cleanup_is_process_group_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            context = self._preflight(fixture)
            environment = dict(
                fixture["environment"],
                ROS_MASTER_URI="http://127.0.0.1:19530",
                GAZEBO_MASTER_URI="http://127.0.0.1:19564",
            )
            resolved = self.adapter.resolve_backend_delegate(context, "launch", environment=environment)
            self.assertEqual("launch_command", resolved["field"])
            self.assertTrue(resolved["command"][0].endswith("formal_backend_delegate.py"))
            self.assertEqual(
                str(Path(fixture["manifest_path"]).parent / "formal_runtime.bag.active"),
                resolved["case_artifacts"]["recorder_artifact"],
            )
            observed = {}

            def fake_exec(path, argv, env):
                observed["path"] = path
                observed["argv"] = argv
                observed["env"] = env

            with self.assertRaisesRegex(self.adapter.AdapterError, "exec returned unexpectedly"):
                self.adapter.dispatch_backend_command(context, "launch", environment=environment, executor=fake_exec)
            self.assertEqual(resolved["command"], observed["argv"])
            self.assertEqual(context.backend.backend_hash, observed["env"]["SMPCC_FORMAL_RUNTIME_BACKEND_HASH"])

            command = [sys.executable, str(MODULE_PATH.resolve()), "launch"]
            record = {
                "adapter_id": self.adapter.ADAPTER_ID,
                "case_manifest_hash": context.case_manifest_hash,
                "pid": 12345,
                "process_group_id": 12345,
                "command": command,
                "command_hash": self.adapter.toolchain.canonical_hash(command),
            }
            plan = self.adapter.owned_pid_cleanup_plan([record], context)
            self.assertEqual("adapter-owned process group only", plan[0]["scope"])
            self.assertEqual(["SIGTERM", "SIGKILL_IF_STILL_OWNED_AFTER_TIMEOUT"], plan[0]["signals"])

            record["command"] = ["killall", "rosmaster"]
            record["command_hash"] = self.adapter.toolchain.canonical_hash(record["command"])
            with self.assertRaisesRegex(self.adapter.AdapterError, "forbidden broad"):
                self.adapter.owned_pid_cleanup_plan([record], context)

    def test_backend_and_world_geometry_must_be_real_separate_frozen_assets(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            del fixture["freeze"]["formal_runtime_backend"]
            self._refresh_freeze_master_case_binding(fixture)
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "formal runtime backend"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])

        with tempfile.TemporaryDirectory() as temporary:
            fixture = self._fixture(Path(temporary))
            simulator = fixture["freeze"]["simulator_assets"]
            simulator["world_geometry_file"] = simulator["world_file"]
            simulator["world_geometry_hash"] = simulator["world_hash"]
            self._refresh_freeze_master_case_binding(fixture)
            with self._validating_fixture(fixture):
                with self.assertRaisesRegex(self.adapter.AdapterError, "clearance geometry|separate files"):
                    self.adapter.preflight_case(environment=fixture["environment"], sim_root=fixture["sim_root"])


if __name__ == "__main__":
    unittest.main()
