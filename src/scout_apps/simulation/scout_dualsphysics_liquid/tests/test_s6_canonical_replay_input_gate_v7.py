#!/usr/bin/env python3
"""Fixture-only positive and negative tests for canonical S6 input gate v7."""

from __future__ import annotations

import ast
import copy
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_s6_canonical_replay_input_gate_v7.py"
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s6_canonical_replay_input_gate_v7 as gate  # noqa: E402


class CanonicalReplayInputV7Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.times = [1.0, 1.05]
        paths = [
            "Run.csv", "Run.out", "RunPARTs.csv", "data/Part_Head.ibi4",
            "data/PartInfo.ibi4", "data/PartMotionRef.ibi4",
            "data/PartOut_000.obi4", "data/Part_0007.bi4", "data/Part_0008.bi4",
        ]
        identities = {
            relative: {
                "relative_path": relative, "type": "regular", "mode": "0600",
                "size_bytes": index + 1, "device": 1, "inode": index + 1,
                "nlink": 1, "sha256": f"{index + 1:064x}",
            }
            for index, relative in enumerate(paths)
        }
        frames = [
            {
                "index": index, "time_s": time_s, "step": step,
                "runparts_steps_since_previous": interval_steps,
                "particle_count": gate.EXPECTED_PARTICLE_COUNT, "nout": 0,
                "canonical_ids_sha256": gate.CANONICAL_IDS_SHA256,
                "source_order_sha256": gate.CANONICAL_IDS_SHA256,
                "source_order_was_canonical": True,
                "canonical_particle_arrays_sha256": f"{index + 100:064x}",
                "finite": True, "class_counts": dict(gate.EXPECTED_CLASS_COUNTS),
                "identity": identities[f"data/Part_{index:04d}.bi4"],
            }
            for index, time_s, step, interval_steps in (
                (7, self.times[0], 0, 0), (8, self.times[1], 2177, 2178)
            )
        ]
        files = [identities[relative] for relative in paths]
        self.external_inventory_receipt_sha256 = "a" * 64
        self.expected_inventory_sha256 = gate.frame_canonical_sha256(files)
        self.frame = {
            "schema_version": "smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v2",
            "document_type": "SMPCC_R8_LIQUID_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V2",
            "status": "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V2",
            "root": "/fixture/output",
            "contract": {
                "expected_root": "/fixture/output",
                "external_inventory_receipt_sha256": self.external_inventory_receipt_sha256,
                "expected_inventory_sha256": self.expected_inventory_sha256,
                "expected_start_index": 7, "expected_frame_count": len(frames),
                "expected_times_sha256": gate.frame_canonical_sha256(self.times),
                "expected_particle_count": gate.EXPECTED_PARTICLE_COUNT,
                "expected_canonical_ids_sha256": gate.CANONICAL_IDS_SHA256,
                "expected_class_counts": dict(gate.EXPECTED_CLASS_COUNTS),
                "time_tolerance_s": 0.00002,
                "runparts_step_semantics":
                    "RESTART_FIRST_ROW_ZERO_FIRST_INTERVAL_MINUS_ONE_THEN_ACCUMULATE",
            },
            "inventory": {
                "file_count": len(files), "directory_count": 1,
                "canonical_sha256": self.expected_inventory_sha256, "files": files,
            },
            "bindings": {
                "run_files": [identities[name] for name in ("Run.csv", "Run.out", "RunPARTs.csv")],
                "part_head": identities["data/Part_Head.ibi4"],
                "part_info": identities["data/PartInfo.ibi4"],
                "part_motion_ref": identities["data/PartMotionRef.ibi4"],
                "part_out": identities["data/PartOut_000.obi4"],
            },
            "frames": frames,
            "integrity": {
                "external_inventory_receipt_bound": True, "exact_root": True,
                "exact_inventory": True, "no_symlinks": True, "no_hardlinks": True,
                "no_special_files": True, "no_toctou": True,
                "frame_indices_contiguous": True, "frame_times_match_runparts": True,
                "runparts_restart_step_contract": True, "particle_count_stable": True,
                "particle_id_set_canonical": True, "particle_array_order_normalized": True,
                "particle_classes_stable": True, "nout_zero": True, "finite": True,
                "frame_manifest_sha256": gate.frame_canonical_sha256(frames),
            },
            "claims": {
                "reader_is_post_execution": True, "candidate_executed_by_reader": False,
                "solver_executed_by_reader": False, "gpu_exposed": False,
                "network_used": False, "sudo_used": False, "apparmor_loaded": False,
                "real_bag_read": False, "optional_bag_read": False,
            },
        }
        self.frame_bytes = gate.canonical_json(self.frame)
        rows = "time_s,zsurf_m\n" + "".join(f"{value:.17g},{0.058 + index * 1e-6:.17g}\n" for index, value in enumerate(self.times))
        self.gauges = {name: rows.encode() for name in gate.PROBES}

    def admit(self, **changes):
        values = {"frame_manifest_bytes": self.frame_bytes, "frame_manifest_path": "/fixture/frame_manifest.json",
                  "external_inventory_receipt_sha256": self.external_inventory_receipt_sha256,
                  "expected_inventory_sha256": self.expected_inventory_sha256,
                  "gauge_csv_bytes": self.gauges, "attachment_frame": "MOVING_CONTAINER_REFERENCE_REF_0",
                  "probe_grid": gate.expected_probe_grid()}
        values.update(changes)
        return gate.admit(**values)

    def test_valid_canonical_input_binds_native_grid_and_dependencies(self) -> None:
        result = self.admit()
        self.assertEqual("PASS_S6_CANONICAL_REPLAY_INPUT_ADMISSION_V7", result["status"])
        self.assertEqual(16, result["native_gauges"]["probe_count"])
        self.assertEqual(0.0145, result["native_gauges"]["probe_radius_m"])
        self.assertEqual(result["finalized_frames"]["time_grid_sha256"], result["native_gauges"]["time_grid_sha256"])
        self.assertTrue(result["claims"]["optional_not_read"])
        self.assertFalse(result["claims"]["stage6_pass"])

    def test_missing_duplicate_reordered_or_coordinate_attachment_drift_fails(self) -> None:
        cases = []
        missing = dict(self.gauges); missing.pop(gate.PROBES[-1]); cases.append({"gauge_csv_bytes": missing})
        reordered = dict(reversed(list(self.gauges.items()))); cases.append({"gauge_csv_bytes": reordered})
        grid = gate.expected_probe_grid(); grid[1] = {**grid[1], "x_m": grid[1]["x_m"] + 1e-8}; cases.append({"probe_grid": grid})
        cases.append({"attachment_frame": "WORLD"})
        for changes in cases:
            with self.subTest(changes=changes):
                with self.assertRaises(gate.CanonicalReplayInputV7Error): self.admit(**changes)

    def test_time_grid_missing_invalid_and_per_slot_validity_fail(self) -> None:
        wrong = dict(self.gauges)
        lines = wrong[gate.PROBES[0]].decode().splitlines(); fields = lines[2].split(","); fields[0] = "999"; lines[2] = ",".join(fields)
        wrong[gate.PROBES[0]] = ("\n".join(lines) + "\n").encode()
        with self.assertRaisesRegex(gate.CanonicalReplayInputV7Error, "time grid"):
            self.admit(gauge_csv_bytes=wrong)
        for token in ("NA", "nan", "garbage"):
            wrong = dict(self.gauges); lines = wrong[gate.PROBES[0]].decode().splitlines()
            lines[1] = lines[1].split(",")[0] + "," + token; wrong[gate.PROBES[0]] = ("\n".join(lines) + "\n").encode()
            with self.assertRaisesRegex(gate.CanonicalReplayInputV7Error, "invalid ratio"):
                self.admit(gauge_csv_bytes=wrong)

    def test_frame_manifest_particle_id_class_and_hash_drift_fails(self) -> None:
        for mutate in ("ids", "class", "hash"):
            changed = copy.deepcopy(self.frame)
            if mutate == "ids": changed["frames"][1]["canonical_ids_sha256"] = "0" * 64
            elif mutate == "class": changed["frames"][1]["class_counts"]["fluid"] -= 1
            else: changed["integrity"]["frame_manifest_sha256"] = "0" * 64
            with self.subTest(mutate=mutate):
                with self.assertRaises(Exception):
                    self.admit(frame_manifest_bytes=gate.canonical_json(changed))

    def test_v1_receipt_and_inventory_binding_drift_fails(self) -> None:
        changed = copy.deepcopy(self.frame)
        changed["schema_version"] = "smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v1"
        with self.assertRaises(Exception):
            self.admit(frame_manifest_bytes=gate.canonical_json(changed))
        for values in ({"external_inventory_receipt_sha256": "0" * 64},
                       {"expected_inventory_sha256": "0" * 64}):
            with self.assertRaisesRegex(gate.CanonicalReplayInputV7Error, "binding"):
                self.admit(**values)

    def test_public_self_check_has_no_execution_or_discovery_surface(self) -> None:
        report = gate.self_check()
        for key in ("real_solver_output_read", "optional_bag_read", "external_write",
                    "candidate_executed", "solver_or_gpu_executed", "stage6_pass"):
            self.assertFalse(report[key])
        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
        imports.update(node.module.split(".")[0] for node in ast.walk(tree)
                       if isinstance(node, ast.ImportFrom) and node.module)
        self.assertFalse(imports & {"socket", "requests", "subprocess", "rosbag", "rospy"})


if __name__ == "__main__":
    unittest.main()
