#!/usr/bin/env python3
"""Static/tempdir-only tests for the recoverable S6 v7 transaction core."""

from __future__ import annotations

import ast
import copy
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "scripts/r8_liquid_s6_real_runtime_transaction_v7.py"
SCHEMA = ROOT / "schema/target_host_s6_real_runtime_transaction_receipt_v7.json"
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s6_real_runtime_transaction_v7 as transaction  # noqa: E402


class S6RealRuntimeTransactionV7Tests(unittest.TestCase):
    def make_case(self, base: Path, suffix: str = "case") -> tuple[transaction.TransactionSpec, dict[str, bytes], dict[str, object], dict[str, object]]:
        root = base / suffix
        root.mkdir()
        final = root / "delivery"
        spec = transaction.TransactionSpec(
            transaction_id=f"s6-v7-primary-{suffix}",
            runtime_contract_sha256="0" * 64,
            expected_previous_ledger_sha256=transaction.ZERO_SHA256,
            partial_root=final.with_name(final.name + ".partial"),
            final_root=final,
            ledger_path=root / "liquid-secondary-ledger.jsonl",
            final_receipt_path=root / "final-transaction-receipt-v7.json",
        )
        runtime_contract = self.runtime_contract(spec)
        contract_sha = transaction.sha256_bytes(transaction.canonical_json(runtime_contract))
        spec = transaction.TransactionSpec(
            transaction_id=spec.transaction_id, runtime_contract_sha256=contract_sha,
            expected_previous_ledger_sha256=spec.expected_previous_ledger_sha256,
            partial_root=spec.partial_root, final_root=spec.final_root,
            ledger_path=spec.ledger_path, final_receipt_path=spec.final_receipt_path)
        artifacts = self.artifacts()
        bundle = self.bundle(runtime_contract, artifacts)
        return spec, artifacts, runtime_contract, bundle

    @staticmethod
    def runtime_contract(spec: transaction.TransactionSpec) -> dict[str, object]:
        identity = {"path":"/fixture/file","sha256":"a"*64,"size_bytes":1,"mode":"0440","device":1,"inode":1,"nlink":1,"mtime_ns":1,"ctime_ns":1}
        window = {"start_index":0,"end_index":3,"start_s":0.0,"end_s":1.5}
        return {"schema_version":"smpcc-r8-liquid-s6-real-runtime-contract-v7","document_type":"SMPCC_R8_LIQUID_S6_REAL_RUNTIME_CONTRACT_V7","status":"ADMITTED_EXACT_FINALIZED_S5B0_PRIMARY_RUNTIME_MATERIALIZED_V7","attempt_id":transaction.ATTEMPT_ID,"planned_denominator":1,"source_outcome":"UNKNOWN","parents":{"policy":identity,"s5a0_selected_bag_receipt":identity,"s5a1_transfer_manifest":identity,"source_bag":identity,"s5b0_result_package":{"root":"/fixture/package","inventory_sha256":"a"*64,"file_count":1,"receipt_sha256":"a"*64,"result_qc_sha256":"a"*64,"checksums_sha256":"a"*64}},"canonical_s5b0":{"frame_manifest":{"identity":identity,"schema_version":"smpcc-r8-liquid-s5b0-finalized-solver-frames-manifest-v2","status":"PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V2","frame_manifest_sha256":"a"*64,"external_inventory_receipt_sha256":"a"*64,"expected_inventory_sha256":"a"*64,"frame_count":4,"particle_count":9078,"canonical_ids_sha256":transaction.CANONICAL_IDS_SHA256,"class_counts":{"fixed_boundary":0,"moving_boundary":2669,"floating":0,"fluid":6409}},"gauge_manifest":{"identity":identity,"schema_version":"smpcc-r8-liquid-s5b0-native-gauge-manifest-v1","status":"PASS_S5B0_NATIVE_GAUGE_MANIFEST_V1","native_gauge_manifest_sha256":"b"*64,"probe_count":16,"attachment_frame":"MOVING_CONTAINER_REFERENCE_REF_0"},"canonical_time_grid":{"sha256":"c"*64,"slot_count":4,"source":"REAL_FINALIZED_FRAME_AND_NATIVE_GAUGE_MANIFESTS","frame_manifest_declares_same":True,"gauge_manifest_declares_same":True},"probe_grid_sha256":"d"*64,"frame_reader":{"relative_path":"scripts/r8_liquid_s5b0_finalized_frame_reader_v2.py","sha256":transaction.FRAME_READER_SHA256},"frame_schema":{"relative_path":"schema/target_host_s5b0_finalized_solver_frames_manifest_v2.json","sha256":transaction.FRAME_SCHEMA_SHA256},"bi4_reader":{"relative_path":"scripts/r8_liquid_bi4_reader_v1.py","sha256":"e"*64},"integrity":{"canonical_manifest_not_flat_replacement":True,"exact_inventory":True,"contiguous_frame_indices":True,"particle_count_stable":True,"particle_ids_stable":True,"particle_classes_stable":True,"nout_zero":True,"finite":True,"no_symlink_hardlink_special_or_toctou":True}},"runtime_paths":{"partial_root":str(spec.partial_root),"final_root":str(spec.final_root),"comparison_ledger":str(spec.ledger_path),"final_transaction_receipt":str(spec.final_receipt_path),"evidence_index":str(spec.final_root/"evidence_index.json"),"checksums":str(spec.final_root/"checksums.sha256")},"analysis_windows":{"materialization_source":"FUTURE_REAL_FINALIZED_TIME_GRID_ONLY","time_grid_sha256":"c"*64,"first15":window,"full_motion":window,"recorded_tail":window,"solver_tail":window},"transaction_admission":{"transaction_id":spec.transaction_id,"expected_previous_ledger_sha256":spec.expected_previous_ledger_sha256,"prepared_then_publish_then_committed_then_receipt":True,"staging_complete_before_prepared":True,"consumer_requires_three_way_consistency":True},"claims":{"stage6_pass":False,"development_only":True,"physical_reference_pending":True,"physical_fidelity_validated":False,"paired_ranking":False,"cross_method_ranking":False,"selected_trajectory_cpu_comparison":False,"formal":False,"production":False,"physical_primary":False}}

    @staticmethod
    def artifacts() -> dict[str, bytes]:
        import io
        from PIL import Image

        def encoded_image(color: tuple[int, int, int]) -> bytes:
            stream = io.BytesIO()
            Image.new("RGB", (16, 12), color).save(stream, format="PNG")
            return stream.getvalue()

        png = encoded_image((0, 114, 178)); gray = encoded_image((96, 96, 96))
        keyframes = [encoded_image((40 + index * 50, 80, 140)) for index in range(3)]
        result = {
            "animation/primary.mp4": b"fixture-mp4-bounded-parser-tested-separately\n",
            "animation/primary_preview.gif": b"GIF89a-fixture-bounded-parser-tested-separately\n",
            "data/metrics.csv": b"surface,secondary,amplitude_error_m\nH_crest,H_proxy,0.0\n",
            "data/metrics.json": transaction.canonical_json({"fixture": "metrics"}),
            "data/selected_signals.json": transaction.canonical_json({"fixture": "selected"}),
            "data/surface_timeseries.csv": b"time_s,H_crest_m\n0.0,0.0\n",
            "data/surface_timeseries.json": transaction.canonical_json({"fixture": "surface"}),
            "figures/primary_shared_x_timeseries.pdf": b"%PDF-1.4\n% fixture\n",
            "figures/primary_shared_x_timeseries.png": png,
            "figures/primary_shared_x_timeseries.svg": b'<svg xmlns="http://www.w3.org/2000/svg" width="16" height="12"></svg>\n',
            "figures/primary_shared_x_timeseries_grayscale.png": gray,
            "keyframes/primary_first.png": keyframes[0],
            "keyframes/primary_middle.png": keyframes[1],
            "keyframes/primary_last.png": keyframes[2],
        }
        analysis = {"schema_version": "smpcc-r8-liquid-s6-primary-analysis-v7",
                    "status": "PASS_S6_PRIMARY_ANALYSIS_V7", "attempt_id": transaction.ATTEMPT_ID,
                    "planned_denominator": 1, "source_outcome": "UNKNOWN", "fixture": True}
        result["reports/analysis_result.json"] = transaction.canonical_json(analysis)
        result["reports/eda_report.json"] = transaction.canonical_json({"schema_version": "fixture-eda"})
        comparison = {"schema_version": "smpcc-r8-liquid-s6-comparison-manifest-v7",
            "status": "S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY",
            "attempt_id": transaction.ATTEMPT_ID, "planned_denominator": 1,
            "source_outcome": "UNKNOWN", "compared_secondaries": ["H_proxy", "H_modal"],
            "paired_ranking": False, "cross_method_ranking": False,
            "selected_trajectory_cpu_comparison": False, "physical_reference_pending": True,
            "physical_fidelity_validated": False, "formal": False, "production": False}
        result["comparison_manifest.json"] = transaction.canonical_json(comparison)
        figure_files = ("figures/primary_shared_x_timeseries.png",
                        "figures/primary_shared_x_timeseries.pdf",
                        "figures/primary_shared_x_timeseries.svg",
                        "figures/primary_shared_x_timeseries_grayscale.png")
        figure = {"schema_version": "smpcc-r8-liquid-s6-figure-manifest-v7",
            "source_analysis_sha256": transaction.sha256_json(analysis),
            "layout": "THREE_VERTICAL_SHARED_X_PANELS", "dual_y_axes": False,
            "palette": "OKABE_ITO", "redundant_line_styles": True,
            "formats": ["PNG", "PDF", "SVG", "GRAYSCALE_PNG"],
            "artifacts": {name: {"sha256": transaction.sha256_bytes(result[name]),
                "size_bytes": len(result[name])} for name in figure_files},
            "qa": {"color_render_pass": True, "grayscale_render_pass": True,
                "svg_render_pass": True, "no_clipping": True, "no_missing_glyphs": True,
                "no_dual_y_axis": True, "source_data_hash_bound": True,
                "multimodal_visual_review": True}}
        result["reports/figure_manifest.json"] = transaction.canonical_json(figure)
        visual = {"schema_version": "smpcc-r8-liquid-s6-multimodal-visual-qa-v7",
            "status": "PASS_S6_MULTIMODAL_VISUAL_QA_V7",
            "reviewed_preview_sha256": transaction.sha256_bytes(png),
            "reviewed_grayscale_sha256": transaction.sha256_bytes(gray),
            "no_clipping": True, "no_missing_glyphs": True, "no_legend_occlusion": True,
            "panel_alignment": True, "grayscale_distinguishable": True,
            "data_not_visually_clipped": True, "cross_panel_units_consistent": True}
        result["reports/visual_qa.json"] = transaction.canonical_json(visual)
        result["reports/quality_control.json"] = transaction.canonical_json({
            "canonical_grid": True, "valid_probes_per_slot": 16, "optional_unread": True,
            "PHYSICAL_REFERENCE_PENDING": True, "visual_qa_programmatic": True,
            "visual_qa_human_pending": False})
        media_files = ("animation/primary.mp4", "animation/primary_preview.gif",
                       "keyframes/primary_first.png", "keyframes/primary_middle.png",
                       "keyframes/primary_last.png")
        rendered_hashes = [f"{index + 1:064x}" for index in range(3)]
        media = {"schema_version": "smpcc-r8-liquid-s6-media-manifest-v7",
            "attempt_id": transaction.ATTEMPT_ID, "source_frame_manifest_sha256": "a" * 64,
            "frames": [{"index": index, "time_s": index / 10,
                "source_bi4_sha256": f"{index + 4:064x}",
                "rendered_png_sha256": rendered_hashes[index], "probe_grid_sha256": "b" * 64,
                "attachment_frame": "MOVING_CONTAINER_REFERENCE_REF_0"} for index in range(3)],
            "fps": 10, "frame_count": 3, "duration_s": .3, "decoded_mp4_fps": 10.0,
            "decoded_mp4_duration_s": .3, "decoded_gif_duration_s": .3,
            "keyframes": {name: {"sha256": transaction.sha256_bytes(result[name]),
                "source_index": index, "source_rendered_png_sha256": rendered_hashes[index]}
                for name, index in (("keyframes/primary_first.png", 0),
                                    ("keyframes/primary_middle.png", 1),
                                    ("keyframes/primary_last.png", 2))},
            "artifacts": {name: {"sha256": transaction.sha256_bytes(result[name]),
                "size_bytes": len(result[name])} for name in media_files},
            "numeric_fact_source": False}
        result["reports/media_manifest.json"] = transaction.canonical_json(media)
        evidence_paths = sorted(result)
        evidence = {"schema_version": "smpcc-r8-liquid-s6-evidence-index-v7",
            "attempt_id": transaction.ATTEMPT_ID, "planned_denominator": 1,
            "source_outcome": "UNKNOWN", "entries": [{"relative_path": name,
                "sha256": transaction.sha256_bytes(result[name]), "size_bytes": len(result[name])}
                for name in evidence_paths],
            "excluded_self_referential_paths": ["checksums.sha256", "evidence_index.json"],
            "optional_unread": True, "physical_reference_pending": True}
        result["evidence_index.json"] = transaction.canonical_json(evidence)
        result["checksums.sha256"] = "".join(
            f"{transaction.sha256_bytes(result[name])}  {name}\n" for name in sorted(result)
        ).encode("ascii")
        return result

    @staticmethod
    def bundle(contract: dict[str, object], artifacts: dict[str, bytes]) -> dict[str, object]:
        normal=transaction._normalise_artifacts(artifacts);inventory=transaction._expected_inventory(normal)
        c=contract["canonical_s5b0"]
        quality={"analysis_status":"PASS_S6_PRIMARY_ANALYSIS_V7","figure_manifest_sha256":transaction.sha256_bytes(normal["reports/figure_manifest.json"]),"media_manifest_sha256":transaction.sha256_bytes(normal["reports/media_manifest.json"]),"visual_qa_sha256":transaction.sha256_bytes(normal["reports/visual_qa.json"]),"comparison_manifest_sha256":transaction.sha256_bytes(normal["comparison_manifest.json"]),"evidence_index_sha256":transaction.sha256_bytes(normal["evidence_index.json"]),"checksums_sha256":transaction.sha256_bytes(normal["checksums.sha256"]),"figure_formats":["PNG","PDF","SVG","GRAYSCALE_PNG"],"keyframe_count":3,"mp4_complete_decode":True,"gif_complete_decode":True,"media_timing_verified":True,"programmatic_visual_qa":True,"multimodal_visual_review":True,"no_clipping":True,"no_missing_glyphs":True,"no_legend_occlusion":True,"panel_alignment":True,"grayscale_distinguishable":True,"no_dual_y_axis":True}
        return {"schema_version":"smpcc-r8-liquid-s6-real-runtime-artifact-bundle-v7","document_type":"SMPCC_R8_LIQUID_S6_REAL_RUNTIME_ARTIFACT_BUNDLE_V7","status":"S6_PRIMARY_ARTIFACT_BUNDLE_PRECOMMIT_ADMISSION_PASS_V7","attempt_id":transaction.ATTEMPT_ID,"planned_denominator":1,"source_outcome":"UNKNOWN","contract":{"sha256":transaction.sha256_bytes(transaction.canonical_json(contract)),"size_bytes":len(transaction.canonical_json(contract))},"canonical_inputs":{"finalized_frame_manifest":{"schema_version":c["frame_manifest"]["schema_version"],"status":c["frame_manifest"]["status"],"content_sha256":c["frame_manifest"]["frame_manifest_sha256"]},"external_inventory_receipt_sha256":c["frame_manifest"]["external_inventory_receipt_sha256"],"expected_inventory_sha256":c["frame_manifest"]["expected_inventory_sha256"],"canonical_ids_sha256":transaction.CANONICAL_IDS_SHA256,"particle_count":9078,"class_counts":c["frame_manifest"]["class_counts"],"native_gauge_manifest":{"schema_version":c["gauge_manifest"]["schema_version"],"status":c["gauge_manifest"]["status"],"content_sha256":c["gauge_manifest"]["native_gauge_manifest_sha256"],"probe_count":16,"attachment_frame":"MOVING_CONTAINER_REFERENCE_REF_0"},"time_grid_sha256":c["canonical_time_grid"]["sha256"],"probe_grid_sha256":c["probe_grid_sha256"],"frame_reader_sha256":transaction.FRAME_READER_SHA256,"frame_schema_sha256":transaction.FRAME_SCHEMA_SHA256,"bi4_reader_sha256":c["bi4_reader"]["sha256"]},"required_artifacts":list(normal),"inventory":inventory,"inventory_sha256":transaction.sha256_json(inventory),"quality":quality,"claims":{"stage6_pass":False,"development_only":True,"physical_reference_pending":True,"physical_fidelity_validated":False,"paired_ranking":False,"cross_method_ranking":False,"selected_trajectory_cpu_comparison":False,"formal":False,"production":False,"physical_primary":False}}

    @staticmethod
    def ledger(spec: transaction.TransactionSpec) -> list[dict[str, object]]:
        if not spec.ledger_path.exists():
            return []
        return [json.loads(line) for line in spec.ledger_path.read_bytes().splitlines()]

    def assert_final_pass(self, report: dict[str, object]) -> None:
        self.assertEqual("COMMITTED_RECEIPT_CONSISTENT", report["status"])
        self.assertTrue(report["consumer_acceptance"]["accepted"])
        self.assertTrue(report["consumer_acceptance"]["three_way_consistent"])
        self.assertTrue(report["claims"]["stage6_pass"])
        self.assertTrue(report["claims"]["physical_reference_pending"])
        self.assertFalse(report["claims"]["physical_fidelity_validated"])
        self.assertFalse(report["claims"]["paired_ranking"])
        self.assertFalse(report["claims"]["cross_method_ranking"])

    @staticmethod
    def execute(case, *, fail_after=None):
        spec, artifacts, contract, bundle = case
        return transaction.execute_transaction(spec, artifacts, contract, bundle,
                                               fail_after=fail_after)

    @staticmethod
    def consume(case):
        spec, artifacts, contract, bundle = case
        return transaction.consume_transaction(spec, artifacts, contract, bundle)

    def test_schema_static_self_check_and_no_execution_surface(self) -> None:
        schema = json.loads(SCHEMA.read_bytes())
        Draft202012Validator.check_schema(schema)
        transaction.assert_deep_closed(schema)
        report = transaction.self_check()
        self.assertIn("NOT_ADMITTED", report["status"])
        self.assertFalse(report["receipt_hash_self_reference"])
        for key in (
            "external_root_materialized",
            "real_bag_read",
            "optional_bag_read",
            "candidate_executed",
            "solver_or_gpu_executed",
            "sudo_used",
            "network_used",
            "stage6_pass",
        ):
            self.assertFalse(report[key])

        tree = ast.parse(SCRIPT.read_text(encoding="utf-8"))
        imports = {
            alias.name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        }
        imports.update(
            node.module.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        )
        self.assertFalse(imports & {"socket", "requests", "subprocess", "rosbag", "rospy"})
        execution_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in {"system", "popen", "run", "Popen", "execv", "execve"}
        ]
        self.assertEqual([], execution_calls)

    def test_runtime_and_artifact_schemas_are_closed_and_pin_v2_claim_ceiling(self) -> None:
        for path in (transaction.RUNTIME_CONTRACT_SCHEMA_PATH,
                     transaction.ARTIFACT_BUNDLE_SCHEMA_PATH):
            schema = json.loads(path.read_bytes())
            Draft202012Validator.check_schema(schema); transaction.assert_deep_closed(schema)
        bundle_schema = json.loads(transaction.ARTIFACT_BUNDLE_SCHEMA_PATH.read_bytes())
        canonical = bundle_schema["$defs"]["canonicalInputs"]["properties"]
        self.assertEqual(transaction.FRAME_READER_SHA256, canonical["frame_reader_sha256"]["const"])
        self.assertEqual(transaction.FRAME_SCHEMA_SHA256, canonical["frame_schema_sha256"]["const"])
        claims = bundle_schema["$defs"]["claims"]["properties"]
        for name in ("paired_ranking", "cross_method_ranking",
                     "selected_trajectory_cpu_comparison", "physical_fidelity_validated"):
            self.assertFalse(claims[name]["const"])

    def test_clean_transaction_orders_witnesses_and_consumer_is_idempotent(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            case = self.make_case(Path(temporary)); spec, artifacts, contract, bundle = case
            report = self.execute(case)
            self.assert_final_pass(report)
            self.assertFalse(spec.partial_root.exists())
            self.assertTrue(spec.final_root.is_dir())
            entries = self.ledger(spec)
            self.assertEqual(["PREPARED", "COMMITTED"], [row["payload"]["phase"] for row in entries])
            self.assertTrue(all(row["payload"]["stage6_pass"] is False for row in entries))
            self.assertEqual(entries[0]["entry_sha256"], entries[1]["previous_entry_sha256"])
            self.assertEqual(0o600, stat.S_IMODE(spec.ledger_path.stat().st_mode))
            self.assertEqual(0o440, stat.S_IMODE(spec.final_receipt_path.stat().st_mode))
            second = self.execute(case)
            self.assertEqual(report, second)
            self.assertEqual(2, len(self.ledger(spec)))

    def test_every_durable_failure_boundary_recovers_without_duplicate_entries(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            for index, point in enumerate(sorted(transaction.FAILURE_POINTS)):
                with self.subTest(point=point):
                    case = self.make_case(base, f"fault-{index}-{point}"); spec, artifacts, contract, bundle = case
                    with self.assertRaisesRegex(transaction.InjectedFailure, point):
                        self.execute(case, fail_after=point)
                    before = self.ledger(spec)
                    self.assertTrue(all(row["payload"]["stage6_pass"] is False for row in before))
                    if point == "after_staging":
                        self.assertEqual([], before)
                        self.assertTrue(spec.partial_root.is_dir())
                    elif point == "after_prepared":
                        self.assertEqual(["PREPARED"], [row["payload"]["phase"] for row in before])
                        self.assertTrue(spec.partial_root.is_dir())
                    elif point in {"after_publish", "after_revalidate"}:
                        self.assertEqual(["PREPARED"], [row["payload"]["phase"] for row in before])
                        self.assertTrue(spec.final_root.is_dir())
                    else:
                        self.assertEqual(["PREPARED", "COMMITTED"], [row["payload"]["phase"] for row in before])
                    recovered = self.execute(case)
                    self.assert_final_pass(recovered)
                    self.assertEqual(2, len(self.ledger(spec)))

    def test_prepared_without_commit_and_published_without_commit_are_not_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            prepared_case = self.make_case(base, "prepared-only")
            prepared_spec, prepared_artifacts, prepared_contract, prepared_bundle = prepared_case
            with self.assertRaises(transaction.InjectedFailure):
                self.execute(prepared_case, fail_after="after_prepared")
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "exact final root"):
                self.consume(prepared_case)

            published_case = self.make_case(base, "published-only")
            published_spec, published_artifacts, published_contract, published_bundle = published_case
            with self.assertRaises(transaction.InjectedFailure):
                self.execute(published_case, fail_after="after_publish")
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "PREPARED and COMMITTED"):
                self.consume(published_case)

    def test_committed_without_receipt_is_not_accepted_then_recovers_by_receipt_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            case = self.make_case(Path(temporary), "committed-only"); spec, artifacts, contract, bundle = case
            with self.assertRaises(transaction.InjectedFailure):
                self.execute(case, fail_after="after_committed")
            self.assertEqual(["PREPARED", "COMMITTED"], [row["payload"]["phase"] for row in self.ledger(spec)])
            self.assertFalse(spec.final_receipt_path.exists())
            with self.assertRaises(FileNotFoundError):
                self.consume(case)
            recovered = self.execute(case)
            self.assert_final_pass(recovered)
            self.assertEqual(2, len(self.ledger(spec)))

    def test_rename_noreplace_collision_and_public_collision_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            source = base / "source"
            destination = base / "destination"
            source.mkdir()
            destination.mkdir()
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "collision"):
                transaction._rename_noreplace(source, destination)
            self.assertTrue(source.is_dir())
            self.assertTrue(destination.is_dir())

            case = self.make_case(base, "race"); spec, artifacts, contract, bundle = case
            with self.assertRaises(transaction.InjectedFailure):
                self.execute(case, fail_after="after_prepared")
            spec.final_root.mkdir()
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "both partial and final"):
                self.execute(case)
            self.assertEqual(["PREPARED"], [row["payload"]["phase"] for row in self.ledger(spec)])

    def test_final_tree_receipt_ledger_and_predecessor_drift_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            case = self.make_case(base, "tree-drift"); spec, artifacts, contract, bundle = case
            self.execute(case)
            target = spec.final_root / "data/surface_timeseries.csv"
            os.chmod(target, 0o640)
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "inventory"):
                self.consume(case)

            case = self.make_case(base, "receipt-drift"); spec, artifacts, contract, bundle = case
            self.execute(case)
            os.chmod(spec.final_receipt_path, 0o640)
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "receipt"):
                self.consume(case)

            case = self.make_case(base, "truncated-ledger"); spec, artifacts, contract, bundle = case
            with self.assertRaises(transaction.InjectedFailure):
                self.execute(case, fail_after="after_prepared")
            with spec.ledger_path.open("ab") as stream:
                stream.write(b'{"truncated":')
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "truncated"):
                self.execute(case)

            case = self.make_case(base, "stale-predecessor"); spec, artifacts, contract, bundle = case
            stale = transaction.TransactionSpec(
                transaction_id=spec.transaction_id,
                runtime_contract_sha256=spec.runtime_contract_sha256,
                expected_previous_ledger_sha256="f" * 64,
                partial_root=spec.partial_root,
                final_root=spec.final_root,
                ledger_path=spec.ledger_path,
                final_receipt_path=spec.final_receipt_path,
            )
            stale_contract = copy.deepcopy(contract)
            stale_contract["runtime_paths"]["partial_root"] = str(stale.partial_root)
            stale_contract["runtime_paths"]["final_root"] = str(stale.final_root)
            stale_contract["runtime_paths"]["comparison_ledger"] = str(stale.ledger_path)
            stale_contract["runtime_paths"]["final_transaction_receipt"] = str(stale.final_receipt_path)
            stale_contract["runtime_paths"]["evidence_index"] = str(stale.final_root / "evidence_index.json")
            stale_contract["runtime_paths"]["checksums"] = str(stale.final_root / "checksums.sha256")
            stale_contract["transaction_admission"]["expected_previous_ledger_sha256"] = "f" * 64
            stale = transaction.TransactionSpec(transaction_id=stale.transaction_id,
                runtime_contract_sha256=transaction.sha256_bytes(transaction.canonical_json(stale_contract)),
                expected_previous_ledger_sha256=stale.expected_previous_ledger_sha256,
                partial_root=stale.partial_root, final_root=stale.final_root,
                ledger_path=stale.ledger_path, final_receipt_path=stale.final_receipt_path)
            stale_bundle = self.bundle(stale_contract, artifacts)
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "stale"):
                transaction.execute_transaction(stale, artifacts, stale_contract, stale_bundle)
            self.assertFalse(stale.final_root.exists())

    def test_precommit_rejects_arbitrary_text_and_claim_or_hash_drift_before_staging(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            case = self.make_case(Path(temporary), "precommit-negative")
            spec, artifacts, contract, bundle = case
            arbitrary = {"comparison_manifest.json": b"{}\n"}
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "mandatory"):
                transaction.execute_transaction(spec, arbitrary, contract, bundle)
            self.assertFalse(spec.partial_root.exists())

            coherent_text = {name: f"coherent-fake:{name}\n".encode()
                             for name in artifacts if name != "checksums.sha256"}
            coherent_text["checksums.sha256"] = "".join(
                f"{transaction.sha256_bytes(coherent_text[name])}  {name}\n"
                for name in sorted(coherent_text)).encode()
            coherent_bundle = self.bundle(contract, coherent_text)
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "invalid JSON"):
                transaction.execute_transaction(spec, coherent_text, contract, coherent_bundle)
            self.assertFalse(spec.partial_root.exists())

    def test_semantic_manifest_or_evidence_drift_rejected_before_staging(self) -> None:
        mutations = {
            "evidence_missing": lambda value: value["entries"].pop(),
            "evidence_wrong_size": lambda value: value["entries"][0].__setitem__("size_bytes", 1),
            "figure_dual_axis": lambda value: value.__setitem__("dual_y_axes", True),
            "media_wrong_keyframe": lambda value: value["keyframes"]["keyframes/primary_first.png"].__setitem__("source_index", 1),
            "visual_review_pending": lambda value: value.__setitem__("status", "PENDING"),
        }
        targets = {"evidence_missing": "evidence_index.json", "evidence_wrong_size": "evidence_index.json",
                   "figure_dual_axis": "reports/figure_manifest.json",
                   "media_wrong_keyframe": "reports/media_manifest.json",
                   "visual_review_pending": "reports/visual_qa.json"}
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            for index, (name, mutate) in enumerate(mutations.items()):
                with self.subTest(name=name):
                    spec, artifacts, contract, _bundle = self.make_case(base, f"semantic-{index}")
                    changed = dict(artifacts); path = targets[name]
                    value = json.loads(changed[path]); mutate(value)
                    changed[path] = transaction.canonical_json(value)
                    changed["checksums.sha256"] = "".join(
                        f"{transaction.sha256_bytes(changed[item])}  {item}\n"
                        for item in sorted(set(changed) - {"checksums.sha256"})).encode()
                    bundle = self.bundle(contract, changed)
                    with self.assertRaises(transaction.S6TransactionV7Error):
                        transaction.execute_transaction(spec, changed, contract, bundle)
                    self.assertFalse(spec.partial_root.exists())
            changed = copy.deepcopy(bundle); changed["claims"]["cross_method_ranking"] = True
            with self.assertRaises(Exception):
                transaction.execute_transaction(spec, artifacts, contract, changed)
            self.assertFalse(spec.partial_root.exists())
            changed = copy.deepcopy(bundle); changed["inventory"][0]["sha256"] = "0" * 64
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "byte inventory"):
                transaction.execute_transaction(spec, artifacts, contract, changed)
            self.assertFalse(spec.partial_root.exists())

    def test_receipt_has_no_hash_self_reference_and_consumer_hash_is_external(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            case = self.make_case(Path(temporary), "no-self-reference"); spec, artifacts, contract, bundle = case
            report = self.execute(case)
            stored = json.loads(spec.final_receipt_path.read_bytes())
            self.assertNotIn("sha256", stored)
            self.assertNotIn("receipt_sha256", stored)
            self.assertEqual(
                transaction.sha256_bytes(spec.final_receipt_path.read_bytes()),
                report["receipt"]["sha256"],
            )
            changed = dict(stored)
            changed["committed_entry_sha256"] = "0" * 64
            encoded = transaction.canonical_json(changed)
            os.chmod(spec.final_receipt_path, 0o640)
            spec.final_receipt_path.write_bytes(encoded)
            os.chmod(spec.final_receipt_path, 0o440)
            with self.assertRaisesRegex(transaction.S6TransactionV7Error, "binding"):
                self.consume(case)

    def test_commit_append_failure_leaves_published_prepared_recoverable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            case = self.make_case(Path(temporary), "commit-write-failure"); spec, artifacts, contract, bundle = case
            original = transaction._append_locked
            count = 0

            def fail_second(descriptor: int, entry: dict[str, object]) -> None:
                nonlocal count
                count += 1
                if count == 2:
                    raise OSError("fixture commit append failure")
                original(descriptor, entry)

            with mock.patch.object(transaction, "_append_locked", side_effect=fail_second):
                with self.assertRaisesRegex(OSError, "commit append"):
                    self.execute(case)
            self.assertTrue(spec.final_root.is_dir())
            self.assertEqual(["PREPARED"], [row["payload"]["phase"] for row in self.ledger(spec)])
            report = self.execute(case)
            self.assert_final_pass(report)


if __name__ == "__main__":
    unittest.main()
