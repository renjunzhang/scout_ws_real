"""Fixture-only and negative tests for S5B0 staging materializer v4."""

from __future__ import annotations

import ast
import copy
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5b0_staging_case_materializer_v4 as materializer


class S5B0StagingCaseMaterializerV4Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="r8-s5b0-stage-v4-test-")
        self.base = Path(self.temporary.name)
        self.sources = materializer._write_fixture_sources(self.base / "sources")
        self.policy, self.gauge = materializer.load_frozen_contract()
        self.evidence = materializer.fixture_evidence(self.sources, self.policy)
        self.number = 0

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def next_stage(self) -> Path:
        self.number += 1
        return self.base / f"stage-{self.number}.partial"

    def run_materialize(self, *, mode: str = "FIXTURE", evidence: dict | None = None) -> dict:
        stage = self.next_stage()
        return materializer.materialize(stage, expected_stage_root=stage, sources=self.sources,
            restart_part_index=901, settled_time_s=45.05001991890928, solver_tail_s=1.0,
            mode=mode, evidence=self.evidence if evidence is None else evidence)

    def test_golden_injects_exact_native_swl_and_closed_manifest(self) -> None:
        manifest = self.run_materialize()
        schema = materializer.read_json(materializer.SCHEMA_PATH)
        Draft202012Validator(schema).validate(manifest)
        case = ET.fromstring((Path(manifest["stage_root"]) / "case/C1M_case.xml").read_bytes())
        gauges = case.findall("./execution/special/gauges")
        self.assertEqual(len(gauges), 1)
        self.assertEqual([node.tag for node in gauges[0]], ["swl"] * 16)
        self.assertEqual([node.attrib["name"] for node in gauges[0]], [f"s5b0_p{i:02d}" for i in range(16)])
        for index, node in enumerate(gauges[0]):
            self.assertEqual(node.attrib, {"name":f"s5b0_p{index:02d}", "motionref":"0", "mkbound":"0"})
            self.assertEqual(node.find("savevtkpart").attrib, {"value":"false"})
            self.assertEqual(node.find("output").attrib, {"value":"true"})
            self.assertEqual(node.find("pointdp").attrib, {"value":"0.001"})
        self.assertEqual(manifest["contract"]["expected_raw_gauge_csv"], list(materializer.EXPECTED_CSV))
        self.assertEqual(manifest["evidence"]["result_qc"]["particle_count_required"], 9078)
        self.assertEqual(manifest["evidence"]["result_qc"]["nout_required"], 0)
        self.assertTrue(manifest["evidence"]["result_qc"]["finite_required"])
        self.assertTrue(manifest["evidence"]["result_qc"]["leak_zero_required"])

    def test_render_rejects_existing_gauge_and_wrong_case_identity(self) -> None:
        source = materializer._fixture_case_xml()
        existing = source.replace(b"<parameters />", b"<parameters /><special><gauges /></special>")
        with self.assertRaisesRegex(materializer.StagingMaterializerV4Error, "already contains"):
            materializer.render_case_v4(existing, settled_time_s=45.05001991890928, duration_s=2.0, gauge=self.gauge)
        wrong = source.replace(b'np="9078"', b'np="9077"')
        with self.assertRaisesRegex(materializer.StagingMaterializerV4Error, "9078"):
            materializer.render_case_v4(wrong, settled_time_s=45.05001991890928, duration_s=2.0, gauge=self.gauge)
        wrong = source.replace(b'refmotion="0"', b'refmotion="1"')
        with self.assertRaisesRegex(materializer.StagingMaterializerV4Error, "refmotion"):
            materializer.render_case_v4(wrong, settled_time_s=45.05001991890928, duration_s=2.0, gauge=self.gauge)

    def test_world_fixed_missing_extra_duplicate_and_parameter_drift_fail(self) -> None:
        rendered = materializer.render_case_v4(materializer._fixture_case_xml(), settled_time_s=45.05001991890928, duration_s=2.0, gauge=self.gauge)
        mutations = []
        root = ET.fromstring(rendered); root.find("./execution/special/gauges/swl").attrib.pop("motionref"); mutations.append(root)
        root = ET.fromstring(rendered); root.find("./execution/special/gauges").remove(root.findall("./execution/special/gauges/swl")[-1]); mutations.append(root)
        root = ET.fromstring(rendered); root.findall("./execution/special/gauges/swl")[1].attrib["name"] = "s5b0_p00"; mutations.append(root)
        root = ET.fromstring(rendered); ET.SubElement(root.find("./execution/special/gauges"), "swl", {"name":"extra","motionref":"0","mkbound":"0"}); mutations.append(root)
        root = ET.fromstring(rendered); root.find("./execution/special/gauges/swl/pointdp").attrib["value"] = "0.002"; mutations.append(root)
        root = ET.fromstring(rendered); root.find("./execution/special/gauges/swl/output").attrib["value"] = "false"; mutations.append(root)
        for root in mutations:
            with self.subTest(xml=ET.tostring(root)[:160]):
                with self.assertRaises(materializer.StagingMaterializerV4Error):
                    materializer.validate_gauges(root, self.gauge, 47.05001991890928)

    def test_old_candidate_and_missing_exact_real_authorization_fail_closed(self) -> None:
        old = copy.deepcopy(self.evidence)
        old["candidate"]["sha256"] = materializer.OLD_CANDIDATE_SHA256
        changed_sources = copy.deepcopy(self.sources)
        changed_sources["candidate"]["sha256"] = materializer.OLD_CANDIDATE_SHA256
        with self.assertRaisesRegex(materializer.StagingMaterializerV4Error, "old"):
            materializer.validate_evidence("FIXTURE", old, changed_sources, self.policy)
        with self.assertRaisesRegex(materializer.StagingMaterializerV4Error, "exact authorization"):
            self.run_materialize(mode="REAL")
        drift = copy.deepcopy(self.evidence)
        drift["candidate"]["capability"] = "FIXED_WORLD_GAUGE"
        with self.assertRaisesRegex(materializer.StagingMaterializerV4Error, "capability"):
            materializer.validate_evidence("FIXTURE", drift, self.sources, self.policy)

    def test_transfer_bi4_config_motion_and_qc_drift_fail_closed(self) -> None:
        mutations = (
            ("transfer", "transfer_id", "wrong"),
            ("transfer", "finalized", False),
            ("settled_clone", "particle_count", 9077),
            ("settled_clone", "nout", 1),
            ("settled_clone", "finite", False),
            ("settled_clone", "leak_zero", False),
            ("settled_clone", "refmotion", 1),
            ("result_qc", "raw_gauge_csv_count", 15),
            ("result_qc", "executed_boundary_motion_required", False),
            ("result_qc", "domain_outside_zero_required", False),
        )
        for section, key, value in mutations:
            evidence = copy.deepcopy(self.evidence)
            evidence[section][key] = value
            with self.subTest(section=section, key=key):
                with self.assertRaises(materializer.StagingMaterializerV4Error):
                    materializer.validate_evidence("FIXTURE", evidence, self.sources, self.policy)

    def test_manifest_schema_is_deep_closed_and_claims_fail_closed(self) -> None:
        schema = materializer.read_json(materializer.SCHEMA_PATH)
        Draft202012Validator.check_schema(schema)
        materializer.assert_deep_closed(schema)
        manifest = self.run_materialize()
        changed = copy.deepcopy(manifest)
        changed["claims"]["solver_executed"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)
        changed = copy.deepcopy(manifest)
        changed["contract"]["expected_raw_gauge_csv"].pop()
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

    def test_ast_command_surface_and_self_check_are_static_fixture_only(self) -> None:
        source = Path(materializer.__file__).read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('choices=("self-check",)', source)
        for forbidden in ("import subprocess", "/usr/bin/sudo", "apparmor_parser", "nvidia-smi", "capture.bag"):
            self.assertNotIn(forbidden, source)
        result = materializer.self_check()
        self.assertEqual(result["status"], "PASS_S5B0_STAGING_CASE_MATERIALIZER_V4_FIXTURE_SELF_CHECK")
        self.assertEqual(result["probe_count"], 16)
        self.assertEqual(result["raw_gauge_csv_count"], 16)
        self.assertTrue(result["fixture_root_removed"])
        for key in ("real_external_input_read","real_bag_read","optional_bag_read","candidate_executed","solver_executed","gpu_exposed","sudo_used","network_used","apparmor_loaded","real_staging_authorized"):
            self.assertFalse(result[key], key)


if __name__ == "__main__":
    unittest.main()
