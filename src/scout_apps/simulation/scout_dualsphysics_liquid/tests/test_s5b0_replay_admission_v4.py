"""Static and negative tests for create-new S5B0 admission v4."""

from __future__ import annotations

import copy
import importlib.util
import json
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent


def load(name: str, filename: str):
    path = ROOT / "scripts" / filename
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gate = load("s5b0_admission_v4_test", "r8_liquid_s5b0_replay_admission_gate_v4.py")
profile = load("s5b0_profile_v4_test", "r8_liquid_s5b0_profile_generator_v4.py")


class S5B0ReplayAdmissionV4Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.policy = json.loads(gate.POLICY_PATH.read_text(encoding="utf-8"))

    def test_self_check_is_finalized_input_but_not_admitted(self) -> None:
        value = gate.self_check()
        self.assertEqual(value["status"], "NOT_ADMITTED_MOTION_ATTACHED_GAUGE_CANDIDATE_REQUIRED")
        self.assertTrue(value["transfer"]["finalized"])
        self.assertFalse(value["candidate"]["old_candidate_admitted"])
        self.assertFalse(value["candidate"]["new_candidate_materialized"])
        self.assertFalse(value["authorization"]["gpu_execution_authorized"])
        self.assertFalse(value["claims"]["solver_executed"])
        self.assertEqual(value["solver_path_rows"], 990)

    def test_schemas_are_deep_closed_and_policy_validates(self) -> None:
        for path in (gate.POLICY_SCHEMA, gate.RECEIPT_SCHEMA):
            schema = json.loads(path.read_text(encoding="utf-8"))
            Draft202012Validator.check_schema(schema)
            gate.assert_deep_closed(schema)
        schema = json.loads(gate.POLICY_SCHEMA.read_text(encoding="utf-8"))
        Draft202012Validator(schema).validate(self.policy)

    def test_real_envelope_rejects_legacy_8m_and_recomputes_budget(self) -> None:
        rows = gate.solver_rows()
        observed = gate.validate_envelope(self.policy, rows)
        self.assertAlmostEqual(observed["maximum_axis_span_m"], 8.883036647976011, places=12)
        self.assertEqual(observed["cell_counts"], [2221, 475, 23])
        self.assertEqual(observed["total_cells"], 24264425)
        self.assertEqual(observed["estimated_vram_bytes"], 3718998784)
        changed = copy.deepcopy(self.policy)
        changed["moving_envelope"]["maximum_axis_span_m"] = 8.0
        with self.assertRaisesRegex(gate.AdmissionV4Error, "maximum span"):
            gate.validate_envelope(changed, rows)

    def test_q98_and_fixed_world_gauges_fail_closed(self) -> None:
        q98 = copy.deepcopy(self.policy["gauge_contract"])
        q98["measurement_source"] = "BI4_Q98_PROXY"
        q98["is_solver_gauge"] = False
        with self.assertRaisesRegex(gate.AdmissionV4Error, "q98/non-Gauge"):
            gate.validate_gauge_contract(q98)
        fixed = copy.deepcopy(self.policy["gauge_contract"])
        fixed["world_fixed_xml_gauge_allowed"] = True
        fixed["attachment_frame"] = "WORLD"
        with self.assertRaisesRegex(gate.AdmissionV4Error, "fixed-world"):
            gate.validate_gauge_contract(fixed)

    def test_optional_c2_gpp13_old_candidate_and_authorization_fail_closed(self) -> None:
        mutations = (
            lambda value: value["selection"].__setitem__("optional_authorized", True),
            lambda value: value["selection"].__setitem__("c2_authorized", True),
            lambda value: value["future_candidate"].__setitem__("compiler", "g++-13"),
            lambda value: value["future_candidate"].__setitem__("gpp13_allowed", True),
            lambda value: value["frozen_parents"].__setitem__("old_candidate_admitted", True),
            lambda value: value["authorization"].__setitem__("gpu_execution_authorized", True),
        )
        for mutate in mutations:
            changed = copy.deepcopy(self.policy)
            mutate(changed)
            with self.subTest(changed=changed):
                with self.assertRaises(gate.AdmissionV4Error):
                    gate.validate_static_boundary(changed)

    def test_future_candidate_receipts_and_lifecycle_remain_placeholders(self) -> None:
        for section, key, value in (
            ("future_candidate", "candidate_sha256", "0"*64),
            ("future_candidate", "build_receipt_path", "/future/build.json"),
            ("future_candidate", "static_audit_receipt_sha256", "1"*64),
            ("lifecycle_placeholders", "replay_id", "replay-v4"),
            ("lifecycle_placeholders", "profile_sha256", "2"*64),
        ):
            changed = copy.deepcopy(self.policy)
            changed[section][key] = value
            with self.subTest(section=section, key=key):
                with self.assertRaises(gate.AdmissionV4Error):
                    gate.validate_static_boundary(changed)

    def test_profile_rejects_device_path_permission_and_uvm_tools_drift(self) -> None:
        template = profile.TEMPLATE_PATH.read_text(encoding="utf-8")
        rendered = profile.render_profile(template, profile.fixture_replacements())
        self.assertIn("deny /dev/nvidia-uvm-tools rw,", rendered)
        for mutate in (
            lambda value: value.__setitem__("NVIDIA0", "/dev/nvidia1"),
            lambda value: value.__setitem__("OUTPUT_ROOT", "/tmp/*"),
            lambda value: value.__setitem__("NVIDIAUVM", "/dev/nvidia-uvm-tools"),
        ):
            changed = profile.fixture_replacements()
            mutate(changed)
            with self.assertRaises(profile.ProfileV4Error):
                profile.render_profile(template, changed)
        with self.assertRaises(profile.ProfileV4Error):
            profile.render_profile(template + "\n/tmp/** rw,\n", profile.fixture_replacements())

    def test_static_sources_contain_no_runtime_or_optional_surface(self) -> None:
        gate_source = (ROOT / "scripts/r8_liquid_s5b0_replay_admission_gate_v4.py").read_text(encoding="utf-8")
        self.assertNotIn("subprocess", gate_source)
        self.assertNotIn("nvidia-smi", gate_source)
        self.assertNotIn("apparmor_parser", gate_source)
        self.assertNotIn("capture.bag", gate_source)


if __name__ == "__main__":
    unittest.main()
