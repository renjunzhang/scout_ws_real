#!/usr/bin/env python3
"""Synthetic-only tests for the create-new S5B0 v2 static execution layer."""

from __future__ import annotations

import ast
import copy
import importlib.util
import json
import sys
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"


def load(name: str, filename: str):
    path = SCRIPTS / filename
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


profile = load("s5b0_profile_v2_test", "r8_liquid_s5b0_profile_generator_v2.py")
gate = load("s5b0_gate_v2_test", "r8_liquid_s5b0_replay_admission_gate_v2.py")
supervisor = load("s5b0_supervisor_v2_test", "r8_liquid_s5b0_replay_supervisor_v2.py")


class S5B0ReplayAdmissionV2Tests(unittest.TestCase):
    def setUp(self) -> None:
        self.policy, _ = gate.load_and_validate_policy()
        self.rows = ((0, 0, 0, 0, 0, 0, 0), (1, 0.2, 0, 0, 5, 0, 0), (2, 0.2, 0, 0, 5, 0, 0))

    def test_policy_and_all_schemas_are_deep_closed(self) -> None:
        for path in (gate.POLICY_SCHEMA, gate.RECEIPT_SCHEMA, gate.QC_SCHEMA):
            schema = json.loads(path.read_bytes())
            Draft202012Validator.check_schema(schema)
            gate.assert_deep_closed(schema)
        schema = json.loads(gate.POLICY_SCHEMA.read_bytes())
        changed = copy.deepcopy(self.policy)
        changed["unexpected"] = True
        with self.assertRaises(ValidationError):
            Draft202012Validator(schema).validate(changed)

    def test_self_checks_are_not_admitted_and_non_executing(self) -> None:
        result = gate.self_check()
        self.assertEqual(result["status"], "NOT_ADMITTED_S5A1_FINALIZED_REQUIRED")
        self.assertFalse(result["gpu_exposed"])
        self.assertFalse(result["solver_executed"])
        self.assertFalse(result["profile_loaded"])
        self.assertFalse(result["files_written"])
        self.assertEqual(profile.self_check()["device_count"], 3)

    def test_exact_stage4_flags_denominator_and_devices(self) -> None:
        self.assertEqual(tuple(self.policy["solver_contract"]["common_flags"]), gate.COMMON_FLAGS)
        self.assertEqual(len(gate.COMMON_FLAGS), 15)
        self.assertEqual(self.policy["selection"]["planned_denominator"], 1)
        self.assertFalse(self.policy["selection"]["optional_authorized"])
        self.assertEqual(self.policy["profile_contract"]["devices"], ["/dev/nvidia0", "/dev/nvidiactl", "/dev/nvidia-uvm"])
        self.assertFalse(self.policy["profile_contract"]["uvm_tools_default"])

    def test_profile_renderer_rejects_uvm_tools_and_extra_writable_tree(self) -> None:
        template = profile.TEMPLATE_PATH.read_text(encoding="utf-8")
        rendered = profile.render_profile(template, profile.fixture_replacements())
        self.assertNotIn("\n  /dev/nvidia-uvm-tools rw,", rendered)
        changed = profile.fixture_replacements()
        changed["NVIDIAUVM"] = "/dev/nvidia-uvm-tools"
        with self.assertRaises(profile.ProfileV2Error):
            profile.render_profile(template, changed)
        with self.assertRaises(profile.ProfileV2Error):
            profile.render_profile(template + "\n/tmp/** rw,\n", profile.fixture_replacements())

    def test_tail_is_in_path_tmax_does_not_add_it_twice(self) -> None:
        checked = gate.validate_solver_path(self.rows)
        estimate = gate.moving_envelope(self.policy, checked, dynamic_free_bytes=12_000_000_000)
        self.assertEqual(estimate["solver_path_last_t_s"], 2.0)
        self.assertAlmostEqual(estimate["tmax_s"], 47.05001991890928)
        self.assertFalse(estimate["tail_added_again"])
        bad = list(map(list, self.rows))
        bad[-1][1] = 0.3
        with self.assertRaises(gate.AdmissionV2Error):
            gate.validate_solver_path(bad)

    def test_envelope_samples_quarters_and_output_grid_and_separates_path_length(self) -> None:
        estimate = gate.moving_envelope(self.policy, self.rows, dynamic_free_bytes=12_000_000_000)
        self.assertGreater(estimate["query_count"], 2 * 4)
        self.assertEqual(estimate["interval_subdivisions"], 4)
        self.assertTrue(estimate["output_grid_included"])
        self.assertFalse(estimate["path_length_is_axis_span"])
        self.assertTrue(estimate["dynamic_resource_admitted"])
        static = gate.moving_envelope(self.policy, self.rows, dynamic_free_bytes=None)
        self.assertFalse(static["dynamic_resource_admitted"])

    def test_axis_span_and_dynamic_vram_fail_closed(self) -> None:
        far = ((0,0,0,0,0,0,0), (1,8.1,0,0,0,0,0), (2,8.1,0,0,0,0,0))
        with self.assertRaisesRegex(gate.AdmissionV2Error, "axis-span"):
            gate.moving_envelope(self.policy, far, dynamic_free_bytes=12_000_000_000)
        with self.assertRaisesRegex(gate.AdmissionV2Error, "dynamic free VRAM"):
            gate.moving_envelope(self.policy, self.rows, dynamic_free_bytes=1)

    def test_qc_rejects_nout_xid_gauge_and_pass_contradiction(self) -> None:
        gate.validate_result_qc(gate.fixture_qc())
        for mutate in (
            lambda value: value["particles"].__setitem__("nout", 1),
            lambda value: value["resources"].__setitem__("xid_count", 1),
            lambda value: value["gauge"].__setitem__("observed_slots", 40),
        ):
            value = gate.fixture_qc()
            mutate(value)
            with self.assertRaises(gate.AdmissionV2Error):
                gate.validate_result_qc(value)

    def test_supervisor_plan_uses_staged_candidate_pgid_and_fresh_roots(self) -> None:
        result = supervisor.self_check()
        argv = result["solver_argv"]
        self.assertTrue(argv[0].endswith("/runtime/candidate"))
        self.assertEqual(argv.count("-gpu:0"), 1)
        self.assertFalse(any(token == "-j" or token.startswith("-j:") for token in argv))
        self.assertTrue(result["spawn_contract"]["start_new_session"])
        with self.assertRaises(supervisor.SupervisorV2Error):
            supervisor.validate_fresh_targets({
                "partial_root":"/x/a", "final_root":"/x/b", "start_receipt":"/x/c",
                "final_receipt":"/x/d", "failure_receipt":"/x/e"}, lambda path: path == "/x/c")
        with self.assertRaisesRegex(supervisor.SupervisorV2Error, "NOT_ADMITTED"):
            supervisor.run_one_shot()

    def test_static_entrypoints_have_no_runtime_subprocess_surface(self) -> None:
        for path in (gate.POLICY_PATH,):
            self.assertTrue(path.is_file())
        for filename in ("r8_liquid_s5b0_replay_admission_gate_v2.py", "r8_liquid_s5b0_profile_generator_v2.py"):
            tree = ast.parse((SCRIPTS / filename).read_text(encoding="utf-8"))
            imports = {alias.name.split(".")[0] for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
            self.assertFalse(imports & {"subprocess", "socket"})


if __name__ == "__main__":
    unittest.main()
