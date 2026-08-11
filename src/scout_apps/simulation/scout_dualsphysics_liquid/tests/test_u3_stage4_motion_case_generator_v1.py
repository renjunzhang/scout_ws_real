#!/usr/bin/env python3
"""Static and negative tests for the Stage-4 motion XML generator."""

from __future__ import annotations

import copy
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_stage4_motion_case_generator_v1 as generator  # noqa: E402
import r8_liquid_u3_stage4_experiment_runner_v1 as runner  # noqa: E402


POLICIES = {
    "zero": PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_settled_zero_replay_20260811T135543Z_v20.json",
    "translation": PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_synthetic_translation_20260811T135543Z_v21.json",
    "yaw": PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_synthetic_yaw_20260811T135543Z_v22.json",
}


class Stage4MotionCaseGeneratorV1Tests(unittest.TestCase):
    def test_schema_is_valid_and_all_object_definitions_are_closed(self) -> None:
        schema = json.loads(generator.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertIs(schema["additionalProperties"], False)
        for name, definition in schema["$defs"].items():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False, name)

    def test_exact_contract_and_parents_validate(self) -> None:
        contract = generator.load_contract()
        verified = generator._verify_parents(contract)
        self.assertEqual(set(contract["cases"]), {"zero", "translation", "yaw"})
        self.assertEqual(verified["checkpoint_part"]["sha256"], "023c2ae47281351f7c601a60709799cf8155745193d7e2aa5ee862256ad9b1f4")
        self.assertEqual(contract["checkpoint"]["time_s"], 45.05001991890928)

    def test_all_cases_change_only_two_motion_blocks(self) -> None:
        contract = generator.load_contract()
        base = generator.BASE_CASE_PATH.read_bytes()
        self.assertEqual(base.count(generator.BASE_MOTION_BLOCK), 2)
        for name, case in contract["cases"].items():
            rendered = generator.render_case(base, case)
            block = generator.motion_block(case)
            self.assertEqual(rendered.count(block), 2, name)
            self.assertEqual(rendered.replace(block, generator.BASE_MOTION_BLOCK), base, name)
            self.assertEqual(len(generator._motion_elements(rendered)), 2, name)

    def test_translation_and_yaw_use_upstream_sinusoidal_elements_and_stop_link(self) -> None:
        contract = generator.load_contract()
        base = generator.BASE_CASE_PATH.read_bytes()
        translation = generator.render_case(base, contract["cases"]["translation"])
        yaw = generator.render_case(base, contract["cases"]["yaw"])
        self.assertEqual(translation.count(b"<mvrectsinu "), 2)
        self.assertEqual(translation.count(b'duration="2.0" next="2"'), 2)
        self.assertEqual(translation.count(b'<ampl x="0.002" y="0" z="0"'), 2)
        self.assertEqual(yaw.count(b"<mvrotsinu "), 2)
        self.assertEqual(yaw.count(b'anglesunits="degrees"'), 2)
        self.assertEqual(yaw.count(b'<ampl v="2.0"'), 2)
        self.assertEqual(yaw.count(b'<axisp1 x="0" y="0" z="0"'), 2)
        self.assertEqual(yaw.count(b'<axisp2 x="0" y="0" z="1"'), 2)
        self.assertEqual(translation.count(b'<mvnull id="2"'), 2)
        self.assertEqual(yaw.count(b'<mvnull id="2"'), 2)

    def test_exact_checkpoint_start_prevents_restart_phase_pre_advance(self) -> None:
        contract = generator.load_contract()
        for case in contract["cases"].values():
            self.assertEqual(case["start_time_s"], contract["checkpoint"]["time_s"])
            block = generator.motion_block(case)
            self.assertIn(b'start="45.05001991890928"', block)

    def test_motion_parameter_and_identity_drift_is_rejected(self) -> None:
        original = generator.load_contract()
        mutations = (
            ("amplitude", lambda value: value["cases"]["translation"].__setitem__("translation_amplitude_m", 0.0021)),
            ("frequency", lambda value: value["cases"]["yaw"].__setitem__("frequency_hz", 2.0)),
            ("phase", lambda value: value["cases"]["translation"].__setitem__("phase_rad", 1.0)),
            ("axis", lambda value: value["cases"]["yaw"].__setitem__("yaw_axis", "NONE")),
            ("duration", lambda value: value["cases"]["translation"].__setitem__("active_duration_s", 1.5)),
            ("tail", lambda value: value["cases"]["yaw"].__setitem__("tail_duration_s", 2.0)),
            ("start", lambda value: value["cases"]["zero"].__setitem__("start_time_s", 45.05)),
            ("output", lambda value: value["cases"]["yaw"].__setitem__("output_xml", "/tmp/yaw.xml")),
        )
        for name, mutate in mutations:
            changed = copy.deepcopy(original)
            mutate(changed)
            with self.assertRaises(generator.MotionCaseError, msg=name):
                generator.semantic_validate(changed, generator.CONTRACT_PATH)

    def test_create_new_writer_refuses_collision_and_keeps_mode(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "case.xml"
            with mock.patch.object(generator, "CASE_ROOT", root):
                observed = generator._write_exclusive(target, b"<case />\n")
                self.assertEqual(observed["mode"], "0640")
                self.assertEqual(observed["sha256"], hashlib.sha256(b"<case />\n").hexdigest())
                with self.assertRaises(FileExistsError):
                    generator._write_exclusive(target, b"different")
            self.assertEqual(target.read_bytes(), b"<case />\n")

    def test_self_check_is_static_and_does_not_materialize_outputs(self) -> None:
        contract = generator.load_contract()
        before = {
            name: generator.identity(Path(case["output_xml"]))
            for name, case in contract["cases"].items()
        }
        report = generator.self_check()
        after = {
            name: generator.identity(Path(case["output_xml"]))
            for name, case in contract["cases"].items()
        }
        self.assertEqual(after, before)
        self.assertEqual(report["status"], "PASS_U3_STAGE4_MOTION_CASE_GENERATOR_V1_SELF_CHECK")
        self.assertIs(report["files_written"], False)
        self.assertIs(report["solver_executed"], False)
        self.assertIs(report["gpu_exposed"], False)
        self.assertIs(report["network_used"], False)

    def test_exact_dynamic_policies_validate_against_frozen_runner(self) -> None:
        contract = generator.load_contract()
        for name, path in POLICIES.items():
            policy = runner.load_policy(path)
            case = contract["cases"][name]
            self.assertEqual(policy["run_id"], case["run_id"])
            self.assertEqual(policy["inputs"]["case_xml"]["path"], case["output_xml"])
            self.assertEqual(policy["experiment"]["family"], case["family"])
            self.assertEqual(policy["experiment"]["motion_kind"], case["motion_kind"])
            self.assertEqual(policy["expected_output"]["end_time_s"], case["end_time_s"])
            self.assertEqual(policy["expected_output"]["part_count"], case["part_count"])
            self.assertEqual(policy["backend"], {"kind": "GPU", "index": 0, "parallel_jobs": 1})

    def test_dynamic_policies_have_exact_restart_and_sandbox_boundary(self) -> None:
        for name, path in POLICIES.items():
            policy = runner.load_policy(path)
            self.assertEqual(policy["restart"], {"enabled": True, "part_index": 901, "part_first": 901, "guest_dir": "/restart"})
            argv = runner.bwrap_argv(policy)
            writes = [argv[index + 1 : index + 3] for index, item in enumerate(argv) if item == "--bind"]
            self.assertEqual(writes, [[policy["run"]["guest_output_root"], "/output"]], name)
            self.assertIn("--unshare-net", argv)
            self.assertNotIn("--share-net", argv)
            self.assertIn("/restart", argv)
            self.assertNotIn("/home/zrj/scout_ws", argv)
            self.assertEqual(policy["solver"]["argv"].count("-partbegin:901:901"), 1)
            self.assertEqual(policy["solver"]["argv"].count("-gpu:0"), 1)

    def test_policy_tmax_restart_or_case_drift_is_rejected(self) -> None:
        source = runner.load_policy(POLICIES["translation"])
        mutations = (
            ("tmax", lambda value: value["solver"]["argv"].__setitem__(value["solver"]["argv"].index("-tmax:48.05001991890928"), "-tmax:49")),
            ("restart", lambda value: value["restart"].__setitem__("part_index", 900)),
            ("network", lambda value: value["sandbox"].__setitem__("network", True)),
        )
        for name, mutate in mutations:
            changed = copy.deepcopy(source)
            mutate(changed)
            with self.assertRaises((runner.Stage4ExperimentError, runner.legacy.Stage4RunError), msg=name):
                runner.semantic_validate(changed, POLICIES["translation"])
        changed = copy.deepcopy(source["inputs"]["case_xml"])
        changed["sha256"] = "0" * 64
        with self.assertRaises(runner.legacy.Stage4RunError):
            runner._verify(changed, full=True)


if __name__ == "__main__":
    unittest.main()
