"""Mock-only static checks for v14's exact retained-object inventory contract."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import struct
import tempfile
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GATE_PATH = PACKAGE_DIR / "scripts/r8_liquid_target_u3_cpu_build_gate_v14.py"
POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_cpu_build_execution_policy_v14.json"
SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_cpu_build_execution_policy_v14.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_cpu_build_gate_v14", GATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def argv_pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1 : index + 3] for index, item in enumerate(argv[:-2]) if item == flag]


def rel_elf(*, e_type: int = 1, section_count: int = 1) -> bytes:
    ident = b"\x7fELF\x02\x01\x01" + b"\0" * 9
    header = struct.pack(
        "<16sHHIQQQIHHHHHH",
        ident,
        e_type,
        62,
        1,
        0,
        0,
        64,
        0,
        64,
        0,
        0,
        64,
        section_count,
        0,
    )
    return header + b"\0" * (64 * section_count)


class TargetU3CpuBuildGateV14Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        cls.schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))

    def test_policy_is_closed_and_v13_is_byte_pinned(self):
        self.assertFalse(self.schema["additionalProperties"])
        self.assertEqual(set(self.policy), set(self.schema["required"]))
        self.assertEqual(
            hashlib.sha256(self.gate.PREVIOUS_GATE_PATH.read_bytes()).hexdigest(),
            self.gate.PREVIOUS_GATE_SHA256,
        )
        self.assertEqual(
            hashlib.sha256(self.gate.previous.POLICY_PATH.read_bytes()).hexdigest(),
            self.gate.PREVIOUS_POLICY_SHA256,
        )
        self.assertNotEqual(self.gate.BUILD_ID, self.gate.previous.BUILD_ID)
        for key in (
            "source_input",
            "isolation",
            "resources",
            "source_copy_command",
            "build_command",
            "generated_wrapper",
            "profile_lifecycle",
            "invariants",
        ):
            self.assertEqual(self.policy[key], self.gate.previous.core.read_json_object(self.gate.previous.POLICY_PATH)[key])
        self.assertEqual(self.policy["static_artifact_audit"]["post_build_object_contract"], self.gate.OBJECT_CONTRACT)
        self.assertTrue(self.policy["invariants"]["no_network"])
        self.assertTrue(self.policy["invariants"]["no_gpu"])
        self.assertTrue(self.policy["invariants"]["no_compiled_artifact_execution"])

    def test_sealed_makefile_derives_the_exact_109_object_manifest(self):
        expected = self.gate.core._source_entries_from_receipt()
        objects = self.gate.parse_cpu_object_manifest_v14(self.gate.core.SOURCE_ROOT, expected)
        self.assertEqual(len(objects), 109)
        self.assertEqual(self.gate.core.canonical_hash(list(objects)), self.gate.OBJECT_NAMES_CANONICAL_SHA256)
        self.assertIn("JDsFixedDt.o", objects)
        self.assertNotIn("JDsGpuInfo.o", objects)
        self.assertTrue(all(f"{name[:-2]}.cpp" in expected for name in objects))

    def test_derived_paths_profiles_and_bwrap_are_rebound_without_widening(self):
        self.assertEqual(self.gate.core.ATTEMPT_ROOT, self.gate.ATTEMPT_ROOT)
        self.assertEqual(self.gate.core.OUTPUT_ROOT, self.gate.OUTPUT_ROOT)
        self.assertEqual(self.gate.core.OUTPUT_SOURCE, self.gate.OUTPUT_SOURCE)
        self.assertEqual(self.gate.core.ARTIFACT_PATH, self.gate.ARTIFACT_PATH)
        for path, digest, name, old_path, old_name in (
            (self.gate.COPY_PROFILE_PATH, self.gate.COPY_PROFILE_SHA256, self.gate.COPY_PROFILE, self.gate.previous.COPY_PROFILE_PATH, self.gate.previous.COPY_PROFILE),
            (self.gate.BUILD_PROFILE_PATH, self.gate.BUILD_PROFILE_SHA256, self.gate.BUILD_PROFILE, self.gate.previous.BUILD_PROFILE_PATH, self.gate.previous.BUILD_PROFILE),
        ):
            text = path.read_text(encoding="utf-8")
            effective = self.gate._effective_profile(text)
            normalized = (
                effective.replace(str(self.gate.ATTEMPT_ROOT), str(self.gate.previous.ATTEMPT_ROOT))
                .replace(str(self.gate.OUTPUT_ROOT), str(self.gate.previous.OUTPUT_ROOT))
                .replace(name, old_name)
            )
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
            self.assertEqual(normalized, self.gate.previous._effective_profile(old_path.read_text(encoding="utf-8")))
            self.assertNotIn("flags=(unconfined)", effective)
        for phase in ("source-copy", "build"):
            argv = self.gate.bwrap_prefix_v14(phase=phase)
            expected_ro = [["/usr", "/usr"]]
            if phase == "source-copy":
                expected_ro.append([str(self.gate.core.SOURCE_ROOT), "/work/input"])
            self.assertEqual(argv_pairs(argv, "--ro-bind"), expected_ro)
            self.assertEqual(argv_pairs(argv, "--symlink"), [["usr/lib", "/lib"], ["usr/lib64", "/lib64"]])
            self.assertEqual(argv_pairs(argv, "--bind"), [[str(self.gate.OUTPUT_ROOT), "/work/output"]])
            self.assertIn("--unshare-net", argv)
            self.assertIn("--disable-userns", argv)
            self.assertNotIn("--proc", argv)
            self.assertNotIn("--dev", argv)
            self.assertNotIn(str(self.gate.previous.OUTPUT_ROOT), argv)

    def _fixture(self, root: Path, *, object_bytes: bytes = rel_elf(), object_mode: int = 0o644, object_name: str = "Alpha.o", nested: bool = False) -> dict[str, dict[str, object]]:
        source = root / "Alpha.cpp"
        source.write_bytes(b"sealed alpha source\n")
        (root / self.gate.core.WRAPPER_NAME).write_text(self.gate.core.WRAPPER_CONTENT, encoding="utf-8")
        os.chmod(root / self.gate.core.WRAPPER_NAME, 0o600)
        object_path = (root / "nested" / object_name) if nested else (root / object_name)
        object_path.parent.mkdir(exist_ok=True)
        object_path.write_bytes(object_bytes)
        os.chmod(object_path, object_mode)
        return {"Alpha.cpp": {"sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "size_bytes": source.stat().st_size}}

    def test_built_source_inventory_requires_one_exact_safe_rel_object_set(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            expected = self._fixture(root)
            report = self.gate.inventory_built_source_v14(root, expected, ("Alpha.o",))
            self.assertEqual(report["object_count"], 1)
            self.assertEqual(report["objects"][0]["elf_type"], "ET_REL")
            self.assertEqual(report["allowed_extra_files"], [self.gate.core.WRAPPER_NAME])

    def test_rel_object_section_header_bound_is_explicit_and_enforced(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            allowed = root / "allowed.o"
            allowed.write_bytes(rel_elf(section_count=self.gate.OBJECT_SECTION_HEADER_COUNT_MAX))
            os.chmod(allowed, 0o644)
            report = self.gate.audit_rel_object_v14(allowed)
            self.assertEqual(report["section_header_count"], self.gate.OBJECT_SECTION_HEADER_COUNT_MAX)

            rejected = root / "rejected.o"
            rejected.write_bytes(rel_elf(section_count=self.gate.OBJECT_SECTION_HEADER_COUNT_MAX + 1))
            os.chmod(rejected, 0o644)
            with self.assertRaises(self.gate.core.GateError):
                self.gate.audit_rel_object_v14(rejected)

    def test_built_source_inventory_rejects_missing_extra_nested_exec_or_nonrel_objects(self):
        cases = (
            ("missing", None, None, 0o644, False),
            ("extra", "Other.o", rel_elf(), 0o644, False),
            ("nested", "Alpha.o", rel_elf(), 0o644, True),
            ("executable", "Alpha.o", rel_elf(), 0o700, False),
            ("nonrel", "Alpha.o", rel_elf(e_type=2), 0o644, False),
        )
        for name, object_name, object_bytes, object_mode, nested in cases:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source = root / "Alpha.cpp"
                source.write_bytes(b"sealed alpha source\n")
                (root / self.gate.core.WRAPPER_NAME).write_text(self.gate.core.WRAPPER_CONTENT, encoding="utf-8")
                os.chmod(root / self.gate.core.WRAPPER_NAME, 0o600)
                if object_name is not None:
                    target = (root / "nested" / object_name) if nested else root / object_name
                    target.parent.mkdir(exist_ok=True)
                    target.write_bytes(object_bytes)
                    os.chmod(target, object_mode)
                expected = {"Alpha.cpp": {"sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "size_bytes": source.stat().st_size}}
                with self.assertRaises(self.gate.core.GateError):
                    self.gate.inventory_built_source_v14(root, expected, ("Alpha.o",))

    def test_built_source_inventory_rejects_symlink_hardlink_or_missing_sealed_cpp(self):
        for kind in ("symlink", "hardlink", "missing_cpp"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source = root / "Alpha.cpp"
                source.write_bytes(b"sealed alpha source\n")
                wrapper = root / self.gate.core.WRAPPER_NAME
                wrapper.write_text(self.gate.core.WRAPPER_CONTENT, encoding="utf-8")
                os.chmod(wrapper, 0o600)
                object_path = root / "Alpha.o"
                if kind == "symlink":
                    object_path.symlink_to(source.name)
                elif kind == "hardlink":
                    os.link(source, object_path)
                else:
                    object_path.write_bytes(rel_elf())
                    os.chmod(object_path, 0o644)
                expected = {"Alpha.cpp": {"sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "size_bytes": source.stat().st_size}}
                if kind == "missing_cpp":
                    expected = {}
                with self.assertRaises(self.gate.core.GateError):
                    self.gate.inventory_built_source_v14(root, expected, ("Alpha.o",))

    def test_static_review_and_transition_use_the_v14_inventory(self):
        review = self.gate.verify_review_artifacts_v14()
        self.assertEqual(review["frozen_v13_gate"]["sha256"], self.gate.PREVIOUS_GATE_SHA256)
        self.assertEqual(review["object_contract"], self.gate.OBJECT_CONTRACT)
        self.assertIs(self.gate.core.inventory_build_output, self.gate.inventory_build_output_v14)
        self.assertIs(self.gate.core.execute_build, self.gate.execute_build_v14)


if __name__ == "__main__":
    unittest.main()
