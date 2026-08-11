from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


WORKSPACE = Path("/home/zrj/scout_ws")
SCRIPT = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/r8_liquid_gpu_stage4_campaign_gate_v1.py"


def load_gate():
    spec = importlib.util.spec_from_file_location("gpu_stage4_campaign_gate_v1_tested", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GpuStage4CampaignGateV1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gate = load_gate()
        cls.policy = cls.gate.policy()

    def test_parent_source_tool_and_object_contracts(self):
        parent = self.gate.verify_parents_and_tools(self.policy)
        sealed = self.gate.validate_sealed_source(self.policy)
        self.assertEqual(parent["object_contract"]["total_object_count"], 131)
        self.assertEqual(parent["object_contract"]["cpp_object_count"], 120)
        self.assertEqual(parent["object_contract"]["cuda_object_count"], 11)
        self.assertEqual(sealed, {"file_count": 352, "total_bytes": 5473917})

    def test_build_command_is_networkless_gpu_less_uid_dropped_and_j1(self):
        argv = self.gate.build_child_argv(self.policy)
        joined = "\n".join(argv)
        self.assertEqual(argv[0], "/usr/bin/timeout")
        self.assertNotIn("/usr/bin/sudo", argv)
        self.assertNotIn("/usr/bin/setpriv", argv)
        self.assertIn("--unshare-net", argv)
        self.assertIn("--disable-userns", argv)
        self.assertIn("-j1", argv)
        self.assertIn("compute_120", joined)
        self.assertIn("sm_120", joined)
        self.assertNotIn("/dev/nvidia", joined)
        self.assertNotIn("g++-13", joined)
        self.assertNotIn("-j2", argv)
        self.assertNotIn("-j4", argv)

    def test_build_writes_only_fresh_external_root(self):
        argv = self.gate.build_child_argv(self.policy)
        root = self.policy["paths"]["attempt_root"]
        joined = "\n".join(argv)
        self.assertIn(root + "/output", joined)
        self.assertNotIn(str(WORKSPACE) + ":", joined)
        self.assertTrue(root.startswith("/home/zrj/scout_liquid_lab/build/"))
        self.assertTrue(root.endswith(".partial"))

    def test_wrapper_is_exact_84_byte_ccflags_only(self):
        wrapper = self.policy["wrapper"]
        content = wrapper["content_utf8"].encode("utf-8")
        self.assertEqual(len(content), 84)
        self.assertEqual(self.gate.sha256_bytes(content), wrapper["sha256"])
        self.assertIn(b"override CCFLAGS += -include cstdint", content)
        self.assertNotIn(b"NCCFLAGS", content)

    def test_create_new_writer_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "receipt.json"
            self.gate.write_new(path, b"first", mode=0o600)
            with self.assertRaises(FileExistsError):
                self.gate.write_new(path, b"second", mode=0o600)
            self.assertEqual(path.read_bytes(), b"first")


if __name__ == "__main__":
    unittest.main()
