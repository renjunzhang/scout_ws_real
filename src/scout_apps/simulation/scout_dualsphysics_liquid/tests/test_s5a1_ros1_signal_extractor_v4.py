#!/usr/bin/env python3
"""Static and synthetic exact-file tests for extractor v4."""

import hashlib
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import r8_liquid_s5a1_ros1_signal_extractor_v4 as extractor  # noqa: E402


class ExtractorV4Tests(unittest.TestCase):
    def test_base_worker_interfaces_exist(self) -> None:
        for name in ("read_exact_primary", "extract_bag_bytes", "nearest_clock_alignment", "tf_cross_check"):
            self.assertTrue(callable(getattr(extractor, name)))

    def test_synthetic_exact_file_read_and_negative_identity(self) -> None:
        raw = b"synthetic-not-a-real-bag"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.bag"
            path.write_bytes(raw); os.chmod(path, 0o400)
            digest = hashlib.sha256(raw).hexdigest()
            with mock.patch.object(extractor.extractor_v3, "extract_bag_bytes", return_value={"status":"SYNTHETIC_PARSE"}):
                value, before, after = extractor.read_exact_primary(
                    path, expected_size=len(raw), expected_mode=0o400, expected_sha256=digest)
            self.assertEqual(value["status"], "SYNTHETIC_PARSE")
            self.assertEqual(before, after)
            with self.assertRaises(Exception):
                extractor.read_exact_primary(path, expected_size=len(raw)+1,
                                             expected_mode=0o400, expected_sha256=digest)


if __name__ == "__main__": unittest.main()
