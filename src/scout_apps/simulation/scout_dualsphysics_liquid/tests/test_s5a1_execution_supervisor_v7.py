#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT/"scripts"))
import r8_liquid_s5a1_execution_supervisor_v7 as s
class Tests(unittest.TestCase):
 def test_static(self):self.assertEqual(s.self_check()["status"],"PASS_S5A1_EXECUTION_SUPERVISOR_V7_STATIC_CONTRACT")
 def test_base_interfaces(self):
  s.configure();
  for n in ("read_exact_primary","nearest_clock_alignment","tf_cross_check"):self.assertTrue(callable(getattr(s.base.extractor,n)))
  for n in ("read_json","validate_s5a0_receipt","validate_package_root"):self.assertTrue(callable(getattr(s.base.gate,n)))
 def test_v6_preserved_fresh_root(self):self.assertEqual(s.preserved()["failure_receipt"]["sha256"],"3051aedaf265a388e949a27807436d31b544fff7efa87a3433ef8ee081ef2408");self.assertIn("v3_v3",str(s.PARTIAL_ROOT))
if __name__=="__main__":unittest.main()
