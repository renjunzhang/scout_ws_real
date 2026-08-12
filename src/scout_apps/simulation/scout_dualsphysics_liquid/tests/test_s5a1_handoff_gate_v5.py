#!/usr/bin/env python3
import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT/"scripts"))
import r8_liquid_s5a1_handoff_gate_v5 as gate
class Tests(unittest.TestCase):
 def test_v3_v3_effective_identity(self):
  p,_=gate.expand_policy();s,_=gate.expand_schema();self.assertEqual(p["transfer_id"],gate.TRANSFER_ID);self.assertEqual(s["properties"]["transfer_id"],{"const":gate.TRANSFER_ID});self.assertIn("v3_v3",p["package"]["planned_partial_root"])
 def test_required_gate_interfaces(self):
  for n in ("read_json","validate_s5a0_receipt","validate_package_bytes","validate_package_root"):self.assertTrue(callable(getattr(gate,n)))
if __name__=="__main__":unittest.main()
