#!/usr/bin/env python3
"""Pure/mock tests: no external root, AppArmor lifecycle or GPU access."""
from __future__ import annotations
import ast,copy,importlib.util,json,sys,unittest
from pathlib import Path
from jsonschema import Draft202012Validator,ValidationError
ROOT=Path(__file__).resolve().parent.parent;S=ROOT/'scripts'
def load(name,file):
 spec=importlib.util.spec_from_file_location(name,S/file);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;assert spec.loader;spec.loader.exec_module(m);return m
gate=load('s5b0_gate_v6_test','r8_liquid_s5b0_replay_execution_gate_v6.py');sup=load('s5b0_sup_v6_test','r8_liquid_s5b0_replay_lifecycle_supervisor_v6.py')
class V6(unittest.TestCase):
 def setUp(self):self.policy,_=gate.load_policy()
 def test_closed_schemas(self):
  for path in (gate.SCHEMA,gate.RECEIPT_SCHEMA):
   s=json.loads(path.read_bytes());Draft202012Validator.check_schema(s);gate.deep_closed(s)
  x=copy.deepcopy(self.policy);x['x']=1
  with self.assertRaises(ValidationError):Draft202012Validator(json.loads(gate.SCHEMA.read_bytes())).validate(x)
 def test_default_deny(self):
  r=gate.self_check();self.assertEqual(r['status'],'NOT_ADMITTED_FRESH_CANDIDATE_REQUIRED');self.assertFalse(r['claims']['solver_executed']);self.assertFalse(r['claims']['optional_bag_read'])
 def test_frozen_gauge_resources_primary(self):
  self.assertEqual(self.policy['selection']['planned_denominator'],1);self.assertFalse(self.policy['selection']['optional_authorized']);self.assertEqual(self.policy['gauge']['probe_count'],16);self.assertEqual(self.policy['resources']['wall_timeout_seconds'],5400);self.assertEqual(self.policy['resources']['minimum_free_vram_bytes'],6442450944)
 def test_premature_input_is_schema_and_gate_rejected(self):
  x=copy.deepcopy(self.policy);x['candidate_input']['sha256']='0'*64
  with self.assertRaises(ValidationError):Draft202012Validator(json.loads(gate.SCHEMA.read_bytes())).validate(x)
  x=copy.deepcopy(self.policy);x['authorization']['user_authorized']=True
  with self.assertRaises(gate.GateV6Error):gate.validate_static(x)
 def test_finalization_requires_exact_authorization(self):
  f={'candidate_path':'/fresh/candidate','candidate_sha256':'1'*64,'build_receipt_path':'/a/build','build_receipt_sha256':'2'*64,'static_audit_receipt_path':'/a/static','static_audit_receipt_sha256':'3'*64,'capability':'MOTION_ATTACHED_16_RAW_JGAUGESWL','replay_id':'s5b0_primary_replay_v6','stage_root':'/r/stage.partial','partial_root':'/r/run.partial','final_root':'/r/run','start_receipt':'/a/start','final_receipt':'/a/final','failure_receipt':'/a/failure','profile_name':'r8-liquid-s5b0-primary-replay-v6','profile_path':'/profiles/a','profile_sha256':'4'*64,'authorization':{'policy_sha256':'0'*64,'candidate_sha256':'1'*64,'profile_sha256':'4'*64,'user_authorized':True}}
  with self.assertRaises(gate.GateV6Error):gate.validate_finalization(self.policy,f)
 def test_postflight_negative(self):
  q={'raw_names':[f'GaugesSwl_s5b0_p{i:02d}.csv' for i in range(16)],'invalid_ratio':0.0,'particles':9078,'nout':0,'finite':True,'motion':True,'xid_count':0,'output_bytes':1}
  gate.validate_postflight(self.policy,q);q['raw_names']=q['raw_names'][:-1]
  with self.assertRaises(gate.GateV6Error):gate.validate_postflight(self.policy,q)
 def test_monitor_and_fresh_negative(self):
  samples=({'monotonic_s':0,'free_vram_bytes':6442450944,'output_bytes':1,'xid_count':0},{'monotonic_s':20,'free_vram_bytes':6442450944,'output_bytes':2,'xid_count':0})
  self.assertEqual(sup.bounded_monitor(samples,self.policy)['sample_count'],2)
  with self.assertRaises(sup.SupervisorV6Error):sup.bounded_monitor(({'monotonic_s':0,'free_vram_bytes':1,'output_bytes':1,'xid_count':0},),self.policy)
 def test_no_subprocess_surface(self):
  for f in ('r8_liquid_s5b0_replay_execution_gate_v6.py','r8_liquid_s5b0_replay_lifecycle_supervisor_v6.py'):
   t=ast.parse((S/f).read_text());names={a.name.split('.')[0] for n in ast.walk(t) if isinstance(n,ast.Import) for a in n.names};self.assertFalse(names&{'subprocess','socket'})
if __name__=='__main__':unittest.main()
