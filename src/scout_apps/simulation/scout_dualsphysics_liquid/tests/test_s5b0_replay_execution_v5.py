#!/usr/bin/env python3
"""Mock/static-only regression checks for the unadmitted S5B0 v5 skeleton."""
from __future__ import annotations
import ast,copy,importlib.util,json,sys,unittest
from pathlib import Path
from jsonschema import Draft202012Validator,ValidationError
ROOT=Path(__file__).resolve().parent.parent; SCRIPTS=ROOT/'scripts'
def load(name:str,file:str):
    spec=importlib.util.spec_from_file_location(name,SCRIPTS/file); mod=importlib.util.module_from_spec(spec);sys.modules[name]=mod;assert spec.loader;spec.loader.exec_module(mod);return mod
profile=load('s5b0_profile_v5_test','r8_liquid_s5b0_profile_generator_v5.py')
gate=load('s5b0_gate_v5_test','r8_liquid_s5b0_replay_execution_gate_v5.py')
supervisor=load('s5b0_supervisor_v5_test','r8_liquid_s5b0_replay_supervisor_v5.py')
class V5Tests(unittest.TestCase):
 def setUp(self):self.policy,_=gate.load_policy()
 def test_schemas_closed_and_reject_extra(self):
  for path in (gate.POLICY_SCHEMA,gate.RECEIPT_SCHEMA):
   schema=json.loads(path.read_bytes());Draft202012Validator.check_schema(schema);gate.deep_closed(schema)
  changed=copy.deepcopy(self.policy);changed['extra']=True
  with self.assertRaises(ValidationError):Draft202012Validator(json.loads(gate.POLICY_SCHEMA.read_bytes())).validate(changed)
 def test_static_result_is_unadmitted_and_nonexecuting(self):
  result=gate.self_check()
  self.assertEqual(result['status'],'NOT_ADMITTED_FRESH_PATCHED_CANDIDATE_AND_EXACT_AUTHORIZATION_REQUIRED')
  self.assertFalse(result['claims']['solver_executed']);self.assertFalse(result['claims']['gpu_exposed']);self.assertFalse(result['claims']['optional_bag_read'])
 def test_contract_freezes_parents_gauge_resources_and_primary_only(self):
  self.assertEqual(self.policy['selection']['planned_denominator'],1);self.assertFalse(self.policy['selection']['optional_authorized'])
  self.assertEqual(self.policy['gauge']['probe_count'],16);self.assertEqual(self.policy['gauge']['source'],'RAW_NATIVE_JGAUGESWL_CSV')
  self.assertTrue(self.policy['gauge']['updated_each_solver_step_before_gauge_compute'])
  self.assertEqual(self.policy['resource']['wall_timeout_seconds'],5400);self.assertEqual(self.policy['resource']['minimum_dynamic_free_vram_bytes'],6442450944);self.assertEqual(self.policy['resource']['maximum_output_bytes'],1073741824)
  self.assertEqual(len(self.policy['parents']),10)
 def test_premature_candidate_or_authorization_rejected(self):
  changed=copy.deepcopy(self.policy);changed['candidate_finalization_input']['candidate_sha256']='0'*64
  with self.assertRaises(gate.GateV5Error):gate.validate(changed)
  changed=copy.deepcopy(self.policy);changed['authorization']['solver_execution_authorized']=True
  with self.assertRaises(gate.GateV5Error):gate.validate(changed)
 def test_profile_is_exact_and_only_output_is_writable(self):
  rendered=profile.render_profile(profile.TEMPLATE_PATH.read_text(),profile.fixture_replacements())
  self.assertIn('deny /dev/nvidia-uvm-tools rw,',rendered)
  bad=profile.fixture_replacements();bad['NVIDIAUVM']='/dev/nvidia-uvm-tools'
  with self.assertRaises(profile.ProfileV5Error):profile.render_profile(profile.TEMPLATE_PATH.read_text(),bad)
  with self.assertRaises(profile.ProfileV5Error):profile.render_profile(profile.TEMPLATE_PATH.read_text()+'\n/tmp/** rw,\n',profile.fixture_replacements())
 def test_supervisor_requires_fresh_targets_and_hard_stops(self):
  result=supervisor.self_check();self.assertTrue(result['run_once_only']);self.assertTrue(result['finally_unload_required']);self.assertFalse(result['runtime_attempted'])
  with self.assertRaises(supervisor.SupervisorV5Error):supervisor.run_one_shot()
  with self.assertRaises(supervisor.SupervisorV5Error):supervisor.validate_fresh_targets({'partial_root':'/a','final_root':'/b','start_receipt':'/c','final_receipt':'/d','failure_receipt':'/e'},lambda p:p=='/c')
 def test_no_static_runtime_surface(self):
  for filename in ('r8_liquid_s5b0_replay_execution_gate_v5.py','r8_liquid_s5b0_profile_generator_v5.py','r8_liquid_s5b0_replay_supervisor_v5.py'):
   tree=ast.parse((SCRIPTS/filename).read_text());imports={a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names};self.assertFalse(imports&{'subprocess','socket'})
if __name__=='__main__':unittest.main()
