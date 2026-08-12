#!/usr/bin/env python3
"""Injected-backend tests for v7; no privileged command can be reached."""
from __future__ import annotations
import ast,copy,importlib.util,json,sys,unittest
from pathlib import Path
from unittest.mock import patch
from jsonschema import Draft202012Validator,ValidationError
ROOT=Path(__file__).resolve().parent.parent;S=ROOT/'scripts'
def load(name,file):
 spec=importlib.util.spec_from_file_location(name,S/file);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;assert spec.loader;spec.loader.exec_module(m);return m
v7=load('s5b0_v7_test','r8_liquid_s5b0_replay_runtime_supervisor_v7.py')
class Fake:
 def __init__(self,fail=''):self.fail=fail;self.calls=[]
 def run(self,argv,**kwargs):self.calls.append((list(argv),kwargs));return 1 if self.fail and self.fail in ' '.join(argv) else 0
 def aa_status_contains(self,name):return self.fail!='verify' and not any('apparmor_parser -K -T -R' in ' '.join(x[0]) for x in self.calls)
 def resource(self):return {'free_vram_bytes':6442450944,'output_bytes':1,'qc':{'raw_names':[f'GaugesSwl_s5b0_p{i:02d}.csv' for i in range(16)],'invalid_ratio':0.0,'particles':9078,'nout':0,'finite':True,'motion':True,'xid_count':0,'output_bytes':1}}
 def xid_count(self):return 0
class V7(unittest.TestCase):
 def setUp(self):self.policy,self.ps=v7.read_policy()
 def test_schema_closed_default_deny(self):
  for path in (v7.SCHEMA,v7.RECEIPT_SCHEMA):
   s=json.loads(path.read_bytes());Draft202012Validator.check_schema(s);v7.deep_closed(s)
  x=copy.deepcopy(self.policy);x['x']=1
  with self.assertRaises(ValidationError):Draft202012Validator(json.loads(v7.SCHEMA.read_bytes())).validate(x)
  r=v7.self_check();self.assertFalse(r['execute_cli_exposed']);self.assertFalse(r['candidate_executed'])
 def test_static_contract_has_default_deny_and_exact_devices(self):
  self.assertEqual(self.policy['selection']['planned_denominator'],1);self.assertFalse(self.policy['selection']['optional_authorized']);self.assertEqual(self.policy['execution']['nvidia_devices'],['/dev/nvidia0','/dev/nvidiactl','/dev/nvidia-uvm']);self.assertEqual(self.policy['limits']['wall_timeout_seconds'],5400)
 def test_runtime_input_is_unmaterialized(self):
  self.assertFalse(self.policy['runtime_input']['finalized']);self.assertTrue(all(v is None for k,v in self.policy['runtime_input'].items() if k!='finalized'))
 def test_injected_load_failure_finally_unloads_and_sudo_k(self):
  final={'profile_name':'r8-liquid-s5b0-test-profile','candidate_path':'/fake/candidate','candidate_sha256':'0'*64,'build_receipt_path':'/fake/build','build_receipt_sha256':'1'*64,'static_audit_receipt_path':'/fake/static','static_audit_receipt_sha256':'2'*64,'stage_root':'/fake/stage.partial'};fake=Fake('apparmor_parser -K -T -a')
  with patch.object(v7,'validate_final'),patch.object(v7,'exact_token'),patch.object(v7,'command_plan',return_value={'load':['apparmor_parser','-K','-T','-a'],'unload':['apparmor_parser','-K','-T','-R'],'invoke':['solver'],'sudo_k':['sudo','-k']}),patch.object(v7,'_file_identity'),patch.object(v7.staging,'materialize'):
   r=v7.execute_once(final,Path('/token'),last_t_s=2,stage_sources={},stage_evidence={},backend=fake)
  self.assertEqual(r['status'],'STOP_AND_PRESERVE_EVIDENCE');self.assertIn('FAILURE_PRESERVED',r['events']);self.assertIn('sudo -k',' '.join(' '.join(x[0]) for x in fake.calls))
 def test_injected_postflight_failure_finally_unloads(self):
  final={'profile_name':'r8-liquid-s5b0-test-profile','candidate_path':'/fake/candidate','candidate_sha256':'0'*64,'build_receipt_path':'/fake/build','build_receipt_sha256':'1'*64,'static_audit_receipt_path':'/fake/static','static_audit_receipt_sha256':'2'*64,'stage_root':'/fake/stage.partial'};fake=Fake()
  with patch.object(v7,'validate_final'),patch.object(v7,'exact_token'),patch.object(v7,'command_plan',return_value={'load':['load'],'unload':['unload'],'invoke':['solver'],'sudo_k':['sudo','-k']}),patch.object(v7,'_file_identity'),patch.object(v7.staging,'materialize'),patch.object(v7.v6,'validate_postflight',side_effect=v7.V7Error('bad qc')):
   r=v7.execute_once(final,Path('/token'),last_t_s=2,stage_sources={},stage_evidence={},backend=fake)
  self.assertEqual(r['status'],'STOP_AND_PRESERVE_EVIDENCE');calls='\n'.join(' '.join(x[0]) for x in fake.calls);self.assertIn('solver',calls);self.assertIn('unload',calls);self.assertIn('sudo -k',calls)
 def test_real_subprocess_backend_is_not_exposed_by_cli(self):
  tree=ast.parse((S/'r8_liquid_s5b0_replay_runtime_supervisor_v7.py').read_text());main=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='main');text=ast.unparse(main);self.assertNotIn('execute_once(',text)
if __name__=='__main__':unittest.main()
