#!/usr/bin/env python3
"""Static gate for v8; runtime inputs are intentionally absent by default."""
from __future__ import annotations
import argparse,hashlib,json,os,re,sys
from pathlib import Path
from typing import Any,Mapping,Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
POLICY=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v8.json';SCHEMA=ROOT/'schema/target_host_s5b0_replay_execution_policy_v8.json';RECEIPT=ROOT/'schema/target_host_s5b0_replay_execution_receipt_v8.json'
PARENTS={'v7_policy':ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v7.json','v7_runtime':ROOT/'scripts/r8_liquid_s5b0_replay_runtime_supervisor_v7.py','staging_v4':ROOT/'scripts/r8_liquid_s5b0_staging_case_materializer_v4.py','s6_policy':ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s6_real_runtime_delivery_policy_v6.json'}
class V8Error(ValueError):pass
def sha(p:Path)->str:return hashlib.sha256(p.read_bytes()).hexdigest()
def deep_closed(v:Any)->None:
 if isinstance(v,dict):
  if v.get('type')=='object' and v.get('additionalProperties') is not False:raise V8Error('open object schema')
  for x in v.values():deep_closed(x)
 elif isinstance(v,list):
  for x in v:deep_closed(x)
def read_policy()->tuple[dict[str,Any],str]:
 raw=POLICY.read_bytes();p=json.loads(raw)
 for f in (SCHEMA,RECEIPT):
  s=json.loads(f.read_bytes());Draft202012Validator.check_schema(s);deep_closed(s)
 Draft202012Validator(json.loads(SCHEMA.read_bytes())).validate(p);return p,hashlib.sha256(raw).hexdigest()
def validate_static(p:Mapping[str,Any])->None:
 if p['status']!='DEFAULT_DENY_EXACT_PRIMARY_RUNTIME_REQUIRED':raise V8Error('status')
 if any(sha(path)!=p['parents'][k] for k,path in PARENTS.items()):raise V8Error('parent drift')
 if p['selection']['planned_denominator']!=1 or any(p['selection'][k] for k in ('optional_bag_read','optional_authorized','c2_authorized')):raise V8Error('selection drift')
 r=p['runtime_input'];a=p['authorization']
 if r['finalized'] or any(v is not None for k,v in r.items() if k!='finalized') or a['user_authorized']:raise V8Error('premature runtime')
 if p['limits']!={'wall_seconds':5400,'kill_after_seconds':30,'sample_min_seconds':10,'sample_max_seconds':30,'free_vram_bytes':6442450944,'max_output_bytes':1073741824,'candidate_mode':'0400','raw_gauges':16,'particles':9078,'max_invalid_ratio':0.001}:raise V8Error('limit drift')
def validate_token(token:Mapping[str,Any],policy_sha:str,candidate_sha:str,profile_sha:str)->None:
 if token!={'policy_sha256':policy_sha,'candidate_sha256':candidate_sha,'profile_sha256':profile_sha,'user_authorized':True}:raise V8Error('exact token')
def self_check()->dict[str,Any]:
 p,h=read_policy();validate_static(p);return {'status':'PASS_S5B0_V8_STATIC_GATE_NOT_AUTHORIZED','policy_sha256':h,'files_written':False,'candidate_executed':False,'gpu_exposed':False,'optional_bag_read':False}
def main(argv:Sequence[str]|None=None)->int:
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check',));a.parse_args(argv)
 try:print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
 except Exception as e:print(json.dumps({'status':'FAIL_S5B0_V8_GATE','error':str(e)},sort_keys=True),file=sys.stderr);return 2
if __name__=='__main__':raise SystemExit(main())
