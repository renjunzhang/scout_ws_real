#!/usr/bin/env python3
"""Static-only v5 validator; it intentionally exposes no runtime entrypoint."""
from __future__ import annotations
import argparse,hashlib,json,sys
from pathlib import Path
from typing import Any,Mapping,Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
POLICY_PATH=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_contract_v5.json'
POLICY_SCHEMA=ROOT/'schema/target_host_s5b0_replay_execution_contract_v5.json'
RECEIPT_SCHEMA=ROOT/'schema/target_host_s5b0_replay_execution_receipt_v5.json'
PROFILE_GENERATOR=ROOT/'scripts/r8_liquid_s5b0_profile_generator_v5.py'
class GateV5Error(ValueError): pass
def sha(path:Path)->str:return hashlib.sha256(path.read_bytes()).hexdigest()
def deep_closed(node:Any)->None:
    if isinstance(node,dict):
        if node.get('type')=='object' and node.get('additionalProperties') is not False: raise GateV5Error('schema object is not closed')
        for child in node.values():deep_closed(child)
    elif isinstance(node,list):
        for child in node:deep_closed(child)
def load_policy()->tuple[dict[str,Any],str]:
    policy=json.loads(POLICY_PATH.read_bytes()); schema=json.loads(POLICY_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema); deep_closed(schema); Draft202012Validator(schema).validate(policy)
    return policy,sha(POLICY_PATH)
def validate(policy:Mapping[str,Any])->None:
    if policy['status']!='NOT_ADMITTED_FRESH_PATCHED_CANDIDATE_AND_EXACT_AUTHORIZATION_REQUIRED':raise GateV5Error('status drift')
    if policy['selection']['planned_denominator']!=1 or policy['selection']['optional_authorized'] or policy['selection']['c2_authorized'] or policy['selection']['optional_bag_read']:raise GateV5Error('selection drift')
    c=policy['candidate_finalization_input']; r=policy['replay_finalization_input']
    if c['finalized'] or any(c[k] is not None for k in ('candidate_path','candidate_sha256','build_receipt_path','build_receipt_sha256','static_audit_receipt_path','static_audit_receipt_sha256')):raise GateV5Error('candidate was prematurely materialized')
    if r['finalized'] or any(v is not None for k,v in r.items() if k!='finalized'):raise GateV5Error('replay was prematurely materialized')
    if any(policy['authorization'].values()):raise GateV5Error('authorization was prematurely set')
    if policy['gauge']['probe_count']!=16 or policy['gauge']['source']!='RAW_NATIVE_JGAUGESWL_CSV' or not policy['gauge']['updated_each_solver_step_before_gauge_compute']:raise GateV5Error('gauge contract drift')
    if policy['resource']!={'wall_timeout_seconds':5400,'minimum_dynamic_free_vram_bytes':6442450944,'maximum_output_bytes':1073741824,'monitor_interval_seconds':10,'monitor_interval_max_seconds':30,'candidate_execution_count':1}:raise GateV5Error('resource contract drift')
def static_receipt(policy:Mapping[str,Any], policy_sha:str)->dict[str,Any]:return {'schema_version':'smpcc-r8-liquid-s5b0-replay-execution-receipt-v5','document_type':'SMPCC_R8_LIQUID_S5B0_REPLAY_EXECUTION_RECEIPT_V5','status':policy['status'],'policy_sha256':policy_sha,'candidate':{'finalized':False,'capability':policy['candidate_finalization_input']['capability']},'replay':{'identity_materialized':False,'profile_materialized':False,'raw_gauge_csv_read':False},'claims':{'files_written':False,'solver_executed':False,'candidate_executed':False,'gpu_exposed':False,'sudo_used':False,'apparmor_loaded':False,'optional_bag_read':False,'network_used':False}}
def self_check()->dict[str,Any]:
    policy,policy_sha=load_policy(); validate(policy); rs=json.loads(RECEIPT_SCHEMA.read_bytes()); Draft202012Validator.check_schema(rs); deep_closed(rs); receipt=static_receipt(policy,policy_sha); Draft202012Validator(rs).validate(receipt)
    return {**receipt,'schemas_deep_closed':True,'profile_generator_sha256':sha(PROFILE_GENERATOR)}
def main(argv:Sequence[str]|None=None)->int:
    p=argparse.ArgumentParser();p.add_argument('command',choices=('self-check',));p.parse_args(argv)
    try: print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
    except Exception as exc:print(json.dumps({'status':'FAIL_S5B0_V5_STATIC_GATE','error':str(exc)},sort_keys=True),file=sys.stderr);return 2
if __name__=='__main__':raise SystemExit(main())
