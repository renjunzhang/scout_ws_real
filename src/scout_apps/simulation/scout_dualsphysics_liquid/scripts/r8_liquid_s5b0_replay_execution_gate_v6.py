#!/usr/bin/env python3
"""Default-deny S5B0 v6 materialization and one-shot execution contract.

Public CLI is static-only.  Future privileged code can consume the pure plans
only after it has supplied exact finalized candidate/profile identities.
"""
from __future__ import annotations
import argparse, hashlib, json, math, os, re, sys
from pathlib import Path
from typing import Any, Mapping, Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
POLICY=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v6.json'
SCHEMA=ROOT/'schema/target_host_s5b0_replay_execution_policy_v6.json'
RECEIPT_SCHEMA=ROOT/'schema/target_host_s5b0_replay_execution_receipt_v6.json'
V5=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_contract_v5.json'
STAGING_V4=ROOT/'scripts/r8_liquid_s5b0_staging_case_materializer_v4.py'
PROFILE=ROOT/'scripts/r8_liquid_s5b0_profile_generator_v5.py'
class GateV6Error(ValueError):pass
def sha(path:Path)->str:return hashlib.sha256(path.read_bytes()).hexdigest()
def deep_closed(node:Any)->None:
 if isinstance(node,dict):
  kind=node.get('type')
  if (kind=='object' or isinstance(kind,list) and 'object' in kind) and node.get('additionalProperties') is not False:raise GateV6Error('open schema object')
  for value in node.values():deep_closed(value)
 elif isinstance(node,list):
  for value in node:deep_closed(value)
def load_policy()->tuple[dict[str,Any],str]:
 raw=POLICY.read_bytes(); value=json.loads(raw)
 for path in (SCHEMA,RECEIPT_SCHEMA):
  schema=json.loads(path.read_bytes());Draft202012Validator.check_schema(schema);deep_closed(schema)
 Draft202012Validator(json.loads(SCHEMA.read_bytes())).validate(value)
 return value,hashlib.sha256(raw).hexdigest()
def validate_static(policy:Mapping[str,Any])->None:
 if policy['status']!='DEFAULT_DENY_FRESH_CANDIDATE_REQUIRED':raise GateV6Error('status drift')
 if policy['selection']!={'attempt_id':'SIM-S1_CORE_H1_C1_Bsmooth_b01_r01','planned_denominator':1,'optional_authorized':False,'optional_bag_read':False,'c2_authorized':False}:raise GateV6Error('selection drift')
 if sha(V5)!=policy['parents']['v5_contract'] or sha(STAGING_V4)!=policy['parents']['staging_v4']:raise GateV6Error('parent drift')
 c=policy['candidate_input']; m=policy['materialization_input']; a=policy['authorization']
 if c['finalized'] or any(c[k] is not None for k in ('path','sha256','build_receipt_path','build_receipt_sha256','static_audit_receipt_path','static_audit_receipt_sha256')):raise GateV6Error('premature candidate')
 if m['finalized'] or any(v is not None for k,v in m.items() if k!='finalized'):raise GateV6Error('premature replay identity')
 if any(v is not None for k,v in a.items() if k!='user_authorized') or a['user_authorized']:raise GateV6Error('premature authorization')
 if policy['gauge']['probe_count']!=16 or policy['resources']['minimum_free_vram_bytes']!=6442450944 or policy['resources']['wall_timeout_seconds']!=5400:raise GateV6Error('frozen contract drift')
def _sha(value:object,label:str)->str:
 if not isinstance(value,str) or not re.fullmatch('[0-9a-f]{64}',value):raise GateV6Error('bad sha '+label)
 return value
def _path(value:object,label:str)->str:
 if not isinstance(value,str) or not value.startswith('/') or value!=os.path.normpath(value):raise GateV6Error('bad path '+label)
 return value
def validate_finalization(policy:Mapping[str,Any], final:Mapping[str,Any])->None:
 required={'candidate_path','candidate_sha256','build_receipt_path','build_receipt_sha256','static_audit_receipt_path','static_audit_receipt_sha256','capability','replay_id','stage_root','partial_root','final_root','start_receipt','final_receipt','failure_receipt','profile_name','profile_path','profile_sha256','authorization'}
 if set(final)!=required:raise GateV6Error('finalization keys differ')
 for key in ('candidate_path','build_receipt_path','static_audit_receipt_path','stage_root','partial_root','final_root','start_receipt','final_receipt','failure_receipt','profile_path'):_path(final[key],key)
 for key in ('candidate_sha256','build_receipt_sha256','static_audit_receipt_sha256','profile_sha256'):_sha(final[key],key)
 if final['capability']!='MOTION_ATTACHED_16_RAW_JGAUGESWL' or not re.fullmatch(r'r8-liquid-s5b0-[a-z0-9-]{8,80}',final['profile_name']) or not re.fullmatch(r'[a-z0-9_-]{12,120}',final['replay_id']):raise GateV6Error('capability/name drift')
 all_paths=[final[k] for k in ('stage_root','partial_root','final_root','start_receipt','final_receipt','failure_receipt','profile_path')]
 if len(set(all_paths))!=len(all_paths) or any('/u3_source_gpu_build_' in p for p in all_paths):raise GateV6Error('identity aliases/uses old candidate')
 auth=final['authorization']
 if set(auth)!={'policy_sha256','candidate_sha256','profile_sha256','user_authorized'} or auth!={'policy_sha256':sha(POLICY),'candidate_sha256':final['candidate_sha256'],'profile_sha256':final['profile_sha256'],'user_authorized':True}:raise GateV6Error('exact authorization absent')
def profile_replacements(final:Mapping[str,Any])->dict[str,str]:
 return {'PROFILE_NAME':final['profile_name'],'STAGED_CANDIDATE':final['stage_root']+'/runtime/candidate','DSPH_CONFIG':final['stage_root']+'/runtime/DsphConfig.xml','LIBCUDA':'/runtime/lib/libcuda.so.1','LIBNVIDIA_PTXJIT':'/runtime/lib/libnvidia-ptxjitcompiler.so.1','CASE_ROOT':final['stage_root']+'/case','RESTART_ROOT':final['stage_root']+'/restart','NVIDIA0':'/dev/nvidia0','NVIDIACTL':'/dev/nvidiactl','NVIDIAUVM':'/dev/nvidia-uvm','OUTPUT_ROOT':final['partial_root']+'/output'}
def solver_argv(policy:Mapping[str,Any],final:Mapping[str,Any],last_t_s:float)->list[str]:
 validate_finalization(policy,final)
 if not isinstance(last_t_s,(int,float)) or not math.isfinite(last_t_s) or last_t_s<=1:raise GateV6Error('bad solver path duration')
 tmax=policy['solver']['settled_time_s']+float(last_t_s)
 return [final['stage_root']+'/runtime/candidate',final['stage_root']+'/case/C1M_case',final['partial_root']+'/output','-gpu:0','-partbegin:901:901',final['stage_root']+'/restart',f'-tmax:{tmax:.15g}','-tout:0.05','-ompthreads:1','-stable:1','-vres:0','-cellmode:full','-cfl:0.1','-shifting:none','-viscoart:0.3','-sv:binx,info','-svres:1','-svtimers:0','-svdomainvtk:0','-saveposdouble:1','-nortimes:1','-createdirs:1','-csvsep:0']
def validate_postflight(policy:Mapping[str,Any],qc:Mapping[str,Any])->None:
 if set(qc)!={'raw_names','invalid_ratio','particles','nout','finite','motion','xid_count','output_bytes'}:raise GateV6Error('qc keys differ')
 if qc['raw_names']!=[f'GaugesSwl_s5b0_p{i:02d}.csv' for i in range(16)] or not isinstance(qc['invalid_ratio'],(int,float)) or qc['invalid_ratio']>0.001:raise GateV6Error('raw Gauge contract failed')
 if qc['particles']!=9078 or qc['nout']!=0 or qc['finite'] is not True or qc['motion'] is not True or qc['xid_count']!=0 or not isinstance(qc['output_bytes'],int) or qc['output_bytes']>1073741824:raise GateV6Error('particle/motion/output qc failed')
def static_receipt(policy:Mapping[str,Any],policy_sha:str)->dict[str,Any]:
 return {'schema_version':'smpcc-r8-liquid-s5b0-replay-execution-receipt-v6','document_type':'SMPCC_R8_LIQUID_S5B0_REPLAY_EXECUTION_RECEIPT_V6','status':'NOT_ADMITTED_FRESH_CANDIDATE_REQUIRED','policy_sha256':policy_sha,'phase':'STATIC','identity':{'replay_id':None,'stage_root':None,'partial_root':None,'final_root':None,'start_receipt':None,'final_receipt':None,'failure_receipt':None,'profile_name':None,'profile_path':None,'profile_sha256':None},'candidate':{'path':None,'sha256':None,'build_receipt_sha256':None,'static_audit_receipt_sha256':None,'capability':policy['candidate_input']['required_capability'],'disarmed':False},'gauge':{'raw_csv_count':0,'raw_csv_complete':False,'native_attached':False,'invalid_ratio':None},'resources':{'free_vram_bytes':None,'output_bytes':None,'xid_count':None,'sample_count':0},'lifecycle':{'profile_loaded':False,'profile_unloaded':False,'zero_residue':False,'receipts_o_excl_nofollow':True,'failure_preserved':False},'claims':{'files_written':False,'candidate_executed':False,'solver_executed':False,'gpu_exposed':False,'network_used':False,'sudo_used':False,'apparmor_loaded':False,'optional_bag_read':False}}
def self_check()->dict[str,Any]:
 policy,p=load_policy();validate_static(policy);receipt=static_receipt(policy,p);Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(receipt)
 return {**receipt,'schemas_deep_closed':True,'profile_generator_sha256':sha(PROFILE),'execution_attempted':False}
def main(argv:Sequence[str]|None=None)->int:
 ap=argparse.ArgumentParser();ap.add_argument('command',choices=('self-check',));ap.parse_args(argv)
 try:print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
 except Exception as exc:print(json.dumps({'status':'FAIL_S5B0_V6_STATIC','error':str(exc)},sort_keys=True),file=sys.stderr);return 2
if __name__=='__main__':raise SystemExit(main())
