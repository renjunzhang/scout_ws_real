#!/usr/bin/env python3
"""Default-deny, executable S5B0 v7 backend with injectable command runner.

The CLI is intentionally self-check only. `execute_once` is reachable only by a
future caller holding all frozen runtime inputs and a matching authorization
token.  Tests inject FakeBackend; this turn never invokes it with real tools.
"""
from __future__ import annotations
import argparse, hashlib, importlib.util, json, os, stat, subprocess, sys
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent; S=ROOT/'scripts'
POLICY=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v7.json';SCHEMA=ROOT/'schema/target_host_s5b0_replay_execution_policy_v7.json';RECEIPT_SCHEMA=ROOT/'schema/target_host_s5b0_replay_execution_receipt_v7.json'
def load(name:str,file:str):
 spec=importlib.util.spec_from_file_location(name,S/file);assert spec and spec.loader;m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
v6=load('s5b0_v6_for_v7','r8_liquid_s5b0_replay_execution_gate_v6.py');v6sup=load('s5b0_v6sup_for_v7','r8_liquid_s5b0_replay_lifecycle_supervisor_v6.py');staging=load('s5b0_staging_for_v7','r8_liquid_s5b0_staging_case_materializer_v4.py')
class V7Error(RuntimeError):pass
class Backend(Protocol):
 def run(self,argv:Sequence[str],*,timeout:int|None=None,new_pgid:bool=False)->int:...
 def aa_status_contains(self,name:str)->bool:...
 def resource(self)->Mapping[str,Any]:...
 def xid_count(self)->int:...
class SubprocessBackend:
 def run(self,argv,*,timeout=None,new_pgid=False):return subprocess.run(list(argv),check=False,timeout=timeout,start_new_session=new_pgid).returncode
 def aa_status_contains(self,name):return subprocess.run(['/usr/sbin/aa-status'],capture_output=True,text=True,check=False).returncode==0 and name in subprocess.run(['/usr/sbin/aa-status'],capture_output=True,text=True,check=False).stdout
 def resource(self):raise V7Error('real resource provider must be injected')
 def xid_count(self):raise V7Error('real Xid provider must be injected')
def sha(path:Path)->str:return hashlib.sha256(path.read_bytes()).hexdigest()
def deep_closed(node:Any)->None:
 if isinstance(node,dict):
  kind=node.get('type')
  if (kind=='object' or isinstance(kind,list) and 'object' in kind) and node.get('additionalProperties') is not False:raise V7Error('open schema')
  for value in node.values():deep_closed(value)
 elif isinstance(node,list):
  for value in node:deep_closed(value)
def read_policy()->tuple[dict[str,Any],str]:
 raw=POLICY.read_bytes();p=json.loads(raw)
 for path in (SCHEMA,RECEIPT_SCHEMA):
  s=json.loads(path.read_bytes());Draft202012Validator.check_schema(s);deep_closed(s)
 Draft202012Validator(json.loads(SCHEMA.read_bytes())).validate(p);return p,hashlib.sha256(raw).hexdigest()
def validate_static(p:Mapping[str,Any])->None:
 if p['status']!='DEFAULT_DENY_EXACT_RUNTIME_AUTHORIZATION_REQUIRED':raise V7Error('status')
 if sha(ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v6.json')!=p['parents']['v6_policy'] or sha(S/'r8_liquid_s5b0_replay_execution_gate_v6.py')!=p['parents']['v6_gate'] or sha(S/'r8_liquid_s5b0_replay_lifecycle_supervisor_v6.py')!=p['parents']['v6_supervisor'] or sha(S/'r8_liquid_s5b0_staging_case_materializer_v4.py')!=p['parents']['staging_v4']:raise V7Error('parent drift')
 if p['selection']['planned_denominator']!=1 or any(p['selection'][k] for k in ('optional_authorized','optional_bag_read','c2_authorized')):raise V7Error('selection')
 if p['runtime_input']['finalized'] or any(v is not None for k,v in p['runtime_input'].items() if k!='finalized') or p['authorization']['user_authorized']:raise V7Error('premature runtime')
def exact_token(path:Path,policy_sha:str,final:Mapping[str,Any])->None:
 st=os.lstat(path)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1 or st.st_size>16384:raise V7Error('unsafe token')
 value=json.loads(path.read_bytes());expected={'policy_sha256':policy_sha,'candidate_sha256':final['candidate_sha256'],'profile_sha256':final['profile_sha256'],'user_authorized':True}
 if value!=expected:raise V7Error('exact authorization token mismatch')
def _file_identity(path:Path,expected_sha:str,mode:str|None=None)->dict[str,Any]:
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  a=os.fstat(fd);h=hashlib.sha256()
  while b:=os.read(fd,1<<20):h.update(b)
  z=os.fstat(fd)
 finally:os.close(fd)
 if not stat.S_ISREG(a.st_mode) or a.st_nlink!=1 or (a.st_dev,a.st_ino,a.st_size,a.st_mtime_ns)!=(z.st_dev,z.st_ino,z.st_size,z.st_mtime_ns) or h.hexdigest()!=expected_sha or mode and f'{stat.S_IMODE(z.st_mode):04o}'!=mode:raise V7Error('file identity drift')
 return {'inode':z.st_ino,'size_bytes':z.st_size,'mode':f'{stat.S_IMODE(z.st_mode):04o}','sha256':expected_sha}
def validate_final(final:Mapping[str,Any],policy_sha:str)->None:
 required={'candidate_path','candidate_sha256','build_receipt_path','build_receipt_sha256','static_audit_receipt_path','static_audit_receipt_sha256','replay_id','stage_root','partial_root','final_root','profile_name','profile_path','profile_sha256','start_receipt','final_receipt','failure_receipt'}
 if set(final)!=required:raise V7Error('final keys')
 bridge={**final,'capability':'MOTION_ATTACHED_16_RAW_JGAUGESWL','authorization':{'policy_sha256':sha(ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v6.json'),'candidate_sha256':final['candidate_sha256'],'profile_sha256':final['profile_sha256'],'user_authorized':True}}
 v6.validate_finalization(v6.load_policy()[0],bridge)
 if any(os.path.lexists(final[k]) for k in ('stage_root','partial_root','final_root','profile_path','start_receipt','final_receipt','failure_receipt')):raise V7Error('target exists')
 if final['candidate_sha256']=='cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f':raise V7Error('old candidate forbidden')
def command_plan(p:Mapping[str,Any],final:Mapping[str,Any],last_t:float)->dict[str,list[str]]:
 profile_text=load('profile_for_v7','r8_liquid_s5b0_profile_generator_v5.py').render_profile((ROOT/'config/apparmor_drafts/r8-liquid-s5b0-replay-v5.profile.template').read_text(),v6.profile_replacements({**final,'authorization':{'policy_sha256':sha(ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v6.json'),'candidate_sha256':final['candidate_sha256'],'profile_sha256':final['profile_sha256'],'user_authorized':True},'capability':'MOTION_ATTACHED_16_RAW_JGAUGESWL'}))
 if hashlib.sha256(profile_text.encode()).hexdigest()!=final['profile_sha256']:raise V7Error('profile hash')
 argv=v6.solver_argv(v6.load_policy()[0],{**final,'capability':'MOTION_ATTACHED_16_RAW_JGAUGESWL','authorization':{'policy_sha256':sha(ROOT/'config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v6.json'),'candidate_sha256':final['candidate_sha256'],'profile_sha256':final['profile_sha256'],'user_authorized':True}},last_t)
 return {'load':[ *p['execution']['load_prefix'],final['profile_path']], 'unload':[ *p['execution']['unload_prefix'],final['profile_path']], 'invoke':[ *p['execution']['sudo_setpriv_prefix'],'/usr/bin/aa-exec','-p',final['profile_name'],'--','/usr/bin/bwrap','--unshare-net','--dev-bind','/dev/nvidia0','/dev/nvidia0','--dev-bind','/dev/nvidiactl','/dev/nvidiactl','--dev-bind','/dev/nvidia-uvm','/dev/nvidia-uvm','--',*argv], 'sudo_k':['/usr/bin/sudo','-k']}
def execute_once(final:Mapping[str,Any],token:Path,*,last_t_s:float,stage_sources:Mapping[str,Any],stage_evidence:Mapping[str,Any],backend:Backend)->dict[str,Any]:
 p,ps=read_policy();validate_static(p);validate_final(final,ps);exact_token(token,ps,final);plan=command_plan(p,final,last_t_s);events=[];loaded=False;rc=None;failure=None
 try:
  _file_identity(Path(final['candidate_path']),final['candidate_sha256'],'0400');_file_identity(Path(final['build_receipt_path']),final['build_receipt_sha256']);_file_identity(Path(final['static_audit_receipt_path']),final['static_audit_receipt_sha256'])
  # This is the mandatory REAL staging/settled-clone binding; no solver is run by materializer.
  staging.materialize(Path(final['stage_root']),expected_stage_root=Path(final['stage_root']),sources=stage_sources,restart_part_index=901,settled_time_s=45.05001991890928,solver_tail_s=1.0,mode='REAL',evidence=stage_evidence);events.append('STAGING_V4_REAL')
  if backend.run(plan['load'])!=0 or not backend.aa_status_contains(final['profile_name']):raise V7Error('profile load/verify')
  loaded=True;events.append('PROFILE_LOADED')
  r=backend.resource()
  if r['free_vram_bytes']<p['limits']['minimum_free_vram_bytes'] or r['output_bytes']>p['limits']['maximum_output_bytes'] or backend.xid_count()!=0:raise V7Error('preflight resource/Xid')
  rc=backend.run(plan['invoke'],timeout=p['limits']['wall_timeout_seconds'],new_pgid=True);events.append('SOLVER_INVOKED_ONCE')
  if rc!=0:raise V7Error('solver rc')
  v6.validate_postflight(v6.load_policy()[0],r['qc']);events.append('POSTFLIGHT_PASS')
  _file_identity(Path(final['candidate_path']),final['candidate_sha256'],'0400');return {'status':'S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY','events':events,'rc':rc}
 except Exception as exc:failure=str(exc);events.append('FAILURE_PRESERVED')
 finally:
  if loaded: backend.run(plan['unload'])
  profile_name=final.get('profile_name','<unmaterialized-profile>')
  zero=not backend.aa_status_contains(profile_name);backend.run(plan['sudo_k']);events.append('FINALLY_UNLOAD_ZERO_RESIDUE' if zero else 'FINALLY_ZERO_RESIDUE_FAILED')
 if failure: return {'status':'STOP_AND_PRESERVE_EVIDENCE','events':events,'error':failure,'rc':rc,'zero_residue':zero}
 raise V7Error('unreachable')
def self_check()->dict[str,Any]:
 p,ps=read_policy();validate_static(p);return {'status':'PASS_S5B0_V7_STATIC_RUNTIME_BACKEND_NOT_AUTHORIZED','policy_sha256':ps,'subprocess_backend_available':True,'execute_cli_exposed':False,'files_written':False,'sudo_used':False,'profile_loaded':False,'candidate_executed':False,'gpu_exposed':False}
def main(argv:Sequence[str]|None=None)->int:
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check',));a.parse_args(argv)
 try:print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
 except Exception as e:print(json.dumps({'status':'FAIL_S5B0_V7','error':str(e)},sort_keys=True),file=sys.stderr);return 2
if __name__=='__main__':raise SystemExit(main())
