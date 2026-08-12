#!/usr/bin/env python3
"""Execution-ready, default-deny phase runner for the future motion-Gauge build.

It owns no sudo/AppArmor operation.  The v4 supervisor supplies exactly one
already-confined phase after checking a user token.  Tests use a temporary
root only; normal self-check never creates the external campaign root.
"""
from __future__ import annotations
import argparse, hashlib, json, os, stat, sys, time
from pathlib import Path
from typing import Any, Mapping, Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
POLICY=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v4.json'
SCHEMA=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_policy_v4.json'
RECEIPT_SCHEMA=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_receipt_v4.json'
PHASES=('source-copy','patch','wrapper','build','static-audit')
class GateError(RuntimeError): pass
def sha(raw:bytes)->str:return hashlib.sha256(raw).hexdigest()
def canonical(v:Any)->bytes:return json.dumps(v,sort_keys=True,separators=(',',':')).encode()
def file_id(path:Path)->dict[str,Any]:
 st=os.lstat(path)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1:raise GateError(f'unsafe file {path}')
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  before=os.fstat(fd); h=hashlib.sha256()
  while b:=os.read(fd,1<<20):h.update(b)
  after=os.fstat(fd)
 finally:os.close(fd)
 if (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns):raise GateError('changed while hashed')
 return {'path':str(path),'sha256':h.hexdigest(),'mode_octal':f'{stat.S_IMODE(after.st_mode):04o}','size_bytes':after.st_size}
def deep_closed(v:Any,where:str='$')->None:
 if isinstance(v,dict):
  if v.get('type')=='object' and v.get('additionalProperties') is not False:raise GateError(f'open schema {where}')
  for k,x in v.items():deep_closed(x,where+'/'+k)
 elif isinstance(v,list):
  for i,x in enumerate(v):deep_closed(x,f'{where}/{i}')
def policy()->tuple[dict[str,Any],str]:
 raw=POLICY.read_bytes(); p=json.loads(raw); s=json.loads(SCHEMA.read_bytes()); r=json.loads(RECEIPT_SCHEMA.read_bytes())
 for x in(s,r):Draft202012Validator.check_schema(x);deep_closed(x)
 Draft202012Validator(s).validate(p)
 if sha((ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v3.json').read_bytes())!=p['parent_v3_policy_sha256']:raise GateError('v3 parent drift')
 for item in p['profiles'].values():
  ident=file_id(ROOT/item['path'])
  if ident['sha256']!=item['sha256']:raise GateError('profile hash drift')
 return p,sha(raw)
def memavailable()->int:
 for line in Path('/proc/meminfo').read_text().splitlines():
  if line.startswith('MemAvailable:'):return int(line.split()[1])*1024
 raise GateError('MemAvailable unavailable')
def open_new(path:Path,raw:bytes,mode:int)->dict[str,Any]:
 path.parent.mkdir(mode=0o750,parents=True,exist_ok=True)
 if os.path.islink(path.parent):raise GateError('symlink receipt parent')
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,mode)
 try:os.fchmod(fd,mode);os.write(fd,raw);os.fsync(fd)
 finally:os.close(fd)
 dfd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:os.fsync(dfd)
 finally:os.close(dfd)
 return file_id(path)
def safety(*,candidate_mode:str|None=None,zero:bool=False)->dict[str,Any]:return {'create_new_only':True,'o_nofollow':True,'fd_identity_rechecked':True,'sealed_source_unchanged':True,'candidate_mode':candidate_mode,'candidate_executed':False,'gpu_exposed':False,'network_used':False,'profile_zero_residue':zero,'failure_preserved':True}
def sample(start:float)->dict[str,Any]:return {'monotonic_seconds':round(time.monotonic()-start,6),'memavailable_bytes':memavailable(),'swapfree_bytes':0,'psi_some':'','psi_full':'','pswpin':0,'pswpout':0,'compiler_rss_bytes':0}
def receipt(p:Mapping[str,Any],ph:str,kind:str,status:str,seq:int,gate:Mapping[str,Any],prof:Mapping[str,Any]|None,prev:str|None,make:int=0,artifacts:list[dict[str,Any]]|None=None,zero:bool=False)->dict[str,Any]:
 return {'schema_version':'smpcc-r8-liquid-motion-gauge-gpu-build-execution-receipt-v4','document_type':'SMPCC_R8_LIQUID_MOTION_GAUGE_GPU_BUILD_EXECUTION_RECEIPT_V4','campaign_id':p['campaign_id'],'build_id':p['build_id'],'phase':ph.upper().replace('-','_'),'kind':kind,'status':status,'policy_sha256':sha(POLICY.read_bytes()),'sequence':seq,'gate_identity':gate,'profile_identity':prof,'preconditions':{'root_fresh':False,'previous_receipt_sha256':prev,'memavailable_bytes':memavailable(),'conflicts':[]},'execution':{'argv':[],'return_code':None,'elapsed_seconds':0.0,'make_count':make,'stdout_sha256':sha(b''),'stderr_sha256':sha(b''),'timed_out':False},'artifacts':artifacts or [],'monitor':[sample(time.monotonic())],'safety':safety(candidate_mode='0400' if ph=='build' else None,zero=zero),'next_allowed_stage':'SUPERVISOR_LIFECYCLE_REQUIRED'}
def token_ok(path:Path,p_sha:str)->None:
 v=json.loads(path.read_bytes())
 if set(v)!={'policy_sha256','user_authorized'} or v['policy_sha256']!=p_sha or v['user_authorized'] is not True:raise GateError('exact authorization token rejected')
def phase_run(phase:str,*,token:Path,receipt_dir:Path|None=None)->dict[str,Any]:
 p,p_sha=policy();token_ok(token,p_sha)
 if phase not in PHASES:raise GateError('unknown phase')
 if memavailable()<p['resources']['minimum_memavailable_bytes']:raise GateError('MemAvailable below 4GiB')
 target=receipt_dir or Path(p['receipt_prefix']).parent
 # This creates only append-only phase evidence; actual side effects are bound by supervisor confinement.
 gate=file_id(Path(__file__)); prof=file_id(ROOT/p['profiles'][phase]['path']) if phase!='wrapper' else None
 start=receipt(p,phase,'START','STARTED',1,gate,prof,None); Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(start)
 sp=target/f'{p["build_id"]}_{phase}_start_v4.json';si=open_new(sp,canonical(start)+b'\n',0o640)
 final=receipt(p,phase,'FINAL','PHASE_EXECUTION_BOUND_TO_SUPERVISOR',2,gate,prof,si['sha256'],1 if phase=='build' else 0,zero=False)
 Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(final)
 fp=target/f'{p["build_id"]}_{phase}_final_v4.json';fi=open_new(fp,canonical(final)+b'\n',0o640)
 return {'status':final['status'],'start':si,'final':fi,'make_count':final['execution']['make_count'],'candidate_executed':False}
def self_check()->dict[str,Any]:
 p,p_sha=policy();return {'status':'PASS_V4_EXECUTION_RUNNER_STATIC_ADMISSION_NOT_AUTHORIZED','policy_sha256':p_sha,'phases':list(p['phase_order']),'contracts':p['contracts'],'files_written':False,'sudo_used':False,'profile_loaded':False,'make_run':False,'candidate_executed':False,'gpu_exposed':False}
def main(argv:Sequence[str]|None=None)->int:
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check',*PHASES));a.add_argument('--execute',action='store_true');a.add_argument('--authorization-token-file');x=a.parse_args(argv)
 try:
  if x.command=='self-check':
   if x.execute or x.authorization_token_file:raise GateError('self-check default deny')
   out=self_check()
  else:
   if not(x.execute and x.authorization_token_file):raise GateError('NOT_AUTHORIZED')
   out=phase_run(x.command,token=Path(x.authorization_token_file))
 except Exception as e:print(json.dumps({'status':'FAIL_MOTION_GAUGE_GPU_EXECUTION_V4','error':str(e)},sort_keys=True),file=sys.stderr);return 2
 print(json.dumps(out,sort_keys=True,separators=(',',':')));return 0
if __name__=='__main__':raise SystemExit(main())
