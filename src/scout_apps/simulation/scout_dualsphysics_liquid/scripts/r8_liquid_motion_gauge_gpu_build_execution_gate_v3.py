#!/usr/bin/env python3
"""Default-deny v3 admission gate for a future isolated motion-Gauge build.

This module is deliberately safe to run without authorization: self-check only
reads repository contracts.  Real phases require both flags and an exact policy
hash token; the privileged AppArmor lifecycle belongs solely to the v3 outer
supervisor.
"""
from __future__ import annotations
import argparse, hashlib, json, os, stat, sys
from pathlib import Path
from typing import Any, Mapping, Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode = True
ROOT=Path(__file__).resolve().parent.parent
POLICY=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v3.json'
SCHEMA=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_policy_v3.json'
RECEIPT_SCHEMA=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_receipt_v3.json'
PHASES=('source-copy','patch','wrapper','build','static-audit')
PARENT_PATHS={
 'v2_policy':ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v2.json',
 'g1_gate':ROOT/'scripts/r8_liquid_target_u3_gpu_build_gate_v1.py',
 'g2_gate':ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v2.py',
 'patch_gate':ROOT/'scripts/r8_liquid_motion_attached_gauge_patch_gate_v2.py'}
class GateError(RuntimeError): pass
def digest(raw:bytes)->str: return hashlib.sha256(raw).hexdigest()
def identity(path:Path)->dict[str,Any]:
 st=os.lstat(path)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1: raise GateError(f'unsafe regular file: {path}')
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  before=os.fstat(fd); h=hashlib.sha256()
  while b:=os.read(fd,1<<20): h.update(b)
  after=os.fstat(fd)
 finally: os.close(fd)
 if (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns): raise GateError(f'changed while hashing: {path}')
 return {'path':str(path),'sha256':h.hexdigest(),'mode_octal':f'{stat.S_IMODE(after.st_mode):04o}','size_bytes':after.st_size}
def deep_closed(node:Any, at:str='$')->None:
 if isinstance(node,dict):
  if node.get('type')=='object' and node.get('additionalProperties') is not False: raise GateError(f'open object schema: {at}')
  for k,v in node.items(): deep_closed(v,f'{at}/{k}')
 elif isinstance(node,list):
  for i,v in enumerate(node): deep_closed(v,f'{at}/{i}')
def read_policy()->tuple[dict[str,Any],str]:
 raw=POLICY.read_bytes(); value=json.loads(raw); schema=json.loads(SCHEMA.read_bytes()); receipt=json.loads(RECEIPT_SCHEMA.read_bytes())
 for item in (schema,receipt): Draft202012Validator.check_schema(item); deep_closed(item)
 Draft202012Validator(schema).validate(value)
 return value,digest(raw)
def validate_static(policy:Mapping[str,Any])->dict[str,Any]:
 for key,path in PARENT_PATHS.items():
  if identity(path)['sha256']!=policy['parents'][key]: raise GateError(f'parent drift: {key}')
 contracts=policy['contracts']; resources=policy['resources']
 if (contracts['source_entries'],contracts['patch_changed'],contracts['patch_unchanged'],contracts['wrapper_bytes'],contracts['objects'],contracts['static_commands']) != (352,6,346,84,131,557): raise GateError('cardinality drift')
 if resources['parallel_jobs']!=1 or resources['make_limit']!=1 or not 10<=resources['monitor_interval_seconds']<=30: raise GateError('parallel/monitor drift')
 if resources['wall_timeout_seconds']!=5400 or resources['cpu_limit_seconds']!=5400 or resources['address_space_limit_bytes']!=8589934592: raise GateError('resource limit drift')
 suffix='\n'.join(contracts['make_argv_suffix'])
 if '-j1' not in suffix or 'g++-11' not in suffix or 'cuda-12.8' not in suffix or 'compute_120' not in suffix or 'sm_120' not in suffix or '-j2' in suffix or '-j4' in suffix: raise GateError('frozen Make identity drift')
 for role,item in policy['profiles'].items():
  path=ROOT/item['path']; data=identity(path); text=path.read_text()
  if f"profile {item['name']} " not in text: raise GateError(f'profile name drift: {role}')
  if any(x in text for x in ('/dev/nvidia','network inet stream','network inet6 stream','flags=(unconfined)','*.o')): raise GateError(f'profile forbidden surface: {role}')
  if role=='static_audit' and text.count('.o r,')!=131: raise GateError('static audit does not enumerate 131 objects')
  if role=='patch' and '/newroot/work/tmp/' not in text: raise GateError('patch tmp pre-pivot delta absent')
 if policy['authorization']['default_authorized'] or policy['authorization']['runtime_smoke_included']: raise GateError('default authorization/runtime drift')
 return {'parents':len(PARENT_PATHS),'profiles':4,'source_entries':352,'patch_changed':6,'objects':131,'static_commands':557}
def safe_new(path:Path,data:bytes,mode:int=0o640)->dict[str,Any]:
 """Runtime primitive; only called by future authorized phase implementation."""
 parent=path.parent
 for part in (parent,*parent.parents):
  if part==part.parent: break
  if os.path.lexists(part) and stat.S_ISLNK(os.lstat(part).st_mode): raise GateError(f'symlink component: {part}')
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,mode)
 try:
  os.fchmod(fd,mode); os.write(fd,data); os.fsync(fd)
 finally: os.close(fd)
 dfd=os.open(parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try: os.fsync(dfd)
 finally: os.close(dfd)
 return identity(path)
def validate_token(path:Path,policy_sha:str)->None:
 raw=path.read_bytes(); st=os.lstat(path)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1 or st.st_size>16384: raise GateError('unsafe authorization token')
 value=json.loads(raw)
 if set(value)!={'policy_sha256','user_authorized'} or value['policy_sha256']!=policy_sha or value['user_authorized'] is not True: raise GateError('exact policy authorization missing')
def self_check()->dict[str,Any]:
 policy,sha=read_policy(); counts=validate_static(policy)
 root=Path(policy['campaign']['attempt_root']); prefix=Path(policy['campaign']['audit_prefix'])
 return {'status':'PASS_V3_STATIC_EXECUTION_ADMISSION_NOT_AUTHORIZED','policy_sha256':sha,'counts':counts,'attempt_root_absent':not os.path.lexists(root),'receipt_prefix_absent':not any(prefix.parent.glob(prefix.name+'*')),'files_written':False,'sudo_used':False,'profile_loaded':False,'make_run':False,'candidate_executed':False,'gpu_exposed':False}
def main(argv:Sequence[str]|None=None)->int:
 p=argparse.ArgumentParser();p.add_argument('command',choices=('self-check',*PHASES));p.add_argument('--execute',action='store_true');p.add_argument('--authorization-token-file');a=p.parse_args(argv)
 try:
  if a.command=='self-check':
   if a.execute or a.authorization_token_file: raise GateError('self-check rejects execution inputs')
   report=self_check()
  else:
   policy,sha=read_policy(); validate_static(policy)
   if not(a.execute and a.authorization_token_file): raise GateError('NOT_AUTHORIZED: exact execute flag and token required')
   validate_token(Path(a.authorization_token_file),sha)
   raise GateError('AUTHORIZED_TOKEN_ACCEPTED_BUT_PHASE_RUNNER_NOT_BOUND: supervisor must bind exact lifecycle')
 except Exception as e:
  print(json.dumps({'status':'FAIL_MOTION_GAUGE_GPU_BUILD_EXECUTION_V3','error':str(e)},sort_keys=True),file=sys.stderr);return 2
 print(json.dumps(report,sort_keys=True,separators=(',',':')));return 0
if __name__=='__main__': raise SystemExit(main())
