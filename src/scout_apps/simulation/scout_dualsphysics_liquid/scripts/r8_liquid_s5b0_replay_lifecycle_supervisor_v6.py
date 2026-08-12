#!/usr/bin/env python3
"""S5B0 v6 one-shot lifecycle implementation, inert unless future-bound.

It supplies deterministic plans and bounded monitor/postflight primitives.  Its
CLI deliberately has no execute command, so checking this file cannot create a
root, load AppArmor, or execute a candidate.
"""
from __future__ import annotations
import argparse, hashlib, importlib.util, json, os, stat, sys, time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent; SCRIPTS=ROOT/'scripts'
def module(name:str,file:str):
 spec=importlib.util.spec_from_file_location(name,SCRIPTS/file); assert spec and spec.loader
 out=importlib.util.module_from_spec(spec);spec.loader.exec_module(out);return out
gate=module('s5b0_gate_v6_runtime','r8_liquid_s5b0_replay_execution_gate_v6.py')
profile=module('s5b0_profile_v5_runtime','r8_liquid_s5b0_profile_generator_v5.py')
class SupervisorV6Error(ValueError):pass
def exact_fresh(paths:Mapping[str,str],exists:Callable[[str],bool])->None:
 required={'stage_root','partial_root','final_root','start_receipt','final_receipt','failure_receipt','profile_path'}
 if set(paths)!=required or len(set(paths.values()))!=len(paths):raise SupervisorV6Error('fresh target keys/aliases differ')
 for key,value in paths.items():
  if not value.startswith('/') or value!=os.path.normpath(value) or exists(value):raise SupervisorV6Error('unsafe/nonfresh '+key)
def write_receipt_new(path:Path,payload:Mapping[str,Any],mode:int=0o600)->dict[str,Any]:
 """Future phase primitive: anchored O_EXCL+NOFOLLOW, fsync, then identity."""
 if not path.is_absolute() or os.path.lexists(path):raise SupervisorV6Error('receipt not fresh')
 data=json.dumps(payload,allow_nan=False,sort_keys=True,separators=(',',':')).encode()+b'\n'
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,mode)
 try:os.fchmod(fd,mode);os.write(fd,data);os.fsync(fd)
 finally:os.close(fd)
 dfd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:os.fsync(dfd)
 finally:os.close(dfd)
 st=os.lstat(path)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1:raise SupervisorV6Error('unsafe receipt after write')
 return {'path':str(path),'sha256':hashlib.sha256(data).hexdigest(),'mode':f'{stat.S_IMODE(st.st_mode):04o}','size_bytes':st.st_size,'nlink':st.st_nlink}
def disarm_candidate(path:Path,expected_sha:str)->dict[str,Any]:
 """Future post-build primitive: no-follow hash, exact mode 0400."""
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  before=os.fstat(fd)
  if not stat.S_ISREG(before.st_mode) or before.st_nlink!=1:raise SupervisorV6Error('candidate unsafe')
  digest=hashlib.sha256()
  while block:=os.read(fd,1<<20):digest.update(block)
  if digest.hexdigest()!=expected_sha:raise SupervisorV6Error('candidate hash drift')
  os.fchmod(fd,0o400);after=os.fstat(fd)
 finally:os.close(fd)
 if stat.S_IMODE(after.st_mode)!=0o400:raise SupervisorV6Error('candidate disarm failed')
 return {'mode':'0400','sha256':expected_sha,'inode':after.st_ino,'size_bytes':after.st_size,'nlink':after.st_nlink}
def bounded_monitor(samples:Sequence[Mapping[str,Any]],policy:Mapping[str,Any])->dict[str,Any]:
 if not samples:raise SupervisorV6Error('resource samples absent')
 previous=None
 for sample in samples:
  if set(sample)!={'monotonic_s','free_vram_bytes','output_bytes','xid_count'}:raise SupervisorV6Error('sample fields differ')
  now=sample['monotonic_s']
  if not isinstance(now,(int,float)) or previous is not None and not 10<=now-previous<=30:raise SupervisorV6Error('monitor cadence drift')
  previous=now
  if sample['free_vram_bytes']<policy['resources']['minimum_free_vram_bytes'] or sample['output_bytes']>policy['resources']['maximum_output_bytes'] or sample['xid_count']!=0:raise SupervisorV6Error('resource/Xid gate failed')
 return {'sample_count':len(samples),'free_vram_bytes':samples[-1]['free_vram_bytes'],'output_bytes':samples[-1]['output_bytes'],'xid_count':0}
def lifecycle_plan(policy:Mapping[str,Any],final:Mapping[str,Any],last_t_s:float)->dict[str,Any]:
 gate.validate_finalization(policy,final)
 targets={key:final[key] for key in ('stage_root','partial_root','final_root','start_receipt','final_receipt','failure_receipt','profile_path')}
 exact_fresh(targets,lambda _:False)
 rendered=profile.render_profile(profile.TEMPLATE_PATH.read_text(),gate.profile_replacements(final))
 digest=hashlib.sha256(rendered.encode()).hexdigest()
 if digest!=final['profile_sha256']:raise SupervisorV6Error('exact rendered profile hash differs')
 argv=gate.solver_argv(policy,final,last_t_s)
 return {'status':'PLANNED_NOT_EXECUTED','profile_sha256':digest,'solver_argv':argv,'phases':['CREATE_NEW_START_RECEIPT','MATERIALIZE_REAL_STAGING_V4','LOAD_PROFILE_VERIFY','SAMPLE_PREFLIGHT','RUN_CANDIDATE_ONCE_NEW_PGID','MONITOR_10_TO_30_SECONDS','POSTFLIGHT_16_RAW_GAUGE_PARTICLE_MOTION','FINALLY_UNLOAD_ZERO_RESIDUE','CREATE_NEW_FINAL_OR_FAILURE_RECEIPT'],'finally_unload_required':True,'failure_preserves_partial':True,'candidate_runs':1,'runtime_attempted':False}
def postflight(policy:Mapping[str,Any],qc:Mapping[str,Any],samples:Sequence[Mapping[str,Any]])->dict[str,Any]:
 gate.validate_postflight(policy,qc);return {'qc_pass':True,**bounded_monitor(samples,policy),'raw_csv_count':16,'candidate_executed':True}
def self_check()->dict[str,Any]:
 policy,p=gate.load_policy();gate.validate_static(policy)
 return {'status':'PASS_S5B0_V6_LIFECYCLE_STATIC_ONLY','policy_sha256':p,'phases':9,'receipt_o_excl_nofollow':True,'candidate_disarm_primitive':True,'resource_monitor_primitive':True,'runtime_attempted':False,'files_written':False,'profile_loaded':False,'candidate_executed':False,'gpu_exposed':False}
def main(argv:Sequence[str]|None=None)->int:
 ap=argparse.ArgumentParser();ap.add_argument('command',choices=('self-check',));ap.parse_args(argv)
 try:print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
 except Exception as exc:print(json.dumps({'status':'FAIL_S5B0_V6_SUPERVISOR','error':str(exc)},sort_keys=True),file=sys.stderr);return 2
if __name__=='__main__':raise SystemExit(main())
