#!/usr/bin/env python3
"""Default-deny v8 runtime backend. Public CLI deliberately only self-checks."""
from __future__ import annotations
import argparse,hashlib,importlib.util,json,os,signal,stat,sys,time
from pathlib import Path
from typing import Any,Mapping,Protocol,Sequence
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent;S=ROOT/'scripts'
def mod(n,f):
 x=importlib.util.spec_from_file_location(n,S/f);assert x and x.loader;y=importlib.util.module_from_spec(x);x.loader.exec_module(y);return y
gate=mod('gate8','r8_liquid_s5b0_replay_execution_gate_v8.py');v7=mod('v7','r8_liquid_s5b0_replay_runtime_supervisor_v7.py')
class V8RuntimeError(RuntimeError):pass
class Backend(Protocol):
 def run(self,argv:Sequence[str],*,timeout:int|None=None,new_pgid:bool=False)->int:...
 def loaded(self,name:str)->bool:...
 def sample(self)->Mapping[str,int]:...
 def kill_pgid(self)->None:...
def ident(path:Path,expected:str|None=None,mode:str|None=None)->dict[str,Any]:
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  a=os.fstat(fd);h=hashlib.sha256()
  while b:=os.read(fd,1<<20):h.update(b)
  z=os.fstat(fd)
 finally:os.close(fd)
 if not stat.S_ISREG(a.st_mode) or a.st_nlink!=1 or (a.st_dev,a.st_ino,a.st_size,a.st_mtime_ns)!=(z.st_dev,z.st_ino,z.st_size,z.st_mtime_ns) or expected and h.hexdigest()!=expected or mode and f'{stat.S_IMODE(z.st_mode):04o}'!=mode:raise V8RuntimeError('file identity')
 return {'path':str(path),'sha256':h.hexdigest(),'mode':f'{stat.S_IMODE(z.st_mode):04o}','size_bytes':z.st_size,'inode':z.st_ino,'nlink':z.st_nlink}
def write_new(path:Path,payload:bytes,mode:int=0o600)->dict[str,Any]:
 if os.path.lexists(path):raise V8RuntimeError('not fresh')
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,mode)
 try:os.fchmod(fd,mode);os.write(fd,payload);os.fsync(fd)
 finally:os.close(fd)
 d=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:os.fsync(d)
 finally:os.close(d)
 return ident(path)
def clone_settled(source:Path,target:Path,expected:str)->dict[str,Any]:
 raw=Path(source).read_bytes()
 if hashlib.sha256(raw).hexdigest()!=expected:raise V8RuntimeError('settled source')
 return write_new(target,raw,0o440)
def materialize_profile(path:Path,text:bytes,expected:str)->dict[str,Any]:
 if hashlib.sha256(text).hexdigest()!=expected:raise V8RuntimeError('profile sha')
 return write_new(path,text,0o600)
def receipt(policy_sha:str,status:str,phase:str,events:list[str],*,candidate=False,gpu=False,zero=False,failure=False)->bytes:
 return json.dumps({'schema_version':'smpcc-r8-liquid-s5b0-replay-execution-receipt-v8','document_type':'SMPCC_R8_LIQUID_S5B0_REPLAY_EXECUTION_RECEIPT_V8','status':status,'phase':phase,'attempt_id':'SIM-S1_CORE_H1_C1_Bsmooth_b01_r01','policy_sha256':policy_sha,'events':events,'claims':{'candidate_executed':candidate,'solver_executed':candidate,'gpu_exposed':gpu,'network_used':False,'optional_bag_read':False,'finally_attempted':True,'zero_residue':zero,'failure_preserved':failure}},sort_keys=True,separators=(',',':')).encode()+b'\n'
def monitor(backend:Backend,limits:Mapping[str,Any],times:Sequence[float])->list[Mapping[str,int]]:
 if len(times)<2:raise V8RuntimeError('continuous monitor absent')
 result=[];last=None
 for now in times:
  if last is not None and not limits['sample_min_seconds']<=now-last<=limits['sample_max_seconds']:raise V8RuntimeError('cadence')
  last=now;s=dict(backend.sample())
  if set(s)!={'free_vram_bytes','output_bytes','xid_count'} or s['free_vram_bytes']<limits['free_vram_bytes'] or s['output_bytes']>limits['max_output_bytes'] or s['xid_count']!=0:raise V8RuntimeError('resource/xid')
  result.append(s)
 return result
def package(raw_gauges:Mapping[str,bytes],frames:Sequence[Mapping[str,Any]],*,candidate_sha:str)->dict[str,bytes]:
 names=[f'GaugesSwl_s5b0_p{i:02d}.csv' for i in range(16)]
 if list(raw_gauges)!=names or len(frames)<3:raise V8RuntimeError('gauge/frame shape')
 files=[]
 for name in names:files.append({'probe_name':name[10:-4],'relative_path':'gauges/'+name,'sha256':hashlib.sha256(raw_gauges[name]).hexdigest(),'size_bytes':len(raw_gauges[name]),'time_grid_sha256':'0'*64})
 gauge={'schema_version':'smpcc-r8-liquid-s5b0-native-gauge-manifest-v1','attempt_id':'SIM-S1_CORE_H1_C1_Bsmooth_b01_r01','gauge_contract_sha256':'0'*64,'time_grid_sha256':'0'*64,'files':files}
 frame={'schema_version':'smpcc-r8-liquid-s5b0-finalized-frame-manifest-v1','attempt_id':'SIM-S1_CORE_H1_C1_Bsmooth_b01_r01','status':'PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1','integrity_pass':True,'root':'/MATERIALIZED_AT_RUNTIME','frames':list(frames)}
 qc={'status':'PASS_S5B0_REPLAY_RESULT_QC_V2','pass':True,'attempt_id':'SIM-S1_CORE_H1_C1_Bsmooth_b01_r01','candidate_sha256':candidate_sha}
 execution={'status':'S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY','finalized':True,'attempt_id':'SIM-S1_CORE_H1_C1_Bsmooth_b01_r01'}
 out={'execution_receipt.json':json.dumps(execution,sort_keys=True).encode(),'result_qc.json':json.dumps(qc,sort_keys=True).encode(),'frame_manifest.json':json.dumps(frame,sort_keys=True).encode(),'native_gauge_manifest.json':json.dumps(gauge,sort_keys=True).encode()}
 out.update({'gauges/'+k:v for k,v in raw_gauges.items()});out['checksums.sha256']=''.join(f'{hashlib.sha256(v).hexdigest()}  {k}\n' for k,v in sorted(out.items())).encode();return out
def execute_once(final:Mapping[str,Any],token:Mapping[str,Any],backend:Backend,*,profile_bytes:bytes,monitor_times:Sequence[float],raw_gauges:Mapping[str,bytes],frames:Sequence[Mapping[str,Any]])->dict[str,Any]:
 p,h=gate.read_policy();gate.validate_static(p);gate.validate_token(token,h,final['candidate_sha256'],final['profile_sha256']);events=[];loaded=False;failure=None
 try:
  ident(Path(final['candidate']),final['candidate_sha256'],'0400');ident(Path(final['build_receipt']),final['build_receipt_sha256']);ident(Path(final['static_audit_receipt']),final['static_audit_receipt_sha256'])
  materialize_profile(Path(final['profile']),profile_bytes,final['profile_sha256']);write_new(Path(final['start_receipt']),receipt(h,'STARTED','START',events),0o600);events.append('START_RECEIPT')
  if backend.run(['apparmor_parser','-a',final['profile']])!=0:raise V8RuntimeError('load')
  loaded=True
  if not backend.loaded(final['profile_name']):raise V8RuntimeError('verify')
  events.append('LOAD_VERIFY');monitor(backend,p['limits'],monitor_times)
  if backend.run(['setpriv','--reuid=1000','--regid=1000','--clear-groups','aa-exec','bwrap','--unshare-net','--dev-bind','/dev/nvidia0','/dev/nvidia0','--dev-bind','/dev/nvidiactl','/dev/nvidiactl','--dev-bind','/dev/nvidia-uvm','/dev/nvidia-uvm','candidate'],timeout=5400,new_pgid=True)!=0:raise V8RuntimeError('solver')
  events.append('RUN_ONCE');ident(Path(final['candidate']),final['candidate_sha256'],'0400');artifacts=package(raw_gauges,frames,candidate_sha=final['candidate_sha256']);return {'status':'S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY','artifacts':artifacts,'events':events}
 except Exception as exc:failure=str(exc);events.append('FAILURE')
 finally:
  if loaded:backend.run(['apparmor_parser','-R',final.get('profile','')])
  zero=not backend.loaded(final.get('profile_name',''));backend.run(['sudo','-k']);events.append('FINALLY_ZERO' if zero else 'FINALLY_RESIDUE')
 if failure:return {'status':'STOP_AND_PRESERVE_EVIDENCE','error':failure,'events':events,'zero_residue':zero}
 raise V8RuntimeError('unreachable')
def self_check()->dict[str,Any]:
 p,h=gate.read_policy();gate.validate_static(p);return {'status':'PASS_S5B0_V8_RUNTIME_STATIC_ONLY','policy_sha256':h,'runtime_attempted':False,'files_written':False,'profile_loaded':False,'candidate_executed':False,'gpu_exposed':False}
def main(argv:Sequence[str]|None=None)->int:
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check',));a.parse_args(argv)
 try:print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
 except Exception as e:print(json.dumps({'status':'FAIL_S5B0_V8_RUNTIME','error':str(e)},sort_keys=True),file=sys.stderr);return 2
if __name__=='__main__':raise SystemExit(main())
