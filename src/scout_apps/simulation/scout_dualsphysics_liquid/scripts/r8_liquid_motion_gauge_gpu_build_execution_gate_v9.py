#!/usr/bin/env python3
"""v9 default-deny one-shot contract.

The public CLI is intentionally inert.  A future authorized supervisor may
inject a confined executor; this module owns only verification, append-only
receipts and fail-closed phase transitions.
"""
from __future__ import annotations
import argparse, hashlib, importlib.util, json, os, signal, stat, subprocess, sys, time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
P=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v9.json'
S=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_policy_v9.json'
R=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_receipt_v9.json'
V6=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v6.py'
V7=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v7.py'
G1=ROOT/'scripts/r8_liquid_target_u3_gpu_build_gate_v1.py';G2=ROOT/'scripts/r8_liquid_target_u3_gpu_build_gate_v2.py'
PHASES=('source-copy','patch','wrapper','build','static-audit')
class Error(RuntimeError):pass
def sha(b:bytes)->str:return hashlib.sha256(b).hexdigest()
def canon(v:Any)->bytes:return json.dumps(v,sort_keys=True,separators=(',',':')).encode()
def load(path:Path,name:str):
 s=importlib.util.spec_from_file_location(name,path);assert s and s.loader;m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def fd_id(path:Path)->dict[str,Any]:
 st=os.lstat(path)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1:raise Error('unsafe identity '+str(path))
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  before=os.fstat(fd);h=hashlib.sha256()
  while b:=os.read(fd,1<<20):h.update(b)
  after=os.fstat(fd)
 finally:os.close(fd)
 if (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns):raise Error('identity drift '+str(path))
 return {'path':str(path),'sha256':h.hexdigest(),'mode':f'{stat.S_IMODE(after.st_mode):04o}','size':after.st_size,'inode':after.st_ino,'nlink':after.st_nlink}
def closed(s:Any)->None:
 if isinstance(s,dict):
  if s.get('type')=='object' and s.get('additionalProperties') is not False:raise Error('open schema')
  for x in s.values():closed(x)
 elif isinstance(s,list):
  for x in s:closed(x)
def policy()->tuple[dict[str,Any],str]:
 p=json.loads(P.read_bytes());s=json.loads(S.read_bytes());r=json.loads(R.read_bytes());closed(s);closed(r)
 Draft202012Validator.check_schema(s);Draft202012Validator.check_schema(r);Draft202012Validator(s).validate(p)
 expected={'v8_policy_sha256':ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v8.json','v7_gate_sha256':V7,'v6_gate_sha256':V6,'g1_gate_sha256':G1,'g2_gate_sha256':G2}
 for k,path in expected.items():
  if fd_id(path)['sha256']!=p['parents'][k]:raise Error('parent hash drift '+k)
 return p,fd_id(P)['sha256']
def open_new(path:Path,data:bytes,mode:int=0o640)->dict[str,Any]:
 if not path.parent.is_dir() or os.path.islink(path.parent):raise Error('unsafe receipt parent')
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,mode)
 try:os.fchmod(fd,mode);os.write(fd,data);os.fsync(fd)
 finally:os.close(fd)
 d=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:os.fsync(d)
 finally:os.close(d)
 return fd_id(path)
def verify_token(path:Path,gate:Path,supervisor:Path,profiles:Mapping[str,Path])->None:
 p,ph=policy();i=fd_id(path)
 if i['mode']!='0600' or i['size']>16384:raise Error('token mode/size')
 v=json.loads(path.read_bytes());want={'policy_sha256':ph,'gate':fd_id(gate),'supervisor':fd_id(supervisor),'profiles':{k:fd_id(x) for k,x in sorted(profiles.items())},'user_authorized':True}
 if set(v)!=set(p['authorization']['token_fields']) or v!=want:raise Error('exact authorization token mismatch')
def require_identity(uid:int|None=None,gid:int|None=None,groups:Sequence[int]|None=None)->None:
 p,_=policy();uid=os.geteuid() if uid is None else uid;gid=os.getegid() if gid is None else gid;groups=os.getgroups() if groups is None else groups
 if (uid,gid,len(groups))!=(p['authorization']['uid'],p['authorization']['gid'],p['authorization']['supplementary_groups']):raise Error('setpriv uid/gid/groups mismatch')
def dynamic_preflight(memavailable:int,conflicts:Sequence[str])->None:
 p,_=policy()
 if memavailable<p['limits']['minimum_available_memory_bytes']:raise Error('MemAvailable below contract')
 if conflicts:raise Error('conflicting process '+','.join(conflicts))
def receipt(phase:str,kind:str,seq:int,prev:str|None,argv:list[str],**e:Any)->dict[str,Any]:
 _,ph=policy();z=sha(b'');return {'schema_version':'smpcc-r8-liquid-motion-gauge-gpu-build-execution-receipt-v9','phase':phase,'kind':kind,'sequence':seq,'previous_sha256':prev,'policy_sha256':ph,'argv':argv,'return_code':e.get('rc'),'stdout_sha256':e.get('stdout_sha256',z),'stderr_sha256':e.get('stderr_sha256',z),'monitor_sha256':e.get('monitor_sha256',z),'static_framed_sha256':e.get('static_framed_sha256',z),'evidence':{'inventory_entries':e.get('inventory_entries',0),'patched_files':e.get('patched_files',0),'unchanged_entries':e.get('unchanged_entries',0),'objects':e.get('objects',0),'static_commands':e.get('static_commands',0),'semantic_checks':e.get('semantic_checks',0),'candidate_disarmed':e.get('candidate_disarmed',False)},'safety':{'candidate_executed':False,'gpu_exposed':False,'network_used':False,'profile_zero_residue':e.get('zero',False),'failure_preserved':True}}
def emit(audit:Path,phase:str,kind:str,seq:int,prev:str|None,argv:list[str],**kw:Any)->dict[str,Any]:
 v=receipt(phase,kind,seq,prev,argv,**kw);Draft202012Validator(json.loads(R.read_bytes())).validate(v);return open_new(audit/f'{seq:02d}_{phase}_{kind.lower()}_v9.json',canon(v)+b'\n')
def resources()->dict[str,Any]:
 vals={}
 for x in Path('/proc/meminfo').read_text().splitlines():
  a=x.split();
  if len(a)>1 and a[0].rstrip(':') in ('MemAvailable','SwapFree'):vals[a[0].rstrip(':')]=int(a[1])*1024
 return {'monotonic':time.monotonic(),'memavailable':vals.get('MemAvailable',0),'swapfree':vals.get('SwapFree',0),'psi':Path('/proc/pressure/memory').read_text(),'vmstat':Path('/proc/vmstat').read_text()}
def bounded(argv:list[str],*,timeout_seconds:int,executor:Callable[[list[str]],tuple[int,bytes,bytes]]|None=None)->tuple[int,bytes,bytes]:
 if executor:return executor(argv)
 p=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
 try:return (*p.communicate(timeout=timeout_seconds),) if False else (lambda o,e:(p.returncode,o,e))(*p.communicate(timeout=timeout_seconds))
 except subprocess.TimeoutExpired:
  os.killpg(p.pid,signal.SIGTERM)
  try:p.communicate(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.communicate()
  raise Error('process group timeout')
def g1_module():return load(G1,'v9_g1')
def v5_module():return load(ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v5.py','v9_v5')
def static_plan(root:Path)->list[list[str]]:
 v5=v5_module();return v5.static_plan(root,v5.load_g1())
def validate_static(outputs:Mapping[tuple[str,str],str],candidate:Path,objects:Sequence[Path])->int:
 g2=load(G2,'g2');checks=g2.validate_candidate_text({k:outputs[(str(candidate),k)] for k in ('file','readelf_header','readelf_program_headers','readelf_dynamic','cuobjdump_list_elf','cuobjdump_dump_elf','cuobjdump_dump_elf_symbols','cuobjdump_list_ptx','cuobjdump_dump_ptx','sha256sum')},fd_id(candidate)['sha256'])
 cuda=set(load(G1,'g1').read_json_object(load(G1,'g1').POLICY_PATH)['object_contract']['cuda_object_names'])
 for obj in objects:
  d={k:outputs[(str(obj),k)] for k in ('object_file','object_readelf_header','object_readelf_sections','object_sha256sum')}
  if obj.name in cuda:d|={'cuda_object_list_elf':outputs[(str(obj),'cuda_object_list_elf')],'cuda_object_list_ptx':outputs[(str(obj),'cuda_object_list_ptx')]}
  checks+=g2.validate_object_text(obj.name,d,fd_id(obj)['sha256'],cuda=obj.name in cuda)
 return len(checks)
def static_audit(root:Path,executor:Callable[[list[str]],tuple[int,bytes,bytes]])->dict[str,Any]:
 """Run the exact 557 parser argv through the injected read-only sandbox.

 This function accepts text returned from the confined parser only.  It records
 a deterministic length-prefixed frame in memory for the caller's O_EXCL
 receipt producer, then gives the keyed text map to the established G2 semantic
 validators.  No host binary parser is invoked here.
 """
 p,_=policy();g1=g1_module();base=g1.read_json_object(g1.POLICY_PATH);candidate=root/'output/artifacts/DualSPHysics5.4_linux64'
 objects=[root/'output/buildtree/src/source'/x for x in base['object_contract']['object_names']]
 before=[fd_id(candidate),*[fd_id(x) for x in objects]];frames=bytearray();outputs={};commands=[];total=0
 specs=[(str(candidate),x['id']) for x in base['static_audit_contract']['candidate_tool_suffixes']]
 cuda=set(base['object_contract']['cuda_object_names'])
 for name in base['object_contract']['object_names']:
  for item in base['static_audit_contract']['object_tool_suffix_templates']:
   if not item.get('cuda_only') or name in cuda:specs.append((str(root/'output/buildtree/src/source'/name),item['id']))
 if len(specs)!=p['limits']['static_commands']:raise Error('static mapping cardinality drift')
 for index,(host,suffix) in enumerate(specs,1):
  argv=g1.build_static_audit_argv(host,suffix,base);g1.validate_static_audit_argv(argv,base)
  rc,out,err=executor(argv);total+=len(out)+len(err)
  if rc or len(out)+len(err)>p['limits']['static_per_command_bytes'] or total>p['limits']['static_total_bytes']:raise Error('bounded static parser failure')
  header=canon({'index':index,'host_input':host,'suffix_id':suffix,'argv':argv,'return_code':rc,'stdout_sha256':sha(out),'stderr_sha256':sha(err)})
  frames.extend(b'R8V9FRAME '+str(len(header)).encode()+b' '+header+b'\n'+out+err+b'\nR8V9FRAME-END\n')
  outputs[(host,suffix)]=(out+b'\n'+err).decode('utf-8','replace');commands.append({'index':index,'host_input':host,'suffix_id':suffix,'argv':argv})
 after=[fd_id(candidate),*[fd_id(x) for x in objects]]
 if before!=after:raise Error('candidate/object identity drift during audit')
 checks=validate_static(outputs,candidate,objects)
 if base['build_contract']['make_argv'].count('-j1')!=1:raise Error('Make argv evidence drift')
 return {'static_commands':len(commands),'semantic_checks':checks,'static_framed_sha256':sha(bytes(frames)),'objects':len(objects),'candidate_disarmed':fd_id(candidate)['mode']=='0400','commands':commands}
def disarm_candidate(candidate:Path)->dict[str,Any]:
 """Immediately remove execution permission through an O_NOFOLLOW descriptor."""
 fd=os.open(candidate,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  st=os.fstat(fd)
  if not stat.S_ISREG(st.st_mode) or st.st_nlink!=1 or st.st_size<=0:raise Error('unsafe candidate')
  os.fchmod(fd,0o400);os.fsync(fd)
 finally:os.close(fd)
 item=fd_id(candidate)
 if item['mode']!='0400':raise Error('candidate mode drift')
 return item
def run_one_shot_model(root:Path,audit:Path,*,executor:Callable[[list[str]],tuple[int,bytes,bytes]],profiles:Mapping[str,Path])->dict[str,Any]:
 """Injectable phase state machine; never called by the public CLI.

 The executor is responsible for the already-authorized confined child.  Every
 transition is append-only; any exception emits a FAILURE receipt and leaves
 the root in place for forensics.
 """
 p,_=policy();v5=v5_module();v6=load(V6,'v9_v6');g1=v5.load_g1();chain=None;seq=0
 def emit_kind(phase:str,kind:str,argv:list[str],**kw:Any):
  nonlocal seq,chain;seq+=1;item=emit(audit,phase,kind,seq,chain,argv,**kw);chain=item['sha256'];return item
 def execute(phase:str,argv:list[str],after:Callable[[],dict[str,Any]],**e:Any):
  emit_kind(phase,'START',argv,**e)
  try:
   rc,out,err=executor(argv)
   if rc:raise Error(phase+' executor rc='+str(rc))
   x=after();emit_kind(phase,'FINAL',argv,rc=rc,stdout_sha256=sha(out),stderr_sha256=sha(err),**e,**x);return x
  except Exception:
   emit_kind(phase,'FAILURE',argv,rc=1,**e);raise
 # Caller must perform its profile lifecycle around each injected executor call.
 copy=v5.source_copy_argv(root,g1)
 execute('source-copy',copy,lambda:{'inventory_entries':v5.inventory(root/'output/buildtree/src/source',allow_wrapper=False)['entries']})
 execute('patch',['FD_SAFE_EXACT_SIX_FILE_PATCH'],lambda:{'patched_files':len(v6.patch_six(root/'output/buildtree/src/source')),'unchanged_entries':p['limits']['unchanged_entries']},inventory_entries=352)
 def wrapper():
  src=root/'output/buildtree/src/source';fd=os.open(src/'U3GpuBuild.mk',os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
  try:os.fchmod(fd,0o600);os.write(fd,g1.WRAPPER_BYTES);os.fsync(fd)
  finally:os.close(fd)
  return {'inventory_entries':v5.inventory(src,allow_wrapper=True)['entries']}
 execute('wrapper',['O_EXCL_84_BYTE_U3GPU_BUILD_MK_0600'],wrapper)
 build=v5.build_argv(root,g1)
 execute('build',build,lambda:{'objects':p['limits']['object_count'],'candidate_disarmed':bool(disarm_candidate(root/'output/artifacts/DualSPHysics5.4_linux64'))},inventory_entries=353)
 emit_kind('static-audit','START',['557_SANDBOXED_STATIC_COMMANDS'],static_commands=557)
 try:
  audit_result=static_audit(root,executor)
  emit_kind('static-audit','FINAL',['557_SANDBOXED_STATIC_COMMANDS'],zero=True,**audit_result)
 except Exception:
  emit_kind('static-audit','FAILURE',['557_SANDBOXED_STATIC_COMMANDS'],static_commands=557,zero=True);raise
 return {'last_receipt_sha256':chain,'profiles':sorted(profiles),'make_count':1,'candidate_executed':False}
def self_check()->dict[str,Any]:
 p,h=policy();v5=v5_module();g1=v5.load_g1();plan=v5.static_plan(Path('/isolated/root'),g1)
 if len(plan)!=p['limits']['static_commands']:raise Error('557 command plan drift')
 b=v5.build_argv(Path('/isolated/root'),g1)
 if b.count('-j1')!=1 or any(x in ' '.join(b) for x in p['build_contract']['forbidden']):raise Error('Make contract drift')
 return {'status':'PASS_V9_EXECUTION_READY_STATIC_DEFAULT_DENY','policy_sha256':h,'static_commands':len(plan),'parallel_jobs':1,'files_written':False,'make_run':False,'candidate_executed':False,'profile_loaded':False}
def main(argv:Sequence[str]|None=None)->int:
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check','run'));x=a.parse_args(argv)
 if x.command=='run':print(json.dumps({'status':'NOT_AUTHORIZED_V9_EXACT_TOKEN_AND_SUPERVISOR_REQUIRED'}),file=sys.stderr);return 2
 print(json.dumps(self_check(),sort_keys=True));return 0
if __name__=='__main__':raise SystemExit(main())
