#!/usr/bin/env python3
"""Bound, default-deny v5 executor primitives for one fresh motion-Gauge build.

No CLI phase runs unless the outer supervisor has supplied a verified exact
authorization token.  The phase functions deliberately take an executor so
unit tests can exercise transitions without starting bwrap, Make, or a parser.
"""
from __future__ import annotations
import argparse,hashlib,importlib.util,json,os,stat,sys,time
from pathlib import Path
from typing import Any,Callable,Mapping,Sequence
from jsonschema import Draft202012Validator
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
POLICY=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v5.json'
SCHEMA=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_policy_v5.json'
RECEIPT_SCHEMA=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_receipt_v5.json'
G1=ROOT/'scripts/r8_liquid_target_u3_gpu_build_gate_v1.py'; PATCH=ROOT/'scripts/r8_liquid_motion_attached_gauge_patch_gate_v2.py'
PHASES=('source-copy','patch','wrapper','build','static-audit')
class GateError(RuntimeError):pass
def sha(b:bytes)->str:return hashlib.sha256(b).hexdigest()
def canon(v:Any)->bytes:return json.dumps(v,sort_keys=True,separators=(',',':')).encode()
def file_id(path:Path)->dict[str,Any]:
 st=os.lstat(path)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1:raise GateError('unsafe regular file')
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  before=os.fstat(fd);h=hashlib.sha256()
  while b:=os.read(fd,1<<20):h.update(b)
  after=os.fstat(fd)
 finally:os.close(fd)
 if (before.st_dev,before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_dev,after.st_ino,after.st_size,after.st_mtime_ns):raise GateError('identity changed while hashed')
 return {'path':str(path),'sha256':h.hexdigest(),'mode':f'{stat.S_IMODE(after.st_mode):04o}','size':after.st_size,'inode':after.st_ino}
def closed(v:Any,at:str='$')->None:
 if isinstance(v,dict):
  if v.get('type')=='object' and v.get('additionalProperties') is not False:raise GateError('open schema '+at)
  for k,x in v.items():closed(x,at+'/'+k)
 elif isinstance(v,list):
  for i,x in enumerate(v):closed(x,f'{at}/{i}')
def load_g1():
 if sha(G1.read_bytes())!='e34ab0facc3c665c6befbb792c7344e0af6e652fd3c6b30dfa81ca8cbea5b502':raise GateError('G1 byte drift')
 sp=importlib.util.spec_from_file_location('mg_g1',G1);assert sp and sp.loader;m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m);return m
def policy()->tuple[dict[str,Any],str]:
 p=json.loads(POLICY.read_bytes());s=json.loads(SCHEMA.read_bytes());r=json.loads(RECEIPT_SCHEMA.read_bytes())
 for x in(s,r):Draft202012Validator.check_schema(x);closed(x)
 Draft202012Validator(s).validate(p)
 if sha((ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v4.json').read_bytes())!=p['parent_v4_policy_sha256']:raise GateError('v4 parent drift')
 if sha(PATCH.read_bytes())!=p['patch_gate_sha256']:raise GateError('patch parent drift')
 return p,sha(POLICY.read_bytes())
def open_new(path:Path,data:bytes,mode:int=0o640)->dict[str,Any]:
 # Runtime caller must have already created a fixed non-symlink parent.
 if not path.parent.is_dir() or os.path.islink(path.parent):raise GateError('unsafe receipt parent')
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,mode)
 try:os.fchmod(fd,mode);os.write(fd,data);os.fsync(fd)
 finally:os.close(fd)
 dfd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:os.fsync(dfd)
 finally:os.close(dfd)
 return file_id(path)
def source_copy_argv(root:Path,g1:Any)->list[str]:
 p=json.loads(g1.POLICY_PATH.read_bytes());a=list(p['source_copy_contract']['full_execution_argv']);old='/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T102641Z_a.partial/output';a=[str(root/'output') if x==old else x for x in a];a[a.index('PATH=/usr/bin:/usr/local/cuda-12.8/bin')]='PATH=/usr/bin';return a
def build_argv(root:Path,g1:Any)->list[str]:
 p=json.loads(g1.POLICY_PATH.read_bytes());old='/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T102641Z_a.partial/output';a=[str(root/'output') if x==old else x for x in p['build_contract']['full_execution_argv']];
 if a.count('-j1')!=1 or any(x in '\n'.join(a) for x in ('-j2','-j4','g++-13')):raise GateError('one-Make argv drift')
 return a
def static_plan(root:Path,g1:Any)->list[list[str]]:
 policy=json.loads(g1.POLICY_PATH.read_bytes());old=str(g1.ROOT_A);candidate=str(g1.OUTPUT_A/'artifacts/DualSPHysics5.4_linux64');out=[]
 def remap(argv:list[str])->list[str]: return [x.replace(old,str(root)) for x in argv]
 for suffix in policy['static_audit_contract']['candidate_tool_suffixes']:out.append(remap(g1.build_static_audit_argv(candidate,suffix['id'],policy)))
 cuda=set(policy['object_contract']['cuda_object_names'])
 for name in policy['object_contract']['object_names']:
  host=str(g1.ROOT_A/'output/buildtree/src/source'/name)
  for suffix in policy['static_audit_contract']['object_tool_suffix_templates']:
   if not suffix.get('cuda_only') or name in cuda:out.append(remap(g1.build_static_audit_argv(host,suffix['id'],policy)))
 if len(out)!=557:raise GateError('static command plan drift')
 return out
def inventory(root:Path,*,allow_wrapper:bool)->dict[str,int]:
 g1=load_g1(); expected=g1.read_json_object(g1.POLICY_PATH)['source_input']; receipt=g1.read_json_object(Path(expected['source_receipt'])); rows=receipt['results']['materialization']['sealed_output']['entries'];want={x['path'].removeprefix('src/source/'):x for x in rows};found={};wrapper=0
 for base,dirs,files in os.walk(root,followlinks=False):
  for n in dirs:
   if os.path.islink(Path(base)/n):raise GateError('symlink directory')
  for n in files:
   path=Path(base)/n;rel=str(path.relative_to(root));st=os.lstat(path)
   if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1 or stat.S_IMODE(st.st_mode)&0o111:raise GateError('unsafe build input')
   fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
   try:magic=os.read(fd,4)
   finally:os.close(fd)
   if magic==b'\x7fELF':raise GateError('ELF before build')
   ident=file_id(path)
   if rel=='U3GpuBuild.mk' and allow_wrapper:
    if ident['size']!=84 or ident['sha256']!=g1.WRAPPER_SHA256 or ident['mode']!='0600':raise GateError('wrapper drift')
    wrapper+=1
   elif rel in want and ident['size']==want[rel]['size_bytes'] and ident['sha256']==want[rel]['sha256']:found[rel]=ident
   else:raise GateError('extra/drift input '+rel)
 if set(found)!=set(want):raise GateError('sealed manifest mismatch')
 return {'entries':len(found)+wrapper,'sealed':len(found),'wrapper':wrapper,'objects':0}
def disarm(path:Path)->dict[str,Any]|None:
 if not path.exists():return None
 fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  st=os.fstat(fd)
  if not stat.S_ISREG(st.st_mode) or st.st_nlink!=1 or st.st_size<=0:raise GateError('bad candidate')
  os.fchmod(fd,0o400)
 finally:os.close(fd)
 item=file_id(path)
 if item['mode']!='0400':raise GateError('candidate disarm drift')
 return item
def receipt(p:Mapping[str,Any],ph:str,kind:str,sequence:int,argv:list[str],chain:list[str],inv:dict[str,int],candidate:dict[str,Any]|None,zero:bool)->dict[str,Any]:return {'schema_version':'smpcc-r8-liquid-motion-gauge-gpu-build-execution-receipt-v5','phase':ph,'kind':kind,'policy_sha256':sha(POLICY.read_bytes()),'sequence':sequence,'argv':argv,'return_code':None,'receipt_chain':chain,'inventory':inv,'candidate':candidate,'safety':{'o_excl':True,'o_nofollow':True,'fsync':True,'candidate_executed':False,'gpu_exposed':False,'network_used':False,'profile_zero_residue':zero}}
def bound_phase(phase:str,root:Path,execute:Callable[[list[str]],int],audit_dir:Path)->dict[str,Any]:
 p,_=policy();g1=load_g1(); audit_dir=Path(audit_dir)
 if phase=='source-copy':argv=source_copy_argv(root,g1);inv={'entries':0,'sealed':0,'wrapper':0,'objects':0}
 elif phase=='patch':argv=['PATCH_SIX_BYTE_PINNED_FILES'];inv=inventory(root/'output/buildtree/src/source',allow_wrapper=False)
 elif phase=='wrapper':argv=['CREATE_U3GPU_BUILD_MK_O_EXCL_0600'];inv=inventory(root/'output/buildtree/src/source',allow_wrapper=False)
 elif phase=='build':argv=build_argv(root,g1);inv=inventory(root/'output/buildtree/src/source',allow_wrapper=True)
 else:argv=['STATIC_AUDIT_557_EXACT_COMMANDS'];inv={'entries':0,'sealed':0,'wrapper':0,'objects':131}
 start=receipt(p,phase,'start',1,argv,[],inv,None,False);Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(start);start_id=open_new(audit_dir/f'{p["campaign"]["build"]}_{phase}_start_v5.json',canon(start)+b'\n')
 rc=execute(argv)
 if rc:raise GateError(f'{phase} executor rc={rc}')
 if phase=='source-copy':inv=inventory(root/'output/buildtree/src/source',allow_wrapper=False)
 if phase=='wrapper':
  src=root/'output/buildtree/src/source';fd=os.open(src/'U3GpuBuild.mk',os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o600)
  try:os.fchmod(fd,0o600);os.write(fd,g1.WRAPPER_BYTES);os.fsync(fd)
  finally:os.close(fd)
  inv=inventory(src,allow_wrapper=True)
 candidate=disarm(root/'output/artifacts/DualSPHysics5.4_linux64') if phase=='build' else None
 final=receipt(p,phase,'final',2,argv,[start_id['sha256']],inv,candidate,False);Draft202012Validator(json.loads(RECEIPT_SCHEMA.read_bytes())).validate(final);fid=open_new(audit_dir/f'{p["campaign"]["build"]}_{phase}_final_v5.json',canon(final)+b'\n')
 return {'start':start_id,'final':fid,'argv':argv,'inventory':inv,'candidate':candidate}
def self_check()->dict[str,Any]:
 p,h=policy();g=load_g1();return {'status':'PASS_V5_BOUND_PHASES_STATIC_DEFAULT_DENY','policy_sha256':h,'copy_argv_count':len(source_copy_argv(Path(p['campaign']['root']),g)),'build_argv_count':len(build_argv(Path(p['campaign']['root']),g)),'static_commands':len(static_plan(Path(p['campaign']['root']),g)),'files_written':False,'make_run':False,'candidate_executed':False}
def main(argv:Sequence[str]|None=None)->int:
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check',*PHASES));x=a.parse_args(argv)
 if x.command!='self-check':print(json.dumps({'status':'FAIL_V5','error':'NOT_AUTHORIZED: outer supervisor only'},sort_keys=True),file=sys.stderr);return 2
 try:r=self_check()
 except Exception as e:print(json.dumps({'status':'FAIL_V5','error':str(e)},sort_keys=True),file=sys.stderr);return 2
 print(json.dumps(r,sort_keys=True,separators=(',',':')));return 0
if __name__=='__main__':raise SystemExit(main())
