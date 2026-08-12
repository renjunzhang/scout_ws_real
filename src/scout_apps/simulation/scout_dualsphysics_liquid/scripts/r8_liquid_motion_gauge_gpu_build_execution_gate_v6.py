#!/usr/bin/env python3
"""v6 real patch/static-audit algorithms, callable only via injected executor."""
from __future__ import annotations
import hashlib,importlib.util,json,os,signal,stat,subprocess,time
from pathlib import Path
from typing import Any,Callable,Sequence
from jsonschema import Draft202012Validator
ROOT=Path(__file__).resolve().parent.parent
POLICY=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v6.json';SCHEMA=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_policy_v6.json';RECEIPT=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_receipt_v6.json'
G1=ROOT/'scripts/r8_liquid_target_u3_gpu_build_gate_v1.py';PV2=ROOT/'scripts/r8_liquid_motion_attached_gauge_patch_gate_v2.py';PV1=ROOT/'scripts/r8_liquid_motion_attached_gauge_patch_gate_v1.py'
class Error(RuntimeError):pass
def sha(b:bytes)->str:return hashlib.sha256(b).hexdigest()
def module(path:Path,name:str):
 s=importlib.util.spec_from_file_location(name,path);assert s and s.loader;m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def file_id(p:Path)->dict[str,Any]:
 st=os.lstat(p)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1:raise Error('unsafe regular file')
 fd=os.open(p,os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  h=hashlib.sha256();before=os.fstat(fd)
  while b:=os.read(fd,1<<20):h.update(b)
  after=os.fstat(fd)
 finally:os.close(fd)
 if (before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_ino,after.st_size,after.st_mtime_ns):raise Error('file drift while hash')
 return {'path':str(p),'sha256':h.hexdigest(),'mode':f'{stat.S_IMODE(after.st_mode):04o}','size':after.st_size,'inode':after.st_ino}
def policy():
 p=json.loads(POLICY.read_text());s=json.loads(SCHEMA.read_text());r=json.loads(RECEIPT.read_text());Draft202012Validator.check_schema(s);Draft202012Validator.check_schema(r);Draft202012Validator(s).validate(p)
 if sha((ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v5.json').read_bytes())!=p['parent_v5_policy_sha256']:raise Error('v5 drift')
 if sha(G1.read_bytes())!=p['g1_gate_sha256'] or sha(PV2.read_bytes())!=p['patch_v2_sha256']:raise Error('byte pinned parent drift')
 return p,sha(POLICY.read_bytes())
def fd_replace_exact(path:Path,before:bytes,after:bytes)->dict[str,Any]:
 fd=os.open(path,os.O_RDWR|os.O_NOFOLLOW|os.O_CLOEXEC)
 try:
  st=os.fstat(fd)
  if not stat.S_ISREG(st.st_mode) or st.st_nlink!=1:raise Error('patch target unsafe')
  os.lseek(fd,0,os.SEEK_SET);actual=os.read(fd,st.st_size)
  if actual!=before:raise Error('patch preimage mismatch '+path.name)
  os.ftruncate(fd,0);os.lseek(fd,0,os.SEEK_SET);os.write(fd,after);os.fsync(fd)
 finally:os.close(fd)
 d=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:os.fsync(d)
 finally:os.close(d)
 return file_id(path)
def patch_six(source:Path)->list[dict[str,Any]]:
 policy();v2=module(PV2,'patchv2');v1=v2.load_v1(v2.read_json(v2.POLICY_PATH));before=v1.load_upstream();after=v1.apply_in_memory(before)
 if len(before)!=6 or set(before)!=set(after):raise Error('exact six patch contract drift')
 result=[]
 for name in sorted(before):
  result.append(fd_replace_exact(source/name,before[name],after[name]))
 if v1.aggregate_source_hash({k:(source/k).read_bytes() for k in sorted(before)})!=v1.aggregate_source_hash(after):raise Error('patched aggregate drift')
 return result
def resource(start:float)->dict[str,Any]:
 vals={}
 for line in Path('/proc/meminfo').read_text().splitlines():
  a=line.split();
  if len(a)>1 and a[0].rstrip(':') in ('MemAvailable','SwapFree'):vals[a[0].rstrip(':')]=int(a[1])*1024
 psi=Path('/proc/pressure/memory').read_text();vm={}
 for l in Path('/proc/vmstat').read_text().splitlines():
  a=l.split();
  if len(a)==2 and a[0] in ('pswpin','pswpout'):vm[a[0]]=int(a[1])
 rss=0
 for x in Path('/proc').glob('[0-9]*'):
  try:
   if (x/'comm').read_text().strip() in ('cicc','ptxas','cc1plus'):rss+=int((x/'statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
  except (OSError,ValueError,IndexError):pass
 temps=[]
 for x in Path('/sys/class/thermal').glob('thermal_zone*'):
  try:temps.append((x/'temp').read_text().strip())
  except OSError:pass
 return {'elapsed':round(time.monotonic()-start,6),'memavailable':vals.get('MemAvailable',0),'swapfree':vals.get('SwapFree',0),'psi_some':next((x for x in psi.splitlines() if x.startswith('some ')),''),'psi_full':next((x for x in psi.splitlines() if x.startswith('full ')),''),'pswpin':vm.get('pswpin',0),'pswpout':vm.get('pswpout',0),'compiler_rss':rss,'temperature':temps}
def run_bounded(argv:list[str],*,executor:Callable[[list[str]],tuple[int,bytes,bytes]]|None=None,limit:int=268435456)->tuple[int,bytes,bytes]:
 if executor:
  rc,out,err=executor(argv)
  if len(out)+len(err)>limit:raise Error('parser output limit')
  return rc,out,err
 p=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True);start=time.monotonic()
 try:
  out,err=p.communicate(timeout=5400)
 except subprocess.TimeoutExpired:
  os.killpg(p.pid,signal.SIGTERM)
  try:out,err=p.communicate(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);out,err=p.communicate()
  raise Error('process group timeout')
 if len(out)+len(err)>limit:raise Error('parser output limit')
 return p.returncode,out,err
def static_audit(root:Path,executor:Callable[[list[str]],tuple[int,bytes,bytes]])->dict[str,Any]:
 p,_=policy();g1=module(G1,'g1');base=json.loads(g1.POLICY_PATH.read_text());old=str(g1.ROOT_A);plan=[];cand=str(g1.OUTPUT_A/'artifacts/DualSPHysics5.4_linux64')
 for x in base['static_audit_contract']['candidate_tool_suffixes']:plan.append(g1.build_static_audit_argv(cand,x['id'],base))
 cuda=set(base['object_contract']['cuda_object_names'])
 for name in base['object_contract']['object_names']:
  host=str(g1.ROOT_A/'output/buildtree/src/source'/name)
  for x in base['static_audit_contract']['object_tool_suffix_templates']:
   if not x.get('cuda_only') or name in cuda:plan.append(g1.build_static_audit_argv(host,x['id'],base))
 if len(plan)!=p['contracts']['static_commands']:raise Error('557 plan drift')
 candidate=root/'output/artifacts/DualSPHysics5.4_linux64';objects=[root/'output/buildtree/src/source'/x for x in base['object_contract']['object_names']];before=[file_id(candidate),*[file_id(x) for x in objects]];total=0;start=time.monotonic()
 for argv in plan:
  rc,out,err=run_bounded([x.replace(old,str(root)) for x in argv],executor=executor,limit=p['limits']['static_output_per_command_bytes']);total+=len(out)+len(err)
  if rc or total>p['limits']['static_output_total_bytes']:raise Error('static parser failure/output aggregate')
 after=[file_id(candidate),*[file_id(x) for x in objects]]
 if before!=after:raise Error('candidate/object identity changed')
 return {'commands':len(plan),'output_bytes':total,'samples':[resource(start)],'candidate_before':before[0],'candidate_after':after[0],'objects_unchanged':True}
def self_check():
 p,h=policy();return {'status':'PASS_V6_REAL_PATCH_STATIC_BINDING_DEFAULT_DENY','policy_sha256':h,'patch_files':p['contracts']['patch_files'],'static_commands':p['contracts']['static_commands'],'files_written':False,'make_run':False,'candidate_executed':False}
if __name__=='__main__':print(json.dumps(self_check(),sort_keys=True))
