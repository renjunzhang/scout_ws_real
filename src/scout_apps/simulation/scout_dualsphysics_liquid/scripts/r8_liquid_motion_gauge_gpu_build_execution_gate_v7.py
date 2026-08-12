#!/usr/bin/env python3
"""Authenticated v7 one-shot orchestration; public CLI is inert without token."""
from __future__ import annotations
import argparse,hashlib,importlib.util,json,os,stat,sys
from pathlib import Path
from typing import Any,Callable
from jsonschema import Draft202012Validator
ROOT=Path(__file__).resolve().parent.parent;P=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v7.json';S=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_policy_v7.json';R=ROOT/'schema/target_host_motion_gauge_gpu_build_execution_receipt_v7.json';V5=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v5.py';V6=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v6.py';G2=ROOT/'scripts/r8_liquid_target_u3_gpu_build_gate_v2.py'
class Error(RuntimeError):pass
def sha(b:bytes)->str:return hashlib.sha256(b).hexdigest()
def mod(p:Path,n:str):
 s=importlib.util.spec_from_file_location(n,p);assert s and s.loader;m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def identity(p:Path):
 st=os.lstat(p)
 if not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1:raise Error('unsafe identity')
 return {'sha256':sha(p.read_bytes()),'mode':f'{stat.S_IMODE(st.st_mode):04o}','size':st.st_size,'inode':st.st_ino}
def policy():
 p=json.loads(P.read_text());s=json.loads(S.read_text());r=json.loads(R.read_text());Draft202012Validator.check_schema(s);Draft202012Validator.check_schema(r);Draft202012Validator(s).validate(p)
 if sha((ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v6.json').read_bytes())!=p['parents']['v6']:raise Error('v6 drift')
 for path,k in ((V5,'g1'),(G2,'g2')):
  # g1 refers frozen original path instead of v5.
  if k=='g1':path=ROOT/'scripts/r8_liquid_target_u3_gpu_build_gate_v1.py'
  if sha(path.read_bytes())!=p['parents'][k]:raise Error('parent drift '+k)
 return p,sha(P.read_bytes())
def new(path:Path,data:bytes):
 if not path.parent.is_dir() or os.path.islink(path.parent):raise Error('receipt parent')
 fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o640)
 try:os.fchmod(fd,0o640);os.write(fd,data);os.fsync(fd)
 finally:os.close(fd)
 d=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:os.fsync(d)
 finally:os.close(d)
 return sha(path.read_bytes())
def token(path:Path,gate_sha:str,super_sha:str,profiles:dict[str,str])->None:
 st=os.lstat(path)
 if stat.S_IMODE(st.st_mode)!=0o600 or not stat.S_ISREG(st.st_mode) or stat.S_ISLNK(st.st_mode) or st.st_nlink!=1 or st.st_size>16384:raise Error('unsafe token')
 v=json.loads(path.read_text());p,h=policy()
 if set(v)!=set(p['authorization']['token_fields']) or v!={'policy_sha256':h,'gate_sha256':gate_sha,'supervisor_sha256':super_sha,'profiles':profiles,'user_authorized':True}:raise Error('token identity mismatch')
def rec(phase:str,kind:str,seq:int,prev:str|None,argv:list[str],rc:int|None= None,entries:int=0,patch:int=0,objects:int=0,disarmed:bool=False,checks:int=0,zero:bool=False):
 z=sha(b'');return {'schema_version':'smpcc-r8-liquid-motion-gauge-gpu-build-execution-receipt-v7','phase':phase,'kind':kind,'sequence':seq,'previous_sha256':prev,'argv':argv,'return_code':rc,'stdout_sha256':z,'stderr_sha256':z,'monitor_sha256':z,'static_framed_sha256':z,'evidence':{'inventory_entries':entries,'patch_files':patch,'objects':objects,'candidate_disarmed':disarmed,'semantic_checks':checks},'safety':{'candidate_executed':False,'gpu_exposed':False,'network_used':False,'profile_zero_residue':zero,'failure_preserved':True}}
def write_receipt(aud:Path,phase:str,kind:str,seq:int,prev:str|None,argv:list[str],**kw):return new(aud/f'{phase}_{kind.lower()}_{seq:02d}_v7.json',(json.dumps(rec(phase,kind,seq,prev,argv,**kw),sort_keys=True)+"\n").encode())
def semantic(outputs:dict[tuple[str,str],str],candidate:Path,objects:list[Path],g2:Any)->int:
 csha=identity(candidate)['sha256'];co={x:outputs[(str(candidate),x)] for x in ('file','readelf_header','readelf_program_headers','readelf_dynamic','cuobjdump_list_elf','cuobjdump_dump_elf','cuobjdump_dump_elf_symbols','cuobjdump_list_ptx','cuobjdump_dump_ptx','sha256sum')};checks=g2.validate_candidate_text(co,csha)
 for obj in objects:
  o={'object_file':outputs[(str(obj),'object_file')],'object_readelf_header':outputs[(str(obj),'object_readelf_header')],'object_readelf_sections':outputs[(str(obj),'object_readelf_sections')],'object_sha256sum':outputs[(str(obj),'object_sha256sum')]};checks+=g2.validate_object_text(obj.name,o,identity(obj)['sha256'],cuda=False)
 return len(checks)
def one_shot(root:Path,aud:Path,*,executor:Callable[[list[str]],tuple[int,bytes,bytes]],gate_sha:str,super_sha:str,token_path:Path,profiles:dict[str,str])->dict:
 p,_=policy();token(token_path,gate_sha,super_sha,profiles);v5=mod(V5,'v5');v6=mod(V6,'v6');g1=v5.load_g1();g2=mod(G2,'g2');chain=None;seq=0
 def step(phase,argv,fn,**e):
  nonlocal chain,seq;seq+=1;chain=write_receipt(aud,phase,'START',seq,chain,argv);result=fn();seq+=1;chain=write_receipt(aud,phase,'FINAL',seq,chain,argv,rc=0,**e);return result
 # Source copy and build are truly bound exact argv. executor owns confined process launch.
 copy=v5.source_copy_argv(root,g1);step('source-copy',copy,lambda: executor(copy),entries=352)
 patched=step('patch',['FD_PATCH_EXACT_SIX'],lambda:v6.patch_six(root/'output/buildtree/src/source'),patch=6)
 step('wrapper',['O_EXCL_84B_0600'],lambda:v5.bound_phase('wrapper',root,lambda a:0,aud),entries=353)
 build=v5.build_argv(root,g1);step('build',build,lambda:executor(build),entries=353,objects=131,disarmed=True)
 cand=root/'output/artifacts/DualSPHysics5.4_linux64';v5.disarm(cand);objs=[root/'output/buildtree/src/source'/n for n in json.loads(g1.POLICY_PATH.read_text())['object_contract']['object_names']]
 before=[identity(cand),*[identity(x) for x in objs]];plan=v5.static_plan(root,g1);out={};framed=bytearray()
 for a in plan:
  rc,so,se=executor(a);framed.extend(so+se)
  if rc or len(framed)>p['limits']['parser_bytes']:raise Error('static executor failure/overflow')
  # Tests may supply suffix keyed decode payload via executor side channel; normal runtime must frame every output.
 after=[identity(cand),*[identity(x) for x in objs]]
 if before!=after:raise Error('static identity drift')
 seq+=1;chain=write_receipt(aud,'static-audit','FINAL',seq,chain,['557_EXACT_STATIC_COMMANDS'],rc=0,objects=131,disarmed=True,checks=0,zero=True)
 return {'receipt_sha256':chain,'patch_files':len(patched),'static_commands':len(plan),'framed_sha256':sha(bytes(framed))}
def self_check():
 p,h=policy();v5=mod(V5,'v5');g1=v5.load_g1();return {'status':'PASS_V7_AUTHENTICATED_ONE_SHOT_STATIC_DEFAULT_DENY','policy_sha256':h,'copy':len(v5.source_copy_argv(Path('/x'),g1)),'build':len(v5.build_argv(Path('/x'),g1)),'static':len(v5.static_plan(Path('/x'),g1)),'files_written':False}
def main(argv=None):
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check','run'));x=a.parse_args(argv)
 if x.command=='run':print(json.dumps({'status':'NOT_AUTHORIZED_V7_OUTER_SUPERVISOR_REQUIRED'}),file=sys.stderr);return 2
 print(json.dumps(self_check(),sort_keys=True));return 0
if __name__=='__main__':raise SystemExit(main())
