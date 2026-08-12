#!/usr/bin/env python3
"""Default-deny supervisor with unconditional unload/zero-residue callbacks."""
from __future__ import annotations
import argparse,importlib.util,json,sys
from pathlib import Path
from typing import Callable,Sequence
ROOT=Path(__file__).resolve().parent.parent;GATE=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v4.py'
sp=importlib.util.spec_from_file_location('g',GATE);assert sp and sp.loader;gate=importlib.util.module_from_spec(sp);sp.loader.exec_module(gate)
class SupervisorError(RuntimeError):pass
def lifecycle(phase:str,*,token:Path,runner:Callable[[list[str]],int],receipt_dir:Path|None=None)->dict:
 p,p_sha=gate.policy(); item=p['profiles'][phase]; profile=ROOT/item['path']; loaded=False;events=[]
 try:
  for argv in (['/usr/sbin/apparmor_parser','-K','-T','-a','--',str(profile)],['/usr/sbin/aa-status']):
   rc=runner(argv);events.append({'argv':argv,'rc':rc});
   if rc:raise SupervisorError('load/status failed')
  loaded=True
  result=gate.phase_run(phase,token=token,receipt_dir=receipt_dir)
  return {'result':result,'events':events}
 finally:
  if loaded:
   argv=['/usr/sbin/apparmor_parser','-K','-T','-R','--',str(profile)];events.append({'argv':argv,'rc':runner(argv)})
   argv=['/usr/sbin/aa-status'];events.append({'argv':argv,'rc':runner(argv)})
  events.append({'argv':['/usr/bin/sudo','-k'],'rc':runner(['/usr/bin/sudo','-k'])})
def self_check():return {'status':'PASS_V4_LIFECYCLE_FINALLY_CONTRACT_NOT_AUTHORIZED',**gate.self_check(),'files_written':False}
def main(argv:Sequence[str]|None=None)->int:
 a=argparse.ArgumentParser();a.add_argument('command',choices=('self-check','run'));x=a.parse_args(argv)
 if x.command=='run':print(json.dumps({'status':'FAIL_MOTION_GAUGE_GPU_SUPERVISOR_V4','error':'NOT_AUTHORIZED: exact user token and external lifecycle authority required'}),file=sys.stderr);return 2
 print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
if __name__=='__main__':raise SystemExit(main())
