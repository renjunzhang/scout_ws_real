#!/usr/bin/env python3
"""v7 public supervisor stays default-deny; injected lifecycle is testable."""
from __future__ import annotations
import importlib.util,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;G=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v7.py'
s=importlib.util.spec_from_file_location('g7',G);assert s and s.loader;gate=importlib.util.module_from_spec(s);s.loader.exec_module(gate)
def lifecycle(profile:Path,runner,body):
 loaded=False
 try:
  for a in (['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-a','--',str(profile)],['/usr/bin/sudo','--','/usr/sbin/aa-status']):
   if runner(a):raise RuntimeError('load/status failure')
  loaded=True;return body()
 finally:
  if loaded:
   runner(['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-R','--',str(profile)])
   runner(['/usr/bin/sudo','--','/usr/sbin/aa-status'])
  runner(['/usr/bin/sudo','-k'])
def main():print(json.dumps({'status':'NOT_AUTHORIZED_V7_EXACT_TOKEN_REQUIRED','self_check':gate.self_check()},sort_keys=True))
if __name__=='__main__':main()
