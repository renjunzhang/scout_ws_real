#!/usr/bin/env python3
"""v6 default-deny lifecycle with injectable exact sudo command runner."""
from __future__ import annotations
from pathlib import Path
from typing import Callable
import importlib.util,json
ROOT=Path(__file__).resolve().parent.parent;G=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v6.py'
s=importlib.util.spec_from_file_location('g6',G);assert s and s.loader;gate=importlib.util.module_from_spec(s);s.loader.exec_module(gate)
PROFILES={'source-copy':'r8-liquid-motion-gauge-gpu-source-copy-20260812t034446z-v3.profile','patch':'r8-liquid-motion-gauge-gpu-patch-20260812t034446z-v3.profile','build':'r8-liquid-motion-gauge-gpu-build-20260812t034446z-v3.profile','static-audit':'r8-liquid-motion-gauge-gpu-static-audit-20260812t034446z-v3.profile'}
def lifecycle(phase:str,runner:Callable[[list[str]],int],body:Callable[[],object])->object:
 if phase not in PROFILES:raise ValueError('unknown phase')
 profile=ROOT/'config/apparmor_drafts'/PROFILES[phase];loaded=False
 try:
  for a in (['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-a','--',str(profile)],['/usr/bin/sudo','--','/usr/sbin/aa-status']):
   if runner(a):raise RuntimeError('load/status failure')
  loaded=True;return body()
 finally:
  if loaded:
   runner(['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-R','--',str(profile)])
   runner(['/usr/bin/sudo','--','/usr/sbin/aa-status'])
  runner(['/usr/bin/sudo','-k'])
def main():print(json.dumps({'status':'NOT_AUTHORIZED_V6_OUTER_ONLY','self_check':gate.self_check()},sort_keys=True))
if __name__=='__main__':main()
