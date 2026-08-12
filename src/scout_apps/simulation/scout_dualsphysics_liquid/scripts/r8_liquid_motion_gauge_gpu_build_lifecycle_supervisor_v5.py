#!/usr/bin/env python3
"""Outer v5 lifecycle: exact profile load, setpriv child, finally unload/status/k."""
from __future__ import annotations
import importlib.util,json,sys
from pathlib import Path
from typing import Callable
ROOT=Path(__file__).resolve().parent.parent;GATE=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v5.py'
s=importlib.util.spec_from_file_location('mg5',GATE);assert s and s.loader;gate=importlib.util.module_from_spec(s);s.loader.exec_module(gate)
PROFILE_PATHS={'source-copy':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-source-copy-20260812t034446z-v3.profile','patch':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-patch-20260812t034446z-v3.profile','build':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-build-20260812t034446z-v3.profile','static-audit':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-static-audit-20260812t034446z-v3.profile'}
def run_phase(phase:str,root:Path,audits:Path,executor:Callable[[list[str]],int],lifecycle:Callable[[list[str]],int])->dict:
 if phase=='wrapper': return {'phase':gate.bound_phase(phase,root,executor,audits),'lifecycle':[]}
 if phase not in PROFILE_PATHS:raise ValueError('unknown phase')
 prof=ROOT/PROFILE_PATHS[phase];events=[];loaded=False
 try:
  for a in (['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-a','--',str(prof)],['/usr/bin/sudo','--','/usr/sbin/aa-status']):
   rc=lifecycle(a);events.append((a,rc))
   if rc:raise RuntimeError('profile load/status failure')
  loaded=True
  return {'phase':gate.bound_phase(phase,root,executor,audits),'lifecycle':events}
 finally:
  if loaded:
   for a in (['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-R','--',str(prof)],['/usr/bin/sudo','--','/usr/sbin/aa-status']):events.append((a,lifecycle(a)))
  events.append((['/usr/bin/sudo','-k'],lifecycle(['/usr/bin/sudo','-k'])))
def main()->int:print(json.dumps({'status':'NOT_AUTHORIZED_V5_OUTER_ONLY','self_check':gate.self_check()},sort_keys=True));return 0
if __name__=='__main__':raise SystemExit(main())
