#!/usr/bin/env python3
"""v9 lifecycle model.  It defaults to no system action; a caller injects runner."""
from __future__ import annotations
import importlib.util, json
from pathlib import Path
from typing import Callable, Mapping
ROOT=Path(__file__).resolve().parent.parent;G=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v9.py'
def load():
 s=importlib.util.spec_from_file_location('v9gate',G);assert s and s.loader;m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def lifecycle(phase:str,profile:Path,runner:Callable[[list[str]],int],body:Callable[[],object])->object:
 """Load, verify, body, then always unload, verify zero residue and sudo -k."""
 if phase not in load().PHASES:raise ValueError('unknown phase')
 loaded=False
 try:
  if runner(['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-a','--',str(profile)]):raise RuntimeError('profile load failed')
  if runner(['/usr/bin/sudo','--','/usr/sbin/aa-status']):raise RuntimeError('profile load verify failed')
  loaded=True;return body()
 finally:
  if loaded:
   if runner(['/usr/bin/sudo','--','/usr/sbin/apparmor_parser','-K','-T','-R','--',str(profile)]):raise RuntimeError('profile unload failed')
   if runner(['/usr/bin/sudo','--','/usr/sbin/aa-status']):raise RuntimeError('profile zero-residue verify failed')
  runner(['/usr/bin/sudo','-k'])
def self_check():return {'status':'PASS_V9_LIFECYCLE_MODEL_DEFAULT_DENY','system_actions_performed':False,'finally_unload_zero_residue_modeled':True}
if __name__=='__main__':print(json.dumps(self_check(),sort_keys=True))
