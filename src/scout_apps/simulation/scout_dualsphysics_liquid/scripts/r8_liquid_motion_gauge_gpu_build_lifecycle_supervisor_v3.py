#!/usr/bin/env python3
"""Default-deny outer lifecycle contract; real sudo lifecycle is not invoked by self-check."""
from __future__ import annotations
import argparse, importlib.util, json, sys
from pathlib import Path
from typing import Sequence
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
GATE=ROOT/'scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v3.py'
spec=importlib.util.spec_from_file_location('motion_gate_v3',GATE); assert spec and spec.loader
gate=importlib.util.module_from_spec(spec); spec.loader.exec_module(gate)
class SupervisorError(RuntimeError): pass
def lifecycle_argv(policy,role,phase):
 item=policy['profiles'][role]; prefix=policy['lifecycle']['only_privileged_prefix']
 return {'load':['/usr/bin/sudo','--',*policy['lifecycle']['load'],str(ROOT/item['path'])], 'invoke':[ *prefix,str(GATE),phase], 'unload':['/usr/bin/sudo','--',*policy['lifecycle']['unload'],str(ROOT/item['path'])], 'zero_residue':['/usr/bin/sudo','--','/usr/sbin/aa-status'], 'sudo_k':['/usr/bin/sudo','-k']}
def self_check():
 policy,sha=gate.read_policy(); counts=gate.validate_static(policy)
 phases={'source-copy':'source_copy','patch':'patch','build':'build','static-audit':'static_audit'}
 return {'status':'PASS_V3_LIFECYCLE_STATIC_PLAN_NOT_AUTHORIZED','policy_sha256':sha,'counts':counts,'phase_lifecycle_argv':{phase:lifecycle_argv(policy,role,phase) for phase,role in phases.items()},'finally_unload_required':True,'zero_residue_required':True,'files_written':False,'sudo_used':False,'profile_loaded':False}
def main(argv:Sequence[str]|None=None)->int:
 p=argparse.ArgumentParser();p.add_argument('command',choices=('self-check','run'));a=p.parse_args(argv)
 try:
  if a.command=='run': raise SupervisorError('NOT_AUTHORIZED: user must separately authorize exact v3 policy hash and lifecycle argv')
  r=self_check()
 except Exception as e: print(json.dumps({'status':'FAIL_MOTION_GAUGE_GPU_LIFECYCLE_V3','error':str(e)},sort_keys=True),file=sys.stderr);return 2
 print(json.dumps(r,sort_keys=True,separators=(',',':')));return 0
if __name__=='__main__': raise SystemExit(main())
