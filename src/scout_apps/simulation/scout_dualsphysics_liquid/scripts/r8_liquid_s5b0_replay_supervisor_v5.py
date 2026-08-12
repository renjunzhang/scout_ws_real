#!/usr/bin/env python3
"""Non-executing v5 lifecycle planner; real execution is deliberately unavailable."""
from __future__ import annotations
import argparse,json,sys
from pathlib import Path
from typing import Any,Callable,Mapping,Sequence
sys.dont_write_bytecode=True
MODULE=Path(__file__).resolve().parent
if str(MODULE) not in sys.path:sys.path.insert(0,str(MODULE))
import r8_liquid_s5b0_replay_execution_gate_v5 as gate
class SupervisorV5Error(ValueError):pass
def validate_fresh_targets(paths:Mapping[str,str],exists:Callable[[str],bool])->None:
    required={'partial_root','final_root','start_receipt','final_receipt','failure_receipt'}
    if set(paths)!=required or len(set(paths.values()))!=5:raise SupervisorV5Error('target set aliases or differs')
    for name,value in paths.items():
        if not value.startswith('/') or str(Path(value))!=value or exists(value):raise SupervisorV5Error(f'unsafe/nonfresh {name}')
def static_plan(policy:Mapping[str,Any])->dict[str,Any]:
    gate.validate(policy)
    return {'status':policy['status'],'lifecycle':['VERIFY_FRESH_PATCHED_CANDIDATE_AND_STATIC_AUDIT','VERIFY_16_NATIVE_RAW_JGAUGESWL_CONTRACT','RESERVE_O_EXCL_RECEIPTS','CREATE_FRESH_ROOTS','DYNAMIC_PREFLIGHT_6GIB_VRAM','LOAD_EXACT_PROFILE_AFTER_USER_AUTHORIZATION','RUN_STAGED_CANDIDATE_ONCE','FINALLY_UNLOAD_PROFILE_AND_ZERO_RESIDUE','VERIFY_16_RAW_GAUGE_CSV_AND_PUBLISH'], 'run_once_only':True,'finally_unload_required':True,'zero_residue_required':True,'runtime_attempted':False}
def run_one_shot(*_a:object,**_k:object)->None:raise SupervisorV5Error('NOT_ADMITTED: exact finalized identity and user authorization required')
def self_check()->dict[str,Any]:
    policy,policy_sha=gate.load_policy();result=static_plan(policy);validate_fresh_targets({'partial_root':'/fixture/r.partial','final_root':'/fixture/r','start_receipt':'/fixture/a.start','final_receipt':'/fixture/a.final','failure_receipt':'/fixture/a.failure'},lambda _:False)
    return {**result,'policy_sha256':policy_sha,'files_written':False,'candidate_executed':False,'solver_executed':False,'gpu_exposed':False,'profile_loaded':False}
def main(argv:Sequence[str]|None=None)->int:
    p=argparse.ArgumentParser();p.add_argument('command',choices=('self-check',));p.parse_args(argv)
    try:print(json.dumps(self_check(),sort_keys=True,separators=(',',':')));return 0
    except Exception as exc:print(str(exc),file=sys.stderr);return 2
if __name__=='__main__':raise SystemExit(main())
