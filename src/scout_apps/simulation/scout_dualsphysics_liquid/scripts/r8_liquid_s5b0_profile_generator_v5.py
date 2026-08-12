#!/usr/bin/env python3
"""Deterministic, non-loading v5 S5B0 exact-profile renderer."""
from __future__ import annotations
import argparse, hashlib, re, sys
from pathlib import Path
from typing import Mapping, Sequence
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parent.parent
TEMPLATE_PATH=ROOT/'config/apparmor_drafts/r8-liquid-s5b0-replay-v5.profile.template'
TOKENS=('PROFILE_NAME','STAGED_CANDIDATE','DSPH_CONFIG','LIBCUDA','LIBNVIDIA_PTXJIT','CASE_ROOT','RESTART_ROOT','NVIDIA0','NVIDIACTL','NVIDIAUVM','OUTPUT_ROOT')
COUNTS={key:1 for key in TOKENS}; COUNTS.update({'CASE_ROOT':2,'RESTART_ROOT':2,'OUTPUT_ROOT':2})
DEVICES={'NVIDIA0':'/dev/nvidia0','NVIDIACTL':'/dev/nvidiactl','NVIDIAUVM':'/dev/nvidia-uvm'}
class ProfileV5Error(ValueError): pass
def _path(value:str,label:str)->str:
    if not value.startswith('/') or str(Path(value))!=value or any(c in value for c in '*?[]{}\n\r'): raise ProfileV5Error(f'unsafe {label}')
    return value
def render_profile(template:str,replacements:Mapping[str,str])->str:
    if set(replacements)!=set(TOKENS): raise ProfileV5Error('token set differs')
    if not re.fullmatch(r'r8-liquid-s5b0-[a-z0-9-]{8,80}',replacements['PROFILE_NAME']): raise ProfileV5Error('profile name differs')
    for key in TOKENS[1:]: _path(replacements[key],key)
    for key,value in DEVICES.items():
        if replacements[key]!=value: raise ProfileV5Error(f'device differs: {key}')
    out=template
    for key in TOKENS:
        marker=f'@@{key}@@'
        if out.count(marker)!=COUNTS[key]: raise ProfileV5Error(f'placeholder count differs: {key}')
        out=out.replace(marker,replacements[key])
    if '@@' in out or 'NOT LOADABLE AS-IS' not in out: raise ProfileV5Error('template markers differ')
    writable=[line.strip() for line in out.splitlines() if re.search(r'\b(?:rw|rwk|rix)\b',line)]
    host=[line for line in writable if line.startswith('/') and ' rix,' not in line and not line.startswith('/dev/')]
    expected=[f"{replacements['OUTPUT_ROOT']}/ rw,",f"{replacements['OUTPUT_ROOT']}/** rwk,"]
    if host!=expected: raise ProfileV5Error('output root is not unique writable tree')
    return out
def fixture_replacements()->dict[str,str]:
    return {'PROFILE_NAME':'r8-liquid-s5b0-fixture-replay-v5','STAGED_CANDIDATE':'/stage/runtime/candidate','DSPH_CONFIG':'/stage/runtime/DsphConfig.xml','LIBCUDA':'/runtime/lib/libcuda.so.1','LIBNVIDIA_PTXJIT':'/runtime/lib/libnvidia-ptxjitcompiler.so.1','CASE_ROOT':'/stage/case','RESTART_ROOT':'/stage/restart','NVIDIA0':'/dev/nvidia0','NVIDIACTL':'/dev/nvidiactl','NVIDIAUVM':'/dev/nvidia-uvm','OUTPUT_ROOT':'/output'}
def self_check()->dict[str,object]:
    raw=TEMPLATE_PATH.read_bytes(); rendered=render_profile(raw.decode(),fixture_replacements())
    return {'status':'PASS_S5B0_PROFILE_V5_STATIC_RENDER_ONLY','template_sha256':hashlib.sha256(raw).hexdigest(),'rendered_sha256':hashlib.sha256(rendered.encode()).hexdigest(),'profile_loaded':False,'files_written':False,'device_count':3}
def main(argv:Sequence[str]|None=None)->int:
    p=argparse.ArgumentParser(); p.add_argument('command',choices=('self-check',)); p.parse_args(argv)
    try: print(__import__('json').dumps(self_check(),sort_keys=True,separators=(',',':'))); return 0
    except Exception as exc: print(str(exc),file=sys.stderr); return 2
if __name__=='__main__': raise SystemExit(main())
