#!/usr/bin/env python3
from __future__ import annotations
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
P=ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v5.json'
PATHS={'source-copy':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-source-copy-20260812t034446z-v3.profile','patch':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-patch-20260812t034446z-v3.profile','build':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-build-20260812t034446z-v3.profile','static-audit':'config/apparmor_drafts/r8-liquid-motion-gauge-gpu-static-audit-20260812t034446z-v3.profile'}
def validate():
 p=json.loads(P.read_text());out={}
 for role,path in PATHS.items():
  raw=(ROOT/path).read_bytes();text=raw.decode();h=hashlib.sha256(raw).hexdigest()
  if h!=p['profiles'][role]:raise ValueError('profile hash drift')
  if any(x in text for x in ('/dev/nvidia','network inet stream','network inet6 stream','g++-13','flags=(unconfined)','*.o')):raise ValueError('forbidden profile surface')
  if 'userns create,' not in text or 'pivot_root' not in text:raise ValueError('missing confined pivot')
  if role=='static-audit' and text.count('.o r,')!=131:raise ValueError('not exact 131 audit objects')
  if role=='patch' and '/newroot/work/tmp/' not in text:raise ValueError('patch tmp delta absent')
  out[role]=h
 return {'status':'PASS_V5_PROFILE_VALIDATION','profiles':out,'profile_loaded':False}
if __name__=='__main__:':pass
