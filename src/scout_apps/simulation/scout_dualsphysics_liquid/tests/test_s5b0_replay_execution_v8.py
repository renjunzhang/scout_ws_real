#!/usr/bin/env python3
from __future__ import annotations
import importlib.util,json,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;S=ROOT/'scripts'
def load(n,f):
 x=importlib.util.spec_from_file_location(n,S/f);m=importlib.util.module_from_spec(x);sys.modules[n]=m;x.loader.exec_module(m);return m
g=load('g8','r8_liquid_s5b0_replay_execution_gate_v8.py');r=load('r8','r8_liquid_s5b0_replay_runtime_supervisor_v8.py')
class Fake:
 def __init__(self,fail=False):self.calls=[];self.fail=fail;self.loaded_flag=False
 def run(self,a,**k):self.calls.append(list(a));self.loaded_flag=self.loaded_flag or '-a' in a;return 1 if self.fail and '-a' in a else 0
 def loaded(self,n):return self.loaded_flag and not any('-R' in a for a in self.calls)
 def sample(self):return {'free_vram_bytes':6442450944,'output_bytes':1,'xid_count':0}
 def kill_pgid(self):pass
class T(unittest.TestCase):
 def test_static(self):
  x=g.self_check();self.assertFalse(x['files_written']);self.assertEqual(x['status'],'PASS_S5B0_V8_STATIC_GATE_NOT_AUTHORIZED')
 def test_monitor_negative(self):
  p,_=g.read_policy()
  with self.assertRaises(r.V8RuntimeError):r.monitor(Fake(),p['limits'],[0])
 def test_package_shape(self):
  gauges={f'GaugesSwl_s5b0_p{i:02d}.csv':b'time;zsurf\n0;0.1\n' for i in range(16)}
  frames=[{'index':i,'time_s':float(i),'relative_path':f'data/Part_{i:04d}.bi4','sha256':'0'*64,'particle_count':9078,'ids_sha256':'1'*64,'class_counts':{'fixed_boundary':0,'moving_boundary':2669,'floating':0,'fluid':6409}} for i in range(3)]
  p=r.package(gauges,frames,candidate_sha='2'*64);self.assertEqual(set(('execution_receipt.json','result_qc.json','frame_manifest.json','native_gauge_manifest.json','checksums.sha256'))<=set(p),True)
if __name__=='__main__':unittest.main()
