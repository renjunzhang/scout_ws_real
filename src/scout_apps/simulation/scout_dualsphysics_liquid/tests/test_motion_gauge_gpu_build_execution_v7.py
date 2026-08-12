import importlib.util,json,os
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
def load(n,p):
 s=importlib.util.spec_from_file_location(n,ROOT/p);m=importlib.util.module_from_spec(s);assert s.loader;s.loader.exec_module(m);return m
g=load('g7','scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v7.py');sup=load('s7','scripts/r8_liquid_motion_gauge_gpu_build_lifecycle_supervisor_v7.py')
def test_static():assert g.self_check()['static']==557
def test_token_exact_mode_and_hashes(tmp_path):
 p,h=g.policy();t=tmp_path/'t';t.write_text('{}');os.chmod(t,0o600)
 with pytest.raises(g.Error):g.token(t,'a'*64,'b'*64,{})
def test_receipt_oexcl_chain(tmp_path):
 a=tmp_path/'a';a.mkdir();h=g.write_receipt(a,'patch','START',1,None,['x']);assert len(h)==64
 with pytest.raises(FileExistsError):g.write_receipt(a,'patch','START',1,None,['x'])
def test_finally_failure():
 seen=[]
 with pytest.raises(ValueError):sup.lifecycle(ROOT/'x',lambda a:seen.append(a) or 0,lambda:(_ for _ in ()).throw(ValueError()))
 assert any('-R' in x for x in seen) and seen[-1]==['/usr/bin/sudo','-k']
