import importlib.util,json
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
def load(n,p):
 s=importlib.util.spec_from_file_location(n,ROOT/p);m=importlib.util.module_from_spec(s);assert s.loader;s.loader.exec_module(m);return m
g=load('g5','scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v5.py');v=load('v5','scripts/r8_liquid_motion_gauge_gpu_build_profile_validator_v5.py');sup=load('s5','scripts/r8_liquid_motion_gauge_gpu_build_lifecycle_supervisor_v5.py')
def test_static_contract():
 r=g.self_check();assert r['static_commands']==557 and r['build_argv_count']>20
def test_profiles():assert v.validate()['status']=='PASS_V5_PROFILE_VALIDATION'
def test_wrapper_o_excl_and_finally(tmp_path,monkeypatch):
 monkeypatch.setattr(g,'inventory',lambda *a,**k:{'entries':352,'sealed':352,'wrapper':0,'objects':0} if not k.get('allow_wrapper') else {'entries':353,'sealed':352,'wrapper':1,'objects':0});sup.gate.inventory=g.inventory
 root=tmp_path/'root';src=root/'output/buildtree/src/source';src.mkdir(parents=True);aud=tmp_path/'audit';aud.mkdir();seen=[]
 r=sup.run_phase('wrapper',root,aud,lambda a:0,lambda a:seen.append(a) or 0)
 assert (src/'U3GpuBuild.mk').read_bytes()==g.load_g1().WRAPPER_BYTES
 assert seen==[] # wrapper is profile-free and therefore has no lifecycle operation
 with pytest.raises(FileExistsError):g.bound_phase('wrapper',root,lambda a:0,aud)
def test_build_candidate_disarm(tmp_path,monkeypatch):
 monkeypatch.setattr(g,'inventory',lambda *a,**k:{'entries':353,'sealed':352,'wrapper':1,'objects':0})
 root=tmp_path/'r';c=root/'output/artifacts/DualSPHysics5.4_linux64';c.parent.mkdir(parents=True);c.write_bytes(b'x');aud=tmp_path/'a';aud.mkdir()
 r=g.bound_phase('build',root,lambda a:0,aud);assert r['candidate']['mode']=='0400'
