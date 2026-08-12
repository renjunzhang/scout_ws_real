import importlib.util
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
def load(n,p):
 s=importlib.util.spec_from_file_location(n,ROOT/p);m=importlib.util.module_from_spec(s);assert s.loader;s.loader.exec_module(m);return m
g=load('g6','scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v6.py');sup=load('s6','scripts/r8_liquid_motion_gauge_gpu_build_lifecycle_supervisor_v6.py')
def test_static():
 r=g.self_check();assert r['patch_files']==6 and r['static_commands']==557
def test_fd_patch_preimage_rejects(tmp_path):
 f=tmp_path/'x';f.write_bytes(b'bad')
 with pytest.raises(g.Error):g.fd_replace_exact(f,b'good',b'new')
def test_fd_patch_replaces_and_hashes(tmp_path):
 f=tmp_path/'x';f.write_bytes(b'old');i=g.fd_replace_exact(f,b'old',b'new');assert f.read_bytes()==b'new' and i['sha256']==g.sha(b'new')
def test_mock_lifecycle_finally():
 seen=[];assert sup.lifecycle('patch',lambda a:seen.append(a) or 0,lambda:'ok')=='ok';assert any('-R' in a for a in seen) and seen[-1]==['/usr/bin/sudo','-k']
def test_mock_bounded_output_rejected():
 with pytest.raises(g.Error):g.run_bounded(['x'],executor=lambda a:(0,b'x'*3,b''),limit=2)
