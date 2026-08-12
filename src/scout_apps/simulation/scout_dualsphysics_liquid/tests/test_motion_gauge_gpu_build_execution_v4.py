import importlib.util,json
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
 s=importlib.util.spec_from_file_location(name,ROOT/path);m=importlib.util.module_from_spec(s);assert s.loader;s.loader.exec_module(m);return m
gate=load('mgv4','scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v4.py')
sup=load('msv4','scripts/r8_liquid_motion_gauge_gpu_build_lifecycle_supervisor_v4.py')
def test_static_admission_and_closed_schemas():
 r=gate.self_check();assert r['status'].startswith('PASS_V4');assert r['contracts']['objects']==131
def test_default_deny_cli():assert gate.main(['build'])==2
def test_profile_exactness_and_no_audit_wildcard():
 p,_=gate.policy()
 for role,item in p['profiles'].items():
  text=(ROOT/item['path']).read_text();assert gate.file_id(ROOT/item['path'])['sha256']==item['sha256'];assert '/dev/nvidia' not in text and '*.o' not in text
 assert (ROOT/p['profiles']['static-audit']['path']).read_text().count('.o r,')==131
def test_mock_lifecycle_finally_and_phase_receipts(tmp_path,monkeypatch):
 p,p_sha=gate.policy(); token=tmp_path/'token.json';token.write_text(json.dumps({'policy_sha256':p_sha,'user_authorized':True}))
 monkeypatch.setattr(gate,'memavailable',lambda:4294967296)
 sup.gate.memavailable=lambda:4294967296
 seen=[]
 def run(argv):seen.append(argv);return 0
 r=sup.lifecycle('build',token=token,runner=run,receipt_dir=tmp_path)
 assert r['result']['make_count']==1
 assert any('-R' in x for x in seen) and seen[-1]==['/usr/bin/sudo','-k']
 assert len(list(tmp_path.glob('*_v4.json')))==2
def test_bad_token_and_create_new_no_overwrite(tmp_path,monkeypatch):
 monkeypatch.setattr(gate,'memavailable',lambda:4294967296)
 p,p_sha=gate.policy(); token=tmp_path/'bad.json';token.write_text(json.dumps({'policy_sha256':'0'*64,'user_authorized':True}))
 with pytest.raises(gate.GateError):gate.phase_run('source-copy',token=token,receipt_dir=tmp_path)
 token.write_text(json.dumps({'policy_sha256':p_sha,'user_authorized':True}))
 gate.phase_run('source-copy',token=token,receipt_dir=tmp_path)
 with pytest.raises(FileExistsError):gate.phase_run('source-copy',token=token,receipt_dir=tmp_path)
