import importlib.util,json,os
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
 s=importlib.util.spec_from_file_location(name,ROOT/path);m=importlib.util.module_from_spec(s);assert s.loader;s.loader.exec_module(m);return m
g=load('v9','scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v9.py')
sup=load('v9sup','scripts/r8_liquid_motion_gauge_gpu_build_lifecycle_supervisor_v9.py')
def test_static_default_deny_contract():
 r=g.self_check();assert r['static_commands']==557 and r['parallel_jobs']==1 and not r['make_run']
def test_receipt_is_oexcl_and_chained(tmp_path):
 a=tmp_path/'audit';a.mkdir();x=g.emit(a,'patch','START',1,None,['patch'])
 assert len(x['sha256'])==64
 with pytest.raises(FileExistsError):g.emit(a,'patch','START',1,None,['patch'])
def test_token_rejects_non_pinned_content(tmp_path):
 t=tmp_path/'token';t.write_text('{}');os.chmod(t,0o600)
 with pytest.raises(g.Error):g.verify_token(t,g.G1,g.V7,{})
def test_candidate_disarm_and_hardlink_rejection(tmp_path):
 c=tmp_path/'candidate';c.write_bytes(b'candidate');os.chmod(c,0o755)
 assert g.disarm_candidate(c)['mode']=='0400'
 d=tmp_path/'linked';os.link(c,d)
 with pytest.raises(g.Error):g.disarm_candidate(c)
def test_dynamic_gate_rejects_memory_and_conflict():
 with pytest.raises(g.Error):g.dynamic_preflight(0,[])
 with pytest.raises(g.Error):g.dynamic_preflight(1<<50,['make'])
def test_lifecycle_always_unloads_on_failure(tmp_path):
 seen=[]
 with pytest.raises(ValueError):sup.lifecycle('build',tmp_path/'x.profile',lambda a:seen.append(a) or 0,lambda:(_ for _ in ()).throw(ValueError()))
 assert any('-R' in x for x in seen) and seen[-1]==['/usr/bin/sudo','-k']
def test_v4_delta_is_exact_and_narrow():
 p=json.loads((ROOT/'config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v9.json').read_text())
 assert p['profile_contract']['only_delta']==['/newroot/work/tmp/ rw,','/newroot/work/tmp/** rw,']
 assert 'g++-13' in p['build_contract']['forbidden']
def test_static_mapping_is_frozen_557():
 g1=g.g1_module();p=g1.read_json_object(g1.POLICY_PATH);specs=len(p['static_audit_contract']['candidate_tool_suffixes'])
 specs+=sum(len([x for x in p['static_audit_contract']['object_tool_suffix_templates'] if not x.get('cuda_only') or n in set(p['object_contract']['cuda_object_names'])]) for n in p['object_contract']['object_names'])
 assert specs==557
