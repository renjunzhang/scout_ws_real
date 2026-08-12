#!/usr/bin/env python3
"""Deterministic v3_v3 adapter over the frozen gate-v4 mechanism."""
import copy, hashlib, json, sys
from pathlib import Path
from jsonschema import Draft202012Validator
MODULE_DIR=Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path: sys.path.insert(0,str(MODULE_DIR))
import r8_liquid_s5a1_handoff_gate_v4 as base
sys.dont_write_bytecode=True
v1=base.v1; v3=base.v3; HandoffError=base.HandoffError; ROOT=MODULE_DIR.parent
POLICY_PATH=ROOT/"config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v3.json"
SCHEMA_PATH=ROOT/"schema/target_host_s5a1_handoff_package_v3.json"
TRANSFER_ID="SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v3"
PARTIAL_ROOT=f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial"; FINAL_ROOT=f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}"

def expand_policy(raw=None):
    if raw is None: raw,ident=v1.read_regular(POLICY_PATH,maximum=v1.MAX_JSON_BYTES)
    else: ident={"path":str(POLICY_PATH),"sha256":hashlib.sha256(raw).hexdigest(),"size_bytes":len(raw)}
    d=json.loads(raw); expected={"policy_id":"liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v3","transfer_id":TRANSFER_ID,"planned_partial_root":PARTIAL_ROOT,"planned_final_root":FINAL_ROOT}
    if set(d)!={"schema_version","document_type","base","overrides"} or d["overrides"]!=expected: raise HandoffError("v3_v3 policy delta differs")
    raw0,_=base._read_exact(base.BASE_POLICY_PATH,base.BASE_POLICY_SHA256); p=copy.deepcopy(json.loads(raw0));p["policy_id"]=expected["policy_id"];p["transfer_id"]=TRANSFER_ID;p["package"]["planned_partial_root"]=PARTIAL_ROOT;p["package"]["planned_final_root"]=FINAL_ROOT
    return p,ident

def expand_schema():
    raw,ident=v1.read_regular(SCHEMA_PATH,maximum=v1.MAX_JSON_BYTES);d=json.loads(raw);expected={"schema_id":"https://scout.local/schema/target_host_s5a1_handoff_package_v3.json","transfer_id":TRANSFER_ID}
    if d.get("revision_kind")!="DETERMINISTIC_CLOSED_SCHEMA_DELTA_V3" or d.get("overrides")!=expected: raise HandoffError("v3 schema delta differs")
    raw0,_=base._read_exact(base.BASE_SCHEMA_PATH,base.BASE_SCHEMA_SHA256);s=copy.deepcopy(json.loads(raw0));s["$id"]=expected["schema_id"];s["properties"]["transfer_id"]={"const":TRANSFER_ID};Draft202012Validator.check_schema(s);v1.assert_schema_deep_closed(s);return s,ident

def install():
    base.POLICY_PATH=POLICY_PATH;base.SCHEMA_PATH=SCHEMA_PATH;base.TRANSFER_ID=TRANSFER_ID;base.PARTIAL_ROOT=PARTIAL_ROOT;base.FINAL_ROOT=FINAL_ROOT;base.expand_policy=expand_policy;base.expand_schema=expand_schema;base.install()

def validate_package_root(root,**kwargs): install();return base.validate_package_root(root,**kwargs)
def validate_package_bytes(package,schema=None,**kwargs): install();return base.validate_package_bytes(package,schema,**kwargs)
def self_check():
    p,pi=expand_policy();s,si=expand_schema();return {"status":"PASS_S5A1_HANDOFF_GATE_V5_V3_V3_ADAPTER_STATIC_CONTRACT","transfer_id":TRANSFER_ID,"policy_sha256":pi["sha256"],"schema_sha256":si["sha256"],"real_bag_read":False,"files_written":False}
def __getattr__(name): return getattr(base,name)
