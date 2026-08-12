#!/usr/bin/env python3
"""Fresh v7 wrapper fixing the extractor interface while preserving v6 evidence."""
import argparse,copy,hashlib,json,os,stat,sys
from pathlib import Path
from jsonschema import Draft202012Validator
MODULE_DIR=Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:sys.path.insert(0,str(MODULE_DIR))
import r8_liquid_s5a1_execution_supervisor_v6 as v6
import r8_liquid_s5a1_handoff_gate_v5 as gate
import r8_liquid_s5a1_ros1_signal_extractor_v4 as extractor
sys.dont_write_bytecode=True
base=v6.base;ROOT=MODULE_DIR.parent;SCRIPT=Path(__file__).resolve();TESTS=ROOT/"tests/test_s5a1_execution_supervisor_v7.py"
EXECUTION_ID="s5a1_primary_bsmooth_b01_20260811T214500Z_v7";TRANSFER_ID=gate.TRANSFER_ID
POLICY=ROOT/"config/target_hosts/liquid_zrj_msi_u2404_s5a1_execution_policy_v7.json";POLICY_SCHEMA=ROOT/"schema/target_host_s5a1_execution_policy_v7.json";RECEIPT_SCHEMA=ROOT/"schema/target_host_s5a1_execution_receipt_v7.json"
PARTIAL_ROOT=Path(gate.PARTIAL_ROOT);FINAL_ROOT=Path(gate.FINAL_ROOT);RECEIPT=Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v7.json");RECEIPT_PARTIAL=Path(str(RECEIPT)+".partial")
V6_RECEIPT=Path("/home/zrj/scout_liquid_lab/audits/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v2.execution_v6.json");V6_PARTIAL=Path("/home/zrj/scout_liquid_lab/incoming/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v2.partial")
ACTIVE=None;ACTIVE_ID=None
class Error(ValueError):pass
def ident(p):
 st=os.lstat(p);raw=p.read_bytes();return {"path":str(p),"sha256":hashlib.sha256(raw).hexdigest(),"mode":f"{stat.S_IMODE(st.st_mode):04o}","size_bytes":st.st_size}
def preserved():
 r=ident(V6_RECEIPT);st=os.lstat(V6_PARTIAL)
 if r["sha256"]!="3051aedaf265a388e949a27807436d31b544fff7efa87a3433ef8ee081ef2408" or any(V6_PARTIAL.iterdir()):raise Error("v6 evidence drifted")
 return {"failure_receipt":r,"partial_root":{"path":str(V6_PARTIAL),"inode":st.st_ino,"mode":"0700","size_bytes":st.st_size,"nlink":st.st_nlink,"entry_count":0,"inventory_sha256":hashlib.sha256(b"").hexdigest()}}
def receipt_schema():
 d=json.loads(RECEIPT_SCHEMA.read_bytes());b=Path(d["base"]["path"]);raw=b.read_bytes()
 if hashlib.sha256(raw).hexdigest()!=d["base"]["sha256"]:raise Error("v6 receipt schema drifted")
 s=json.loads(raw);s["$id"]="https://scout.local/schema/target_host_s5a1_execution_receipt_v7.json";s["properties"]["execution_id"]={"const":EXECUTION_ID};s["properties"]["receipt_id"]={"const":d["receipt_id"]};s["required"] += ["preserved_v6_failure","v7_contract"]
 s["properties"]["preserved_v6_failure"]={"type":"object"};s["properties"]["v7_contract"]={"type":"object"};return s
def admit(expected=None):
 global ACTIVE,ACTIVE_ID
 raw,full=base.read_file_identity(POLICY,expected,maximum_bytes=2**22);p=json.loads(raw);Draft202012Validator(json.loads(POLICY_SCHEMA.read_bytes())).validate(p)
 if sorted(os.getgroups())!=v6.HOST_GROUPS:raise Error("host groups drifted")
 for k,path in {"extractor_v4":Path(extractor.__file__).resolve(),"extractor_v4_tests":ROOT/"tests/test_s5a1_ros1_signal_extractor_v4.py"}.items():
  if ident(path)!=p[k]:raise Error(k+" drifted")
 for k,path in {"policy":gate.POLICY_PATH,"schema":gate.SCHEMA_PATH,"gate":Path(gate.__file__).resolve(),"tests":ROOT/"tests/test_s5a1_handoff_gate_v5.py"}.items():
  if ident(path)!=p["package_v3"][k]:raise Error(k+" drifted")
 if preserved()!=p["preserved_v6"]:raise Error("v6 preserved differs")
 ACTIVE=p;ACTIVE_ID={"path":str(POLICY),"sha256":hashlib.sha256(raw).hexdigest()};configure();a=raw_build(v6.v5.prior.S5A0_INNER_RECEIPT,Path("/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"),PARTIAL_ROOT,receipt_fd=101,source_fd=102);c=v6._canonical(a,101,102)
 if v6._argv_hash(c)!=p["canonical_argv_sha256"] or len(c)!=p["canonical_token_count"]:raise Error("argv drifted")
 return p
def configure():
 v6._configure_paths();gate.install();base.SCRIPT=SCRIPT;base.POLICY=gate.POLICY_PATH;base.PACKAGE_SCHEMA=gate.SCHEMA_PATH;base.GATE=Path(gate.__file__).resolve();base.gate=gate;base.EXTRACTOR=Path(extractor.__file__).resolve();base.extractor=extractor;base.TRANSFER_ID=TRANSFER_ID;base.PARTIAL_ROOT=PARTIAL_ROOT;base.FINAL_ROOT=FINAL_ROOT;base.EXECUTION_RECEIPT=RECEIPT;base.EXECUTION_RECEIPT_PARTIAL=RECEIPT_PARTIAL
 v6.PARTIAL_ROOT=PARTIAL_ROOT;v6.FINAL_ROOT=FINAL_ROOT;v6.EXECUTION_ID=EXECUTION_ID
def raw_build(receipt,source,partial,*,receipt_fd,source_fd):
 a=v6._raw_build(receipt,source,partial,receipt_fd=receipt_fd,source_fd=source_fd);i=a.index("--bind");extra=[]
 for p in (Path(v6.__file__).resolve(),Path(extractor.extractor_v3.__file__).resolve(),Path(gate.base.__file__).resolve()):
  if str(p) not in a:extra += ["--ro-bind",str(p),str(p)]
 a[i:i]=extra;return a
def install():
 configure();base.build_bwrap_argv=build;base.load_package=load;base.reserve_receipt=lambda:v6._BASE_RESERVE(RECEIPT_PARTIAL);base.publish_reserved_receipt=publish;base.mock_failure_receipt=lambda:transform(v6._BASE_MOCK_FAILURE());base.subprocess=v6._SUBPROCESS_PROXY
def build(receipt,source,partial,*,receipt_fd=None,source_fd=None):
 before=(base.publish_reserved_receipt,base.mock_failure_receipt);a=raw_build(receipt,source,partial,receipt_fd=receipt_fd,source_fd=source_fd)
 if before!=(base.publish_reserved_receipt,base.mock_failure_receipt):raise Error("hooks clobbered")
 c=v6._canonical(a,receipt_fd,source_fd)
 if v6._argv_hash(c)!=ACTIVE["canonical_argv_sha256"]:raise Error("runtime argv drifted")
 return a
def load(root):
 package=v6._BASE_LOAD_PACKAGE(root);report=gate.validate_package_root(root);v6._SEMANTIC_ROOTS.append(str(root));v6._SEMANTIC_REPORT=report;return package
def transform(value):
 value=copy.deepcopy(value);value["execution"]["supplementary_groups"]=v6.HOST_GROUPS
 out=v6.transform_receipt(value);out["schema_version"]="smpcc-r8-liquid-s5a1-execution-receipt-v7";out["document_type"]="SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V7";out["execution_id"]=EXECUTION_ID;out["receipt_id"]=f"{TRANSFER_ID}_execution_v7";out["preserved_v6_failure"]=preserved();out["v7_contract"]={"execution_policy":ACTIVE_ID,"extractor_v4":{"path":ACTIVE["extractor_v4"]["path"],"sha256":ACTIVE["extractor_v4"]["sha256"]}}
 return out
def publish(fd,partial,final,value):
 out=transform(value);Draft202012Validator(receipt_schema()).validate(out);v6._BASE_PUBLISH(fd,partial,final,out)
def worker(receipt,source,partial):
 if os.getgroups()!=v6.MAPPED_GROUPS:raise Error("worker groups drifted")
 configure();report=v6._BASE_WORKER(receipt,source,partial);report["worker_supplementary_groups"]=v6.MAPPED_GROUPS;return report
def self_check():
 global ACTIVE
 p=admit();v6._ACTIVE_POLICY=copy.deepcopy(v6._load_policy(p["base_v6_policy"]["sha256"])[0]);v6._ACTIVE_POLICY["sandbox"]["canonical_argv_contract"].update(sha256=p["canonical_argv_sha256"],token_count=p["canonical_token_count"]);v6._ACTIVE_POLICY["package_revision"]={"policy_delta":p["package_v3"]["policy"],"schema_delta":p["package_v3"]["schema"],"gate":p["package_v3"]["gate"],"tests":p["package_v3"]["tests"]};v6._ACTIVE_POLICY_IDENTITY=ACTIVE_ID;install();Draft202012Validator.check_schema(receipt_schema());return {"status":"PASS_S5A1_EXECUTION_SUPERVISOR_V7_STATIC_CONTRACT","extractor_interface":all(callable(getattr(extractor,n,None)) for n in ("read_exact_primary","nearest_clock_alignment","tf_cross_check")),"real_bag_read":False,"bwrap_run":False,"files_written":False}
def main():print(json.dumps(self_check(),sort_keys=True,separators=(",",":")));return 0
if __name__=="__main__":raise SystemExit(main())
