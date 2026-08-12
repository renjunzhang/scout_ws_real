#!/usr/bin/env python3
"""Create-new S5B0 v4 staging materializer with 16 native attached SWL Gauges.

The command surface is fixture-only.  The library accepts ``mode="REAL"`` only
when an exact, closed authorization object binds a fresh patched candidate and
all frozen S5B0 parents.  It never executes a candidate, solver, GPU, profile,
sudo command, network operation, or bag reader.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
import r8_liquid_s5b0_staging_case_materializer_v3 as v3


ROOT = MODULE_DIR.parent
SCHEMA_PATH = ROOT / "schema/target_host_s5b0_staging_case_manifest_v4.json"
S5B0_POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_admission_policy_v4.json"
S5B0_POLICY_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_admission_policy_v4.json"
EXPECTED_POLICY_SHA256 = "d972f66d14882c28abe5629828d2ccaab1013f82a65b82f6250cf55716e41f21"
EXPECTED_POLICY_SCHEMA_SHA256 = "897d617b56619b3761025eff64d27f68977ae9d15ac8ac481cea3beaea099d66"
EXPECTED_GAUGE_CONTRACT_SHA256 = "d0cde7388bf0984ca664258e22cf00dc5f1702951811d5cb29c3282e57583328"
OLD_CANDIDATE_SHA256 = "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f"
DSPH_CONFIG_SHA256 = "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd"
REQUIRED_CAPABILITY = "MOTION_ATTACHED_16_RAW_JGAUGESWL"
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v10"
TRANSFER_STATUS = "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED"
EXPECTED_CSV = tuple(f"GaugesSwl_s5b0_p{index:02d}.csv" for index in range(16))
EVIDENCE_KEYS = {"authorization_status", "real_staging_authorized", "candidate", "transfer", "settled_clone", "result_qc"}
CANDIDATE_KEYS = {"capability", "sha256", "fresh_patched_build", "build_receipt_sha256", "static_audit_receipt_sha256", "built", "static_audit_passed", "old_candidate_reused"}
TRANSFER_KEYS = {"transfer_id", "status", "finalized", "manifest_sha256", "solver_path_sha256", "execution_receipt_sha256"}
SETTLED_KEYS = {"case_xml_sha256", "case_bi4_sha256", "restart_part_sha256", "restart_head_sha256", "dsph_config_sha256", "particle_count", "moving_boundary_count", "fluid_count", "nout", "ids_complete_unique", "finite", "leak_zero", "unique_motion_ref", "motion_ref", "mkbound", "refmotion"}
QC_KEYS = {"raw_gauge_csv_count", "raw_gauge_csv_names", "gauge_slots_complete", "executed_boundary_motion_required", "part_motion_ref_required", "particle_count_required", "ids_complete_unique_required", "nout_required", "finite_required", "leak_zero_required", "domain_outside_zero_required", "inputs_unchanged_required", "checksums_required"}


class StagingMaterializerV4Error(ValueError):
    """A frozen parent, candidate, XML, inventory, or QC invariant failed."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":")) + "\n").encode()


def canonical_sha256(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"), parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise StagingMaterializerV4Error(f"JSON root is not an object: {path}")
    return value


def assert_deep_closed(node: object, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise StagingMaterializerV4Error(f"open schema object: {location}")
        for key, value in node.items():
            assert_deep_closed(value, f"{location}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            assert_deep_closed(value, f"{location}/{index}")


def load_frozen_contract() -> tuple[dict[str, Any], dict[str, Any]]:
    policy_raw = S5B0_POLICY_PATH.read_bytes()
    schema_raw = S5B0_POLICY_SCHEMA_PATH.read_bytes()
    if sha256_bytes(policy_raw) != EXPECTED_POLICY_SHA256 or sha256_bytes(schema_raw) != EXPECTED_POLICY_SCHEMA_SHA256:
        raise StagingMaterializerV4Error("frozen S5B0 v4 policy/schema identity drift")
    policy = read_json(S5B0_POLICY_PATH)
    policy_schema = read_json(S5B0_POLICY_SCHEMA_PATH)
    Draft202012Validator.check_schema(policy_schema)
    assert_deep_closed(policy_schema)
    Draft202012Validator(policy_schema).validate(policy)
    gauge = policy["gauge_contract"]
    if canonical_sha256(gauge) != EXPECTED_GAUGE_CONTRACT_SHA256:
        raise StagingMaterializerV4Error("frozen S5B0 Gauge contract identity drift")
    if policy["selection"] != {"attempt_id":"SIM-S1_CORE_H1_C1_Bsmooth_b01_r01", "role":"PRIMARY_BASELINE", "container":"C1", "planned_denominator":1, "optional_authorized":False, "c2_authorized":False}:
        raise StagingMaterializerV4Error("primary replay selection drift")
    return policy, gauge


def _sha(value: object, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
        raise StagingMaterializerV4Error(f"invalid SHA-256: {label}")
    return value


def fixture_evidence(sources: Mapping[str, Mapping[str, Any]], policy: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "authorization_status": "FIXTURE_ONLY_NOT_REAL_AUTHORIZATION",
        "real_staging_authorized": False,
        "candidate": {"capability": REQUIRED_CAPABILITY, "sha256": sources["candidate"]["sha256"], "fresh_patched_build": True, "build_receipt_sha256": "1"*64, "static_audit_receipt_sha256": "2"*64, "built": True, "static_audit_passed": True, "old_candidate_reused": False},
        "transfer": {"transfer_id": TRANSFER_ID, "status": TRANSFER_STATUS, "finalized": True, "manifest_sha256": policy["finalized_transfer"]["manifest_sha256"], "solver_path_sha256": sources["solver_path"]["sha256"], "execution_receipt_sha256": policy["finalized_transfer"]["execution_receipt_sha256"]},
        "settled_clone": {"case_xml_sha256": sources["case_xml"]["sha256"], "case_bi4_sha256": sources["case_bi4"]["sha256"], "restart_part_sha256": sources["restart_part"]["sha256"], "restart_head_sha256": sources["restart_head"]["sha256"], "dsph_config_sha256": sources["dsph_config"]["sha256"], "particle_count":9078, "moving_boundary_count":2669, "fluid_count":6409, "nout":0, "ids_complete_unique":True, "finite":True, "leak_zero":True, "unique_motion_ref":True, "motion_ref":0, "mkbound":0, "refmotion":0},
        "result_qc": {"raw_gauge_csv_count":16, "raw_gauge_csv_names":list(EXPECTED_CSV), "gauge_slots_complete":True, "executed_boundary_motion_required":True, "part_motion_ref_required":True, "particle_count_required":9078, "ids_complete_unique_required":True, "nout_required":0, "finite_required":True, "leak_zero_required":True, "domain_outside_zero_required":True, "inputs_unchanged_required":True, "checksums_required":True},
    }


def validate_evidence(mode: str, evidence: Mapping[str, Any], sources: Mapping[str, Mapping[str, Any]], policy: Mapping[str, Any]) -> None:
    if mode not in {"FIXTURE", "REAL"} or set(evidence) != EVIDENCE_KEYS:
        raise StagingMaterializerV4Error("mode/evidence root is not closed")
    candidate, transfer, settled, qc = (evidence[name] for name in ("candidate", "transfer", "settled_clone", "result_qc"))
    if set(candidate) != CANDIDATE_KEYS or set(transfer) != TRANSFER_KEYS or set(settled) != SETTLED_KEYS or set(qc) != QC_KEYS:
        raise StagingMaterializerV4Error("evidence section is not closed")
    if mode == "REAL":
        if evidence["authorization_status"] != "EXACT_REAL_STAGING_AUTHORIZATION" or evidence["real_staging_authorized"] is not True:
            raise StagingMaterializerV4Error("REAL staging requires exact authorization")
    elif evidence["authorization_status"] != "FIXTURE_ONLY_NOT_REAL_AUTHORIZATION" or evidence["real_staging_authorized"] is not False:
        raise StagingMaterializerV4Error("fixture evidence attempted real authorization")
    candidate_sha = _sha(candidate["sha256"], "candidate")
    if candidate_sha != sources["candidate"]["sha256"] or candidate_sha == OLD_CANDIDATE_SHA256:
        raise StagingMaterializerV4Error("old or identity-mismatched candidate is forbidden")
    if candidate["capability"] != REQUIRED_CAPABILITY or not candidate["fresh_patched_build"] or not candidate["built"] or not candidate["static_audit_passed"] or candidate["old_candidate_reused"]:
        raise StagingMaterializerV4Error("fresh patched candidate capability is absent")
    _sha(candidate["build_receipt_sha256"], "build receipt")
    _sha(candidate["static_audit_receipt_sha256"], "static audit receipt")
    frozen_transfer = policy["finalized_transfer"]
    if transfer["transfer_id"] != TRANSFER_ID or transfer["status"] != TRANSFER_STATUS or transfer["finalized"] is not True:
        raise StagingMaterializerV4Error("finalized transfer v10 identity/status drift")
    if transfer["manifest_sha256"] != frozen_transfer["manifest_sha256"] or transfer["execution_receipt_sha256"] != frozen_transfer["execution_receipt_sha256"]:
        raise StagingMaterializerV4Error("finalized transfer v10 receipt drift")
    if transfer["solver_path_sha256"] != sources["solver_path"]["sha256"]:
        raise StagingMaterializerV4Error("solver_path is not bound to transfer evidence")
    if mode == "REAL" and transfer["solver_path_sha256"] != frozen_transfer["solver_path_sha256"]:
        raise StagingMaterializerV4Error("REAL solver_path differs from finalized transfer v10")
    expected_settled = {"particle_count":9078, "moving_boundary_count":2669, "fluid_count":6409, "nout":0, "ids_complete_unique":True, "finite":True, "leak_zero":True, "unique_motion_ref":True, "motion_ref":0, "mkbound":0, "refmotion":0}
    if any(settled[key] != value for key, value in expected_settled.items()):
        raise StagingMaterializerV4Error("settled clone particle/Nout/finite/leak/motion evidence drift")
    for key, role in (("case_xml_sha256","case_xml"), ("case_bi4_sha256","case_bi4"), ("restart_part_sha256","restart_part"), ("restart_head_sha256","restart_head"), ("dsph_config_sha256","dsph_config")):
        if settled[key] != sources[role]["sha256"]:
            raise StagingMaterializerV4Error(f"settled clone source identity drift: {role}")
    if mode == "REAL":
        parents = policy["frozen_parents"]
        expected_hashes = {"case_xml_sha256":parents["case_xml_sha256"], "case_bi4_sha256":parents["case_bi4_sha256"], "restart_part_sha256":parents["settled_part_sha256"], "restart_head_sha256":parents["settled_head_sha256"], "dsph_config_sha256":DSPH_CONFIG_SHA256}
        if any(settled[key] != value for key, value in expected_hashes.items()):
            raise StagingMaterializerV4Error("REAL settled/BI4/config parent drift")
    expected_qc = {"raw_gauge_csv_count":16, "raw_gauge_csv_names":list(EXPECTED_CSV), "gauge_slots_complete":True, "executed_boundary_motion_required":True, "part_motion_ref_required":True, "particle_count_required":9078, "ids_complete_unique_required":True, "nout_required":0, "finite_required":True, "leak_zero_required":True, "domain_outside_zero_required":True, "inputs_unchanged_required":True, "checksums_required":True}
    if dict(qc) != expected_qc:
        raise StagingMaterializerV4Error("runtime raw Gauge/boundary/particle QC contract drift")


def _float_text(value: object) -> str:
    result = float(value)
    if not math.isfinite(result):
        raise StagingMaterializerV4Error("non-finite Gauge coordinate")
    return format(result, ".17g")


def _add_gauges(execution: ET.Element, gauge: Mapping[str, Any], tmax_s: float) -> None:
    specials = execution.findall("special")
    if len(specials) > 1 or execution.findall("./special/gauges"):
        raise StagingMaterializerV4Error("base case already contains ambiguous/Gauge special configuration")
    special = specials[0] if specials else ET.SubElement(execution, "special")
    gauges = ET.SubElement(special, "gauges")
    for probe in gauge["probes"]:
        swl = ET.SubElement(gauges, "swl", {"name":probe["name"], "motionref":"0", "mkbound":"0"})
        ET.SubElement(swl, "savevtkpart", {"value":"false"})
        ET.SubElement(swl, "computedt", {"value":_float_text(gauge["compute_dt_s"])})
        ET.SubElement(swl, "computetime", {"start":"0", "end":_float_text(tmax_s)})
        ET.SubElement(swl, "output", {"value":"true"})
        ET.SubElement(swl, "outputdt", {"value":_float_text(gauge["output_dt_s"])})
        ET.SubElement(swl, "outputtime", {"start":"0", "end":_float_text(tmax_s)})
        ET.SubElement(swl, "masslimit", {"coef":_float_text(gauge["masslimit_coef"])})
        ET.SubElement(swl, "pointdp", {"value":_float_text(gauge["pointdp_m"])})
        ET.SubElement(swl, "point0", {"x":_float_text(probe["x_m"]), "y":_float_text(probe["y_m"]), "z":_float_text(gauge["point0_z_m"])})
        ET.SubElement(swl, "point2", {"x":_float_text(probe["x_m"]), "y":_float_text(probe["y_m"]), "z":_float_text(gauge["point2_z_m"])})


def validate_case_identity(root: ET.Element) -> None:
    particles = root.find("./execution/particles")
    if particles is None or particles.attrib.get("np") != "9078" or particles.attrib.get("nb") != "2669":
        raise StagingMaterializerV4Error("case particle identity is not 9078/2669")
    moving, fluid = particles.findall("moving"), particles.findall("fluid")
    if len(moving) != 1 or len(fluid) != 1:
        raise StagingMaterializerV4Error("case moving/fluid cardinality differs")
    if moving[0].attrib != {"mkbound":"0", "mk":"2", "begin":"0", "count":"2669", "refmotion":"0"}:
        raise StagingMaterializerV4Error("unique moving mkbound/refmotion identity differs")
    if fluid[0].attrib != {"mkfluid":"0", "mk":"1", "begin":"2669", "count":"6409"}:
        raise StagingMaterializerV4Error("fluid ID/count identity differs")
    motions = root.findall("./casedef/motion/objreal") + root.findall("./execution/motion/objreal")
    if len(motions) != 2 or any(item.attrib != {"ref":"0"} for item in motions):
        raise StagingMaterializerV4Error("motion ref=0 is not unique across case/execution")


def validate_gauges(root: ET.Element, gauge: Mapping[str, Any], tmax_s: float) -> None:
    nodes = root.findall("./execution/special/gauges")
    if len(nodes) != 1 or len(list(nodes[0])) != 16:
        raise StagingMaterializerV4Error("exactly one 16-item native Gauge list is required")
    expected_names = [f"s5b0_p{index:02d}" for index in range(16)]
    observed = list(nodes[0])
    if [item.tag for item in observed] != ["swl"]*16 or [item.attrib.get("name") for item in observed] != expected_names or len(set(expected_names)) != 16:
        raise StagingMaterializerV4Error("SWL type/name/order/cardinality drift")
    child_tags = ["savevtkpart","computedt","computetime","output","outputdt","outputtime","masslimit","pointdp","point0","point2"]
    for index, (node, probe) in enumerate(zip(observed, gauge["probes"])):
        if node.attrib != {"name":probe["name"], "motionref":"0", "mkbound":"0"}:
            raise StagingMaterializerV4Error(f"world-fixed or wrong attachment at probe {index}")
        children = list(node)
        if [child.tag for child in children] != child_tags:
            raise StagingMaterializerV4Error(f"SWL child contract drift at probe {index}")
        expected = [
            {"value":"false"}, {"value":_float_text(gauge["compute_dt_s"])}, {"start":"0","end":_float_text(tmax_s)},
            {"value":"true"}, {"value":_float_text(gauge["output_dt_s"])}, {"start":"0","end":_float_text(tmax_s)},
            {"coef":_float_text(gauge["masslimit_coef"])}, {"value":_float_text(gauge["pointdp_m"])},
            {"x":_float_text(probe["x_m"]),"y":_float_text(probe["y_m"]),"z":_float_text(gauge["point0_z_m"])},
            {"x":_float_text(probe["x_m"]),"y":_float_text(probe["y_m"]),"z":_float_text(gauge["point2_z_m"])},
        ]
        if [child.attrib for child in children] != expected:
            raise StagingMaterializerV4Error(f"SWL geometry/timing/output drift at probe {index}")


def render_case_v4(source: bytes, *, settled_time_s: float, duration_s: float, gauge: Mapping[str, Any]) -> bytes:
    motion_rendered = v3.render_case(source, settled_time_s=settled_time_s, duration_s=duration_s)
    root = ET.fromstring(motion_rendered)
    validate_case_identity(root)
    execution = root.find("./execution")
    if execution is None:
        raise StagingMaterializerV4Error("execution node is absent")
    tmax = settled_time_s + duration_s
    _add_gauges(execution, gauge, tmax)
    ET.indent(root, space="    ")
    rendered = ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
    root2 = ET.fromstring(rendered)
    validate_case_identity(root2)
    validate_gauges(root2, gauge, tmax)
    v3.validate_rendered_case(rendered, settled_time_s=settled_time_s, duration_s=duration_s)
    return rendered


def materialize(stage_root: Path, *, expected_stage_root: Path, sources: Mapping[str, Mapping[str, Any]], restart_part_index: int, settled_time_s: float, solver_tail_s: float, mode: str, evidence: Mapping[str, Any]) -> dict[str, Any]:
    policy, gauge = load_frozen_contract()
    validate_evidence(mode, evidence, sources, policy)
    stage_root = v3._exact_absolute(Path(stage_root), "stage_root")
    expected_stage_root = v3._exact_absolute(Path(expected_stage_root), "expected_stage_root")
    if stage_root != expected_stage_root or not stage_root.name.endswith(".partial") or os.path.lexists(stage_root):
        raise StagingMaterializerV4Error("stage root is not the exact fresh .partial root")
    if restart_part_index != 901 or abs(float(settled_time_s)-45.05001991890928) > 1e-12 or abs(float(solver_tail_s)-1.0) > 1e-12:
        raise StagingMaterializerV4Error("settled restart identity/timing drift")
    payloads, descriptors, source_metadata = v3._open_exact_sources(sources)
    try:
        rows = v3._solver_rows(payloads["solver_path"], solver_tail_s)
        rendered_case = render_case_v4(payloads["case_xml"], settled_time_s=settled_time_s, duration_s=rows[-1].t_s, gauge=gauge)
        os.mkdir(stage_root, 0o700)
        root_fd = os.open(stage_root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            for directory in ("runtime", "case", "restart"):
                os.mkdir(directory, 0o700, dir_fd=root_fd)
            outputs = dict(payloads)
            outputs["case_xml"] = rendered_case
            staged = {role:v3._write_at(root_fd, v3.DESTINATIONS[role][0], outputs[role], v3.DESTINATIONS[role][1]) for role in v3.ROLES}
            os.fsync(root_fd)
        finally:
            os.close(root_fd)
        v3._postverify_sources(sources, descriptors, source_metadata)
        v3._verify_staged(stage_root, staged)
        staged_root = ET.fromstring((stage_root / "case/C1M_case.xml").read_bytes())
        validate_case_identity(staged_root)
        validate_gauges(staged_root, gauge, settled_time_s + rows[-1].t_s)
        contract = {
            "restart_part_index":901, "settled_time_s":45.05001991890928, "solver_path_row_count":len(rows), "solver_path_last_t_s":rows[-1].t_s, "solver_tail_s":1.0, "tmax_s":settled_time_s+rows[-1].t_s,
            "motion_block_count":2, "motion_ref":0, "mkbound":0, "refmotion":0,
            "gauge_type":"JGaugeSwl", "probe_count":16, "probe_names":[item["name"] for item in gauge["probes"]], "motionref":0, "gauge_mkbound":0,
            "probe_radius_m":gauge["probe_radius_m"], "point0_z_m":gauge["point0_z_m"], "point2_z_m":gauge["point2_z_m"], "pointdp_m":gauge["pointdp_m"], "masslimit_coef":gauge["masslimit_coef"], "compute_dt_s":gauge["compute_dt_s"], "output_dt_s":gauge["output_dt_s"], "savevtkpart":False, "output":True,
            "expected_raw_gauge_csv":list(EXPECTED_CSV), "required_result_artifacts":["executed_boundary_motion.csv","gauge_zsurf.csv","qc_report.json","finalized_solver_frames_manifest.json","checksums.sha256"],
        }
        semantic = {"mode":mode, "contract":contract, "candidate_sha256":evidence["candidate"]["sha256"], "transfer_manifest_sha256":evidence["transfer"]["manifest_sha256"], "sources":{role:{"sha256":sources[role]["sha256"],"size_bytes":sources[role]["size_bytes"]} for role in v3.ROLES}, "staged":{role:{key:staged[role][key] for key in ("path","mode","size_bytes","sha256")} for role in v3.ROLES}}
        manifest = {
            "schema_version":"smpcc-r8-liquid-s5b0-staging-case-manifest-v4", "document_type":"SMPCC_R8_LIQUID_S5B0_STAGING_CASE_MANIFEST_V4", "status":"PASS_S5B0_STAGING_CASE_WITH_16_NATIVE_ATTACHED_SWL_V4", "mode":mode, "stage_root":str(stage_root),
            "bindings":{"s5b0_policy_sha256":EXPECTED_POLICY_SHA256,"s5b0_policy_schema_sha256":EXPECTED_POLICY_SCHEMA_SHA256,"gauge_contract_sha256":EXPECTED_GAUGE_CONTRACT_SHA256,"required_candidate_capability":REQUIRED_CAPABILITY},
            "evidence":dict(evidence), "contract":contract, "sources":{role:dict(sources[role]) for role in v3.ROLES}, "staged":staged,
            "integrity":{"fresh_root":True,"o_excl_writes":True,"exact_inventory":True,"no_symlinks":True,"no_hardlinks":True,"no_special_files":True,"no_toctou":True,"source_identities_unchanged":True,"candidate_is_fresh_patched_not_old":True,"two_motion_blocks_exact":True,"one_gauge_list_exact":True,"sixteen_swl_exact":True,"semantic_manifest_sha256":canonical_sha256(semantic)},
            "claims":{"real_bag_read":False,"optional_bag_read":False,"candidate_executed":False,"solver_executed":False,"gpu_exposed":False,"network_used":False,"sudo_used":False,"apparmor_loaded":False,"raw_gauge_csv_read":False,"real_solver_output_read":False,"replay_authorized":mode=="REAL"},
        }
        schema = read_json(SCHEMA_PATH)
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
        Draft202012Validator(schema).validate(manifest)
        return manifest
    finally:
        for descriptor in descriptors.values():
            os.close(descriptor)


def _fixture_case_xml() -> bytes:
    casedef_motion = """        <motion>
            <objreal ref="0"><begin mov="1" start="0" /><mvnull id="1" /></objreal>
        </motion>"""
    execution_motion = """        <motion>
            <objreal ref="0"><begin mov="1" start="0" /><mvnull id="1" /></objreal>
        </motion>"""
    particles = """        <particles np="9078" nb="2669" nbf="0" mkboundfirst="2" mkfluidfirst="1">
            <moving mkbound="0" mk="2" begin="0" count="2669" refmotion="0" />
            <fluid mkfluid="0" mk="1" begin="2669" count="6409" />
        </particles>"""
    return f"""<?xml version="1.0"?>
<case>
    <casedef>
{casedef_motion}
    </casedef>
    <execution>
        <parameters />
{particles}
{execution_motion}
    </execution>
</case>
""".encode()


def _write_fixture_sources(root: Path) -> dict[str, dict[str, Any]]:
    sources = v3._write_fixture_sources(root)
    case = Path(sources["case_xml"]["path"])
    case.chmod(0o600)
    case.write_bytes(_fixture_case_xml())
    case.chmod(0o440)
    return {role:v3.observe_source(Path(sources[role]["path"])) for role in v3.ROLES}


def self_check() -> dict[str, Any]:
    schema = read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    policy, _ = load_frozen_contract()
    removed = False
    with tempfile.TemporaryDirectory(prefix="r8-s5b0-staging-v4-fixture-") as temporary:
        base = Path(temporary)
        sources = _write_fixture_sources(base / "sources")
        stage = base / "stage.partial"
        manifest = materialize(stage, expected_stage_root=stage, sources=sources, restart_part_index=901, settled_time_s=45.05001991890928, solver_tail_s=1.0, mode="FIXTURE", evidence=fixture_evidence(sources, policy))
        case = ET.fromstring((stage / "case/C1M_case.xml").read_bytes())
        validate_gauges(case, policy["gauge_contract"], manifest["contract"]["tmax_s"])
    removed = not Path(temporary).exists()
    return {"status":"PASS_S5B0_STAGING_CASE_MATERIALIZER_V4_FIXTURE_SELF_CHECK", "manifest_status":manifest["status"], "probe_count":manifest["contract"]["probe_count"], "raw_gauge_csv_count":len(manifest["contract"]["expected_raw_gauge_csv"]), "fixture_root_removed":removed, "schemas_deep_closed":True, "real_external_input_read":False, "real_bag_read":False, "optional_bag_read":False, "candidate_executed":False, "solver_executed":False, "gpu_exposed":False, "sudo_used":False, "network_used":False, "apparmor_loaded":False, "real_staging_authorized":False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), sort_keys=True, separators=(",", ":")))
    except Exception as exc:
        print(json.dumps({"status":"FAIL_S5B0_STAGING_CASE_MATERIALIZER_V4","error":str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
