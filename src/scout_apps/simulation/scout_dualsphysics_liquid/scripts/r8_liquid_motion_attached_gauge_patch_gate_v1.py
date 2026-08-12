#!/usr/bin/env python3
"""Deterministic, static-only source patch contract for motion-attached SWL gauges.

This module never compiles or runs DualSPHysics.  It renders a unified patch in
memory only after every pinned upstream byte sequence has been authenticated.
Applying the rendered patch is deliberately outside this revision: a future
fresh-campaign execution gate must first prove an isolated fresh source copy.
"""

from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, Mapping


ROOT = Path(__file__).resolve().parents[1]
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_attached_gauge_patch_policy_v1.json"
SCHEMA_PATH = ROOT / "schema/target_host_motion_attached_gauge_patch_policy_v1.json"
FIXTURE_PATH = ROOT / "tests/fixtures/motion_attached_gauge_patch_fixture_v1.json"
PATCH_ARTIFACT_PATH = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_v1.diff"
UPSTREAM_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
BARE_REPOSITORY = Path("/home/zrj/scout_liquid_lab/dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git")
SOURCE_PREFIX = "src/source"

BEFORE_SHA256 = {
    "JDsGaugeItem.cpp": "909d0accec54daf2b767d7ab99338ff92ff15b49e9a0bf4c221d7dc6edecb0a4",
    "JDsGaugeItem.h": "4ceee0be27e86b0686d5530574ab79bd8a99a4f64596b474c9234b8da6df2823",
    "JDsGaugeSystem.cpp": "1ccef263531c3757d513284c7187a4e14fec43c88fc1ff4b5ba4092cca2e1e80",
    "JDsGaugeSystem.h": "473831ed3614277ca7e82f2b2ce065a0c37c6039ecbaf9cfcaddef624a2ff86d",
    "JSph.cpp": "206e4486a3a0d304e02da1a73d2e7c7ed96354d559ce481275d09f5245b8c9e7",
    "JSph.h": "abb087c020641d94c3968507f0ce067da4f45cac246ff6bc586d4e8665e39eda",
}


class PatchContractError(RuntimeError):
    """Fail-closed contract violation."""


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


# Each tuple is an exact, one-occurrence byte replacement.  This representation
# makes the patch deterministic and rejects drift before producing any output.
REPLACEMENTS: Mapping[str, tuple[tuple[str, str], ...]] = {
    "JDsGaugeItem.h": (
        (
            "#include \"JMeshDataDef.h\" //<vs_meeshdat>\n",
            "#include \"JMeshDataDef.h\" //<vs_meeshdat>\n#include \"JDsMotionDef.h\"\n",
        ),
        (
            "  float MassLimit; \n  //-Auxiliary variables.\n",
            "  float MassLimit;\n  //-Optional attachment to one predefined moving boundary.\n  bool MotionAttached;\n  unsigned MotionRef;\n  word MotionMkBound;\n  tdouble3 MotionBaseCenter;\n  double MotionRayHalfLength;\n  //-Auxiliary variables.\n",
        ),
        (
            "  const StGaugeSwlRes& GetResult()const{ return(Result); }\n\n  void SetPoints(const tdouble3& point0,const tdouble3& point2,double pointdp=0);\n",
            "  const StGaugeSwlRes& GetResult()const{ return(Result); }\n\n  bool GetMotionAttached()const{ return(MotionAttached); }\n  unsigned GetMotionRef()const{ return(MotionRef); }\n  word GetMotionMkBound()const{ return(MotionMkBound); }\n  void ConfigMotionAttached(unsigned motionref,word mkbound);\n  void ApplyMotionAttached(const StMotionData& motion,bool absolute);\n  void SetPoints(const tdouble3& point0,const tdouble3& point2,double pointdp=0);\n",
        ),
    ),
    "JDsGaugeItem.cpp": (
        (
            "  PointDp=0;\n  SetPoints(TDouble3(0),TDouble3(0),0);\n  MassLimit=0;\n",
            "  PointDp=0;\n  SetPoints(TDouble3(0),TDouble3(0),0);\n  MassLimit=0;\n  MotionAttached=false;\n  MotionRef=UINT_MAX;\n  MotionMkBound=USHRT_MAX;\n  MotionBaseCenter=TDouble3(0);\n  MotionRayHalfLength=0;\n",
        ),
        (
            "//==============================================================================\n/// Changes points definition.\n//==============================================================================\nvoid JGaugeSwl::SetPoints",
            "//==============================================================================\n/// Binds this SWL ray to one exact predefined moving boundary.\n+//==============================================================================\nvoid JGaugeSwl::ConfigMotionAttached(unsigned motionref,word mkbound){\n  if(MotionAttached)Run_Exceptioon(\"Motion attachment is already configured.\");\n  if(Point0.x!=Point2.x || Point0.y!=Point2.y || Point2.z<=Point0.z)\n    Run_Exceptioon(\"A motion-attached SWL gauge requires a global-Z ray.\");\n  MotionAttached=true;\n  MotionRef=motionref;\n  MotionMkBound=mkbound;\n  MotionBaseCenter=(Point0+Point2)/2.;\n  MotionRayHalfLength=(Point2.z-Point0.z)/2.;\n  ConfigDomMCel(false);\n}\n\n//==============================================================================\n/// Consumes already-computed motion data without advancing the motion clock.\n//==============================================================================\nvoid JGaugeSwl::ApplyMotionAttached(const StMotionData& motion,bool absolute){\n  if(!MotionAttached)return;\n  if(motion.ref!=MotionRef || motion.mkbound!=MotionMkBound)\n    Run_Exceptioon(\"Motion reference and MkBound do not match the gauge attachment.\");\n  tdouble3 center=(absolute? MotionBaseCenter: (Point0+Point2)/2.);\n  if(motion.type==MOTT_Linear)center=center+motion.linmov;\n  else if(motion.type==MOTT_Matrix)center=MatrixMulPoint(motion.matmov,center);\n  else if(absolute)center=MotionBaseCenter;\n  // The attached horizontal probe follows the body, while the measurement ray\n  // remains parallel to the global Z axis by construction.\n  const tdouble3 point0=TDouble3(center.x,center.y,center.z-MotionRayHalfLength);\n  const tdouble3 point2=TDouble3(center.x,center.y,center.z+MotionRayHalfLength);\n  SetPoints(point0,point2,PointDp);\n}\n\n//==============================================================================\n/// Changes points definition.\n//==============================================================================\nvoid JGaugeSwl::SetPoints",
        ),
    ),
    "JDsGaugeSystem.h": (
        (
            "class JSphMk;\n",
            "class JSphMk;\nclass JDsMotion;\n",
        ),
        (
            "  JGaugeItem* GetGauge(unsigned c)const;\n\n  void CalculeCpu",
            "  JGaugeItem* GetGauge(unsigned c)const;\n  void UpdateMotionAttachedGauges(const JDsMotion* motion,bool absolute);\n\n  void CalculeCpu",
        ),
    ),
    "JDsGaugeSystem.cpp": (
        (
            "#include \"JSphMk.h\"\n",
            "#include \"JSphMk.h\"\n#include \"JDsMotion.h\"\n",
        ),
        (
            "          gau=AddGaugeSwl(name,cfg.computestart,cfg.computeend,cfg.computedt\n            ,true,pt0,pt2,pointdp,masslimit);\n",
            "          gau=AddGaugeSwl(name,cfg.computestart,cfg.computeend,cfg.computedt\n            ,true,pt0,pt2,pointdp,masslimit);\n          const unsigned motionref=sxml->GetAttributeUnsigned(ele,\"motionref\",true,UINT_MAX);\n          const unsigned mkbound=sxml->GetAttributeUnsigned(ele,\"mkbound\",true,UINT_MAX);\n          if((motionref==UINT_MAX)!=(mkbound==UINT_MAX))\n            Run_ExceptioonFile(\"Attributes motionref and mkbound must be specified together.\",sxml->ErrGetFileRow(ele));\n          if(motionref!=UINT_MAX){\n            if(motionref>USHRT_MAX || mkbound>USHRT_MAX)\n              Run_ExceptioonFile(\"Motion attachment identity is outside the word range.\",sxml->ErrGetFileRow(ele));\n            ((JGaugeSwl*)gau)->ConfigMotionAttached(motionref,word(mkbound));\n          }\n",
        ),
        (
            "  SaveVtkInitPoints();\n}\n\n//==============================================================================\n/// Creates new gauge-Velocity",
            "  unsigned attachedswl=0;\n  for(unsigned c=0;c<GetCount();c++)if(Gauges[c]->Type==JGaugeItem::GAUGE_Swl\n    && ((JGaugeSwl*)Gauges[c])->GetMotionAttached())attachedswl++;\n  if(attachedswl && attachedswl!=16)\n    Run_Exceptioon(\"Exactly 16 motion-attached JGaugeSwl probes are required.\");\n  SaveVtkInitPoints();\n}\n\n//==============================================================================\n/// Updates attached SWL rays from data already produced by JDsMotion::ProcesTime.\n//==============================================================================\nvoid JGaugeSystem::UpdateMotionAttachedGauges(const JDsMotion* motion,bool absolute){\n  if(!motion)Run_Exceptioon(\"Motion-attached gauges require JDsMotion.\");\n  for(unsigned c=0;c<GetCount();c++)if(Gauges[c]->Type==JGaugeItem::GAUGE_Swl){\n    JGaugeSwl* swl=(JGaugeSwl*)Gauges[c];\n    if(swl->GetMotionAttached()){\n      const unsigned ref=swl->GetMotionRef();\n      if(ref>=motion->GetNumObjects()\n        || motion->GetObjIdxByMkBound(swl->GetMotionMkBound())!=ref)\n        Run_Exceptioon(\"Motion attachment does not match the configured moving boundary.\");\n      swl->ApplyMotionAttached(motion->GetMotionData(ref),absolute);\n    }\n  }\n}\n\n//==============================================================================\n/// Creates new gauge-Velocity",
        ),
    ),
    "JSph.h": (
        (
            "  bool CalcMotion(double stepdt);\n  void CalcMotionWaveGen(double stepdt);\n",
            "  bool CalcMotion(double stepdt);\n  void SyncMotionAttachedGauges(bool absolute);\n  void CalcMotionWaveGen(double stepdt);\n",
        ),
    ),
    "JSph.cpp": (
        (
            "  if(xml.GetNodeSimple(\"case.execution.special.gauges\",true))\n    GaugeSystem->LoadXml(&xml,\"case.execution.special.gauges\",MkInfo);\n\n  //-Prepares WaveGen configuration.",
            "  if(xml.GetNodeSimple(\"case.execution.special.gauges\",true)){\n    GaugeSystem->LoadXml(&xml,\"case.execution.special.gauges\",MkInfo);\n    // DsMotion was advanced exactly once above.  On restart its current data is\n    // an absolute TimeStepIni/PartBegin pose; consume it without ProcesTime.\n    SyncMotionAttachedGauges(true);\n  }\n\n  //-Prepares WaveGen configuration.",
        ),
        (
            "  DsMotion->ProcesTime(mode,TimeStep,stepdt);\n  const bool active=DsMotion->GetActiveMotion();\n",
            "  DsMotion->ProcesTime(mode,TimeStep,stepdt);\n  // This hook consumes the just-computed increment and never calls ProcesTime.\n  SyncMotionAttachedGauges(false);\n  const bool active=DsMotion->GetActiveMotion();\n",
        ),
        (
            "//==============================================================================\n/// Add motion from automatic wave generation.\n",
            "//==============================================================================\n/// Synchronises motion-attached gauges from the current JDsMotion state.\n+//==============================================================================\nvoid JSph::SyncMotionAttachedGauges(bool absolute){\n  if(GaugeSystem && GaugeSystem->GetCount())\n    GaugeSystem->UpdateMotionAttachedGauges(DsMotion,absolute);\n}\n\n//==============================================================================\n/// Add motion from automatic wave generation.\n",
        ),
    ),
}


def load_upstream() -> Dict[str, bytes]:
    if not BARE_REPOSITORY.is_dir():
        raise PatchContractError("pinned bare repository is absent")
    result: Dict[str, bytes] = {}
    for name, expected in sorted(BEFORE_SHA256.items()):
        argv = ["/usr/bin/git", f"--git-dir={BARE_REPOSITORY}", "show", f"{UPSTREAM_COMMIT}:{SOURCE_PREFIX}/{name}"]
        proc = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        if proc.returncode:
            raise PatchContractError(f"git show failed for {name}: rc={proc.returncode}")
        if _sha(proc.stdout) != expected:
            raise PatchContractError(f"upstream byte identity drift for {name}")
        result[name] = proc.stdout
    return result


def apply_in_memory(source: Mapping[str, bytes]) -> Dict[str, bytes]:
    if set(source) != set(BEFORE_SHA256):
        raise PatchContractError("source inventory is not the exact six-file set")
    result: Dict[str, bytes] = {}
    for name in sorted(source):
        data = source[name]
        if _sha(data) != BEFORE_SHA256[name]:
            raise PatchContractError(f"before hash mismatch for {name}")
        text = data.decode("utf-8")
        for before, after in REPLACEMENTS[name]:
            if text.count(before) != 1:
                raise PatchContractError(f"non-unique replacement anchor in {name}")
            text = text.replace(before, after, 1)
        # Strip the two accidental diff-authoring sentinels from generated C++.
        # The exact expected count keeps this correction fail-closed.
        sentinel = "\n+//=============================================================================="
        expected_sentinels = 1 if name in {"JDsGaugeItem.cpp", "JSph.cpp"} else 0
        if text.count(sentinel) != expected_sentinels:
            raise PatchContractError(f"generated-source sentinel count drift in {name}")
        text = text.replace(sentinel, "\n//==============================================================================")
        result[name] = text.encode("utf-8")
    return result


def render_unified(before: Mapping[str, bytes], after: Mapping[str, bytes]) -> bytes:
    lines: list[str] = []
    for name in sorted(before):
        lines.extend(difflib.unified_diff(
            before[name].decode().splitlines(keepends=True),
            after[name].decode().splitlines(keepends=True),
            fromfile=f"a/{SOURCE_PREFIX}/{name}",
            tofile=f"b/{SOURCE_PREFIX}/{name}",
            n=3,
        ))
    return "".join(lines).encode("utf-8")


def aggregate_source_hash(files: Mapping[str, bytes]) -> str:
    canonical = b"".join(name.encode() + b"\0" + _sha(files[name]).encode() + b"\n" for name in sorted(files))
    return _sha(canonical)


def validate_fixture(value: object) -> None:
    if not isinstance(value, dict) or set(value) != {"schema_version", "motionref", "mkbound", "ray_axis", "probes"}:
        raise PatchContractError("fixture is not closed")
    if value["schema_version"] != "smpcc-r8-liquid-motion-attached-gauge-fixture-v1":
        raise PatchContractError("fixture schema version drift")
    if value["motionref"] != 0 or value["mkbound"] != 0 or value["ray_axis"] != "GLOBAL_Z":
        raise PatchContractError("fixture attachment identity drift")
    probes = value["probes"]
    if not isinstance(probes, list) or len(probes) != 16:
        raise PatchContractError("fixture must contain exactly 16 probes")
    if [p.get("probe_id") for p in probes] != [f"swl_{i:02d}" for i in range(16)]:
        raise PatchContractError("probe identity/order drift")
    for p in probes:
        if set(p) != {"probe_id", "point0_m", "point2_m", "pointdp_m", "motionref", "mkbound"}:
            raise PatchContractError("probe entry is not closed")
        p0, p2 = p["point0_m"], p["point2_m"]
        if p["motionref"] != 0 or p["mkbound"] != 0 or len(p0) != 3 or len(p2) != 3:
            raise PatchContractError("probe binding drift")
        if p0[:2] != p2[:2] or not p2[2] > p0[2] or p["pointdp_m"] != 0.002:
            raise PatchContractError("probe is not an exact global-Z ray")


def audit(policy: Mapping[str, object], fixture: object) -> dict:
    source = load_upstream()
    patched = apply_in_memory(source)
    patch = render_unified(source, patched)
    validate_fixture(fixture)
    actual_files = {
        name: {"before_sha256": _sha(source[name]), "after_sha256": _sha(patched[name])}
        for name in sorted(source)
    }
    expected_files = {entry["path"].split("/")[-1]: {"before_sha256": entry["before_sha256"], "after_sha256": entry["after_sha256"]} for entry in policy["source_files"]}
    if actual_files != expected_files:
        raise PatchContractError("policy source hashes do not match rendered bytes")
    if policy["patch_identity"]["unified_patch_sha256"] != _sha(patch):
        raise PatchContractError("unified patch hash drift")
    if policy["patch_identity"]["patched_six_file_aggregate_sha256"] != aggregate_source_hash(patched):
        raise PatchContractError("patched aggregate hash drift")
    before_calls = source["JSph.cpp"].count(b"DsMotion->ProcesTime(")
    after_calls = patched["JSph.cpp"].count(b"DsMotion->ProcesTime(")
    if before_calls != after_calls:
        raise PatchContractError("patch adds or removes a ProcesTime call")
    jsph = patched["JSph.cpp"].decode()
    calc = jsph.index("DsMotion->ProcesTime(mode,TimeStep,stepdt);")
    hook = jsph.index("SyncMotionAttachedGauges(false);", calc)
    active = jsph.index("DsMotion->GetActiveMotion();", hook)
    if not calc < hook < active:
        raise PatchContractError("CalcMotion hook ordering drift")
    required = ("ConfigDomMCel(false)", "Exactly 16 motion-attached JGaugeSwl", "motionref", "mkbound", "GLOBAL_Z")
    corpus = b"\n".join(patched.values()).decode() + json.dumps(fixture, sort_keys=True)
    if any(token not in corpus for token in required):
        raise PatchContractError("required semantic marker is absent")
    return {
        "status": "PASS_MOTION_ATTACHED_GAUGE_PATCH_V1_STATIC_CONTRACT",
        "source_commit": UPSTREAM_COMMIT,
        "source_files": actual_files,
        "unified_patch_sha256": _sha(patch),
        "patched_six_file_aggregate_sha256": aggregate_source_hash(patched),
        "probe_count": 16,
        "motionref": 0,
        "mkbound": 0,
        "ray_axis": "GLOBAL_Z",
        "proces_time_call_count_before": before_calls,
        "proces_time_call_count_after": after_calls,
        "compiler_run": False,
        "solver_run": False,
        "gpu_exposed": False,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("self-check", "render-unified"))
    args = parser.parse_args(argv)
    policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
    fixture = json.loads(FIXTURE_PATH.read_text(encoding="utf-8"))
    report = audit(policy, fixture)
    if args.command == "render-unified":
        source = load_upstream()
        sys.stdout.buffer.write(render_unified(source, apply_in_memory(source)))
    else:
        print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PatchContractError, OSError, ValueError, KeyError, TypeError) as exc:
        print(f"FAIL_MOTION_ATTACHED_GAUGE_PATCH_V1: {exc}", file=sys.stderr)
        raise SystemExit(1)
