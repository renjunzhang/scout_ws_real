"""Static checks for the U3 C1 non-executing GenCase seed gate."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts/r8_liquid_target_u3_gencase_seed_gate_v1.py"
POLICY = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_seed_materialization_policy_v1.json"
SCHEMA = PACKAGE / "schema/target_host_u3_gencase_seed_materialization_policy_v1.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_gencase_seed_gate_v1_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_review_artifacts_and_xml_contract_are_static_passes() -> None:
    gate = load_gate()
    review = gate.verify_review_artifacts()
    assert review["template"]["sha256"] == gate.CASE_TEMPLATE_SHA256
    assert review["full_fetch_receipt"]["sha256"] == gate.FULL_FETCH_RECEIPT_SHA256
    assert [entry["destination"] for entry in gate.SEED_FILES] == [
        "GenCase_linux64",
        "DsphConfig.xml",
        "C1_static_Def.xml",
    ]


def test_policy_schema_is_closed_at_the_top_level_and_matches_gate_surface() -> None:
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    assert schema["additionalProperties"] is False
    assert set(policy) == set(schema["required"])
    assert policy["allowed_gate_commands"] == ["self-check", "preflight", "materialize"]
    assert policy["source_provenance"]["gencase"]["host_execution"] == "forbidden"
    assert policy["invariants"]["no_precompiled_elf_execution"] is True


def test_gate_has_no_shell_or_upstream_executable_surface() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "shell=True" not in source
    assert "os.system" not in source
    assert "GenCase_linux64" in source
    assert "subprocess.run(" in source
    assert '"cat-file"' in source
    assert "--no-replace-objects" in source
    assert "precompiled_binary_executed\": False" in source
