#!/usr/bin/env python3
"""Validate one published D4 artifact without importing Acados or CasADi."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from mainline.artifact_publication import load_artifact_contract_directory


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-directory", type=Path, required=True)
    arguments = parser.parse_args()
    contract = load_artifact_contract_directory(arguments.artifact_directory)
    print(
        json.dumps(
            {
                "artifact_sha256": contract.artifact_sha256,
                "model_contract_semantic_sha256": contract.semantic_sha256,
                "status": "VALID_D4_ARTIFACT",
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
