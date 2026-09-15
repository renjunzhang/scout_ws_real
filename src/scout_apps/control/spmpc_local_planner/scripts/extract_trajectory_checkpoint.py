#!/usr/bin/env python3
"""Extract a suffix checkpoint from one saved PredictedHorizon JSON/YAML message."""
import argparse
import json
from pathlib import Path
import yaml
from planning.checkpoint import from_horizon


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("horizon", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--stage", type=int)
    args = parser.parse_args()
    records = [r for r in yaml.safe_load_all(args.horizon.read_text()) if r is not None]
    if len(records) != 1:
        parser.error("input must contain exactly one horizon message")
    checkpoint = from_horizon(records[0], args.stage)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(checkpoint, indent=2, allow_nan=False)+"\n")


if __name__ == "__main__":
    main()
