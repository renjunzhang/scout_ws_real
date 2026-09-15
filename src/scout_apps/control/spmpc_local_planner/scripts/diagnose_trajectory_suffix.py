#!/usr/bin/env python3
"""Compare remaining suffixes offline from a provided full-state prediction."""
import argparse
import json
from pathlib import Path
from planning.suffix import compare_suffixes


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan",type=Path)
    parser.add_argument("checkpoint",type=Path,help="JSON: {task_elapsed_sec, state:[28]}")
    parser.add_argument("output",type=Path)
    parser.add_argument("--nominal-only",action="store_true")
    args=parser.parse_args()
    plan=json.loads(args.plan.read_text()); checkpoint=json.loads(args.checkpoint.read_text())
    if checkpoint.get("plan_id") not in (None,"",plan["plan_id"]):
        parser.error("checkpoint plan identity differs from the supplied task plan")
    result=compare_suffixes(plan, checkpoint["state"], checkpoint["task_elapsed_sec"], not args.nominal_only)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"candidates":len(result["candidates"]),"best":result["best_feasible_candidate"]}))

if __name__=="__main__": main()
