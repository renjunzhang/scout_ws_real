"""Extract an exact suffix initial state from a schema-8/9 horizon dictionary."""
import math


def from_horizon(record, stage=None):
    if (record.get("schema_version"), record.get("cost_model_version"), record.get("liquid_model_version")) not in ((8, 3, 1), (9, 3, 1)):
        raise ValueError("checkpoint requires horizon schema 8/9, cost 3, liquid model 1")
    if record.get("valid") is not True or record.get("model_state_width") != 28:
        raise ValueError("checkpoint requires a valid complete 28-state liquid prediction; never invent missing liquid/FIFO state")
    count = record["horizon_steps"]
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("invalid horizon length")
    stage = count if stage is None else stage
    if isinstance(stage, bool) or not isinstance(stage, int) or not 0 <= stage <= count:
        raise ValueError("checkpoint stage outside horizon")
    states = record["model_states"]
    if len(states) != 28*(count+1) or not all(math.isfinite(float(v)) for v in states):
        raise ValueError("incomplete/nonfinite full-state horizon")
    dt, elapsed = float(record["dt"]), float(record["task_elapsed_sec"])
    if not math.isfinite(dt) or dt <= 0 or not math.isfinite(elapsed) or elapsed < 0:
        raise ValueError("invalid checkpoint clock")
    return {"source": "exact solver predicted state", "cycle_id": record.get("cycle_id"),
            "plan_id": record.get("plan_id"), "stage": stage,
            "task_elapsed_sec": elapsed+stage*dt,
            "state": [float(v) for v in states[28*stage:28*(stage+1)]]}
