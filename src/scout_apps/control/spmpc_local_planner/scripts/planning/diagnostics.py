"""Append-only progress events for long offline optimizations.

This diagnostic stream is separate from the executable plan: a killed solver
leaves its last stage on disk without producing a seemingly valid trajectory.
"""
import json
import time
from pathlib import Path


class ProgressLog:
    def __init__(self, path):
        self.path = Path(path)
        self.started = time.monotonic()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Each attempt needs a new file so evidence is never silently replaced.
        self.stream = self.path.open("x")

    def __call__(self, stage, **details):
        event = dict(stage=stage, wall_seconds=time.monotonic() - self.started, **details)
        self.stream.write(json.dumps(event, allow_nan=False) + "\n")
        self.stream.flush()

    def close(self):
        self.stream.close()


def solver_summary(stats):
    """Keep JSON-safe scalar diagnostics, including the last NLP residuals."""
    result = {key: stats[key] for key in ("return_status", "success", "iter_count") if key in stats}
    iterations = stats.get("iterations", {})
    for key in ("inf_pr", "inf_du", "mu", "obj"):
        values = iterations.get(key, [])
        if values:
            value = float(values[-1])
            if value == value and abs(value) != float("inf"):
                result["last_" + key] = value
    return result
