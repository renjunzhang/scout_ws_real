"""A fixed simulation-time recording horizon, independent of goal status."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObservationWindow:
    start_sec: float
    task_deadline_sec: float
    stop_window_sec: float
    margin_sec: float = 1.

    def __post_init__(self):
        if not math.isfinite(self.start_sec) or self.start_sec < 0:
            raise ValueError('invalid simulation start')
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.task_deadline_sec, self.stop_window_sec, self.margin_sec)):
            raise ValueError('observation durations must be positive and finite')

    @property
    def duration_sec(self):
        return self.task_deadline_sec + self.stop_window_sec + self.margin_sec

    def complete(self, simulation_time):
        if not math.isfinite(simulation_time) or simulation_time < self.start_sec:
            raise ValueError('invalid or regressed simulation clock')
        return simulation_time - self.start_sec >= self.duration_sec
