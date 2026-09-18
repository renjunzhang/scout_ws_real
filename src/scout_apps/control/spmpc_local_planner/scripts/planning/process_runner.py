"""Run one command with a wall-clock budget and capture its output."""

from dataclasses import dataclass
import os
import math
import signal
import subprocess
import time
from typing import List, Optional


@dataclass
class ProcessResult:
    command: List[str]
    returncode: int
    status: str
    wall_seconds: float
    timeout: Optional[float]


def _stop_group(process, grace):
    """Terminate the process group, escalating after the grace period."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def run_process(argv, log_path, timeout=None, terminate_grace=0.5):
    """Run *argv* in its own process group, writing stdout and stderr to log_path.

    ``timeout`` is a wall-clock limit for the complete child process, including
    descendants.  It is intentionally passed as a value by the caller, so a
    caller can implement a larger total deadline across several invocations.
    """
    if timeout is not None and (not math.isfinite(timeout) or timeout <= 0):
        raise ValueError("timeout must be positive or None")
    if not math.isfinite(terminate_grace) or terminate_grace <= 0:
        raise ValueError("terminate_grace must be positive")
    command = list(argv)
    started = time.monotonic()
    status = "completed"

    with open(log_path, "xb") as log_file:
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            shell=False,
            start_new_session=True,
        )
        try:
            returncode = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            status = "timeout"
            _stop_group(process, terminate_grace)
            returncode = process.returncode
        except KeyboardInterrupt:
            _stop_group(process, terminate_grace)
            raise
    if status == "completed" and returncode != 0:
        status = "failed"
    return ProcessResult(command, returncode, status, time.monotonic() - started, timeout)
