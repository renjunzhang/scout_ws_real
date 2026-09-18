import pathlib
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parents[2] / "src" / "scout_apps" / "control" / "spmpc_local_planner" / "scripts"))
from planning import process_runner


class ProcessRunnerTest(unittest.TestCase):
    def test_success_and_nonzero_exit_are_recorded(self):
        with tempfile.TemporaryDirectory() as directory:
            log = pathlib.Path(directory) / "run.log"
            result = process_runner.run_process(
                [sys.executable, "-c", "import sys; print('hello'); sys.exit(3)"], log
            )
            self.assertEqual(result.status, "failed")
            self.assertEqual(result.returncode, 3)
            self.assertIn(b"hello", log.read_bytes())

    def test_timeout_kills_process_group(self):
        with tempfile.TemporaryDirectory() as directory:
            log = pathlib.Path(directory) / "run.log"
            pid_file = pathlib.Path(directory) / "child.pid"
            result = process_runner.run_process(
                [sys.executable, "-c", "import os,signal,subprocess,sys,time; p=subprocess.Popen([sys.executable,'-c','import signal,time; signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(30)']); open(%r,'w').write(str(p.pid)); time.sleep(30)" % str(pid_file)],
                log,
                timeout=0.5,
                terminate_grace=0.1,
            )
            self.assertEqual(result.status, "timeout")
            self.assertLess(result.wall_seconds, 3.0)
            self.assertIsNotNone(result.returncode)
            child_pid = int(pid_file.read_text())
            proc_status = pathlib.Path("/proc") / str(child_pid) / "status"
            for _ in range(20):
                if not proc_status.exists():
                    break
                time.sleep(0.05)
            if proc_status.exists():
                self.assertIn("State:\tZ", proc_status.read_text())

    def test_real_normal_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            result = process_runner.run_process([sys.executable, "-c", "pass"], pathlib.Path(directory) / "run.log")
            self.assertEqual((result.status, result.returncode), ("completed", 0))

    def test_invalid_budgets(self):
        with tempfile.TemporaryDirectory() as directory:
            log = pathlib.Path(directory) / "run.log"
            for invalid in (0, -1, float("nan"), float("inf")):
                with self.assertRaises(ValueError):
                    process_runner.run_process([sys.executable, "-c", "pass"], log, timeout=invalid)
            with self.assertRaises(ValueError):
                process_runner.run_process([sys.executable, "-c", "pass"], log, terminate_grace=0)

    def test_stdout_and_stderr_are_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            log = pathlib.Path(directory) / "run.log"
            process_runner.run_process(
                [sys.executable, "-c", "import sys; print('out'); print('err', file=sys.stderr)"], log
            )
            contents = log.read_text()
            self.assertIn("out", contents)
            self.assertIn("err", contents)
