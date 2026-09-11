#!/usr/bin/env python3
"""Exercise profile dispatch and motion envelopes without connecting to ROS."""
import importlib.util
from pathlib import Path
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'
SPEC = importlib.util.spec_from_file_location('spin_profile', SCRIPTS/'mocap_imu_motion_sequence.py')
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def arguments(profile='spin_center'):
    argv = ['--arm-motion', 'YES', '--timeline-path', '/unused', '--log-path', '/unused', '--profile', profile]
    if profile == 'spin_center':
        argv += ['--static-pre-sec', '5', '--static-post-sec', '5', '--settle-sec', '5', '--spin-hold-sec', '32']
    return M.build_parser().parse_args(argv)


class SpySequence(M.MotionSequence):
    def __init__(self, args):
        self.args, self.calls, self.sequence_complete = args, [], False
    def start_command_publisher(self): pass
    def wait_for_connections(self): pass
    def publish_status(self, value): self.calls.append(('status', value))
    def set_command(self, v, w, event=''): self.calls.append(('command', v, w))
    def log(self, text): pass
    def interruptible_hold(self, seconds): pass
    def publish_event(self, event): self.calls.append(('event', event))
    def hold_zero(self, label, seconds): self.calls.append(('zero', label, seconds))
    def run_segment(self, label, v, w, seconds): self.calls.append(('motion', label, v, w, seconds))
    def run_spin_reversal_sequence(self): self.calls.append(('reversal',))
    def run_s_pattern_round_trip(self, pattern, repetition): self.calls.append(('s', pattern, repetition))


class SpinProfileTest(unittest.TestCase):
    def test_center_has_only_two_opposite_spins_separated_by_stop(self):
        a = arguments(); M.validate_args(a)
        seq = SpySequence(a); seq.run()
        phases = [c for c in seq.calls if c[0] in ('zero', 'motion', 'reversal', 's')]
        self.assertEqual(phases, [
            ('zero', 'static_pre', 5.), ('motion', 'spin_ccw_hold', 0., .2, 32.),
            ('zero', 'settle_after_spin_ccw', 5.), ('motion', 'spin_cw_hold', 0., -.2, 32.),
            ('zero', 'static_post', 5.)])
        self.assertTrue(seq.sequence_complete)
        self.assertEqual(seq.calls[-1], ('status', 'COMPLETE'))

    def test_planar_sequence_and_static_contract_preserved(self):
        a = arguments('planar'); M.validate_args(a)
        seq = SpySequence(a); seq.run()
        self.assertEqual(sum(c[0] == 's' for c in seq.calls), 6)
        self.assertEqual(sum(c[0] == 'reversal' for c in seq.calls), 1)
        self.assertTrue(any(c[0] == 'motion' and c[2] != 0 for c in seq.calls))
        a.static_pre_sec = 5
        with self.assertRaises(ValueError): M.validate_args(a)

    def test_center_rejects_unarmed_short_fast_or_unsettled_motion(self):
        for key, value in [('arm_motion', 'NO'), ('spin_hold_sec', 46.), ('spin_hold_sec', 5.),
                           ('spin_omega', .31), ('settle_sec', 2.), ('static_post_sec', 1.)]:
            a = arguments(); setattr(a, key, value)
            with self.subTest(key=key, value=value), self.assertRaises(ValueError): M.validate_args(a)

    def test_runtime_blocks_translation_before_any_publisher_or_lease_access(self):
        seq = object.__new__(M.MotionSequence); seq.args = arguments()
        for v, w in [(.01, 0.), (0., .21), (0., float('nan'))]:
            with self.subTest(v=v, w=w), self.assertRaises(M.SequenceAbort):
                seq.set_command(v, w)


if __name__ == '__main__':
    unittest.main()
