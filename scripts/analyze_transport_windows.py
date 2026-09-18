#!/usr/bin/env python3
"""Reconstruct development parking candidates from archived ROS bags.

Commands use publication timestamps. FIFO clearance is inferred from a sustained
zero command and the frozen delay, not from GOAL_REACHED or a measured FIFO flag.
No archived experiment is modified. Heights are model outputs, in metres.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import rosbag
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/scout_apps/control/spmpc_local_planner/scripts'))
from planning.stopping import stopping_windows


def sustained_start(series, good):
    suffix = np.logical_and.accumulate(np.asarray(good, bool)[::-1])[::-1]
    indices = np.flatnonzero(suffix)
    return None if len(indices) == 0 else float(series[indices[0], 0])


def interval_covered(series, begin, end, gap):
    times = series[:, 0]
    if times[0] > begin or times[-1] < end:
        return False
    left = max(0, int(np.searchsorted(times, begin, side='right')) - 1)
    right = min(len(times), int(np.searchsorted(times, end, side='left')) + 1)
    return bool(np.all(np.diff(times[left:right]) <= gap))


def analyze_case(case, max_gap=.1, command_epsilon=1e-6):
    task = json.loads((case/'task.json').read_text())
    params = yaml.safe_load((case/'live_params.yaml').read_text())
    heights, velocities, commands, poses = {}, {}, {}, []
    audit_frames, path_frames = set(), set()
    start = None
    invalid_observer = []
    topics = ['/scout/global_path_fixed', '/spmpc/debug/control_cycle_audit',
              '/spmpc/debug/raw_state', '/spmpc/debug/slosh_observer_odom', '/odom']
    with rosbag.Bag(str(case/'run.bag')) as bag:
        for topic, msg, stamp in bag.read_messages(topics=topics):
            if topic == '/scout/global_path_fixed':
                epoch = msg.header.stamp.to_sec()
                if start is not None and start != epoch:
                    raise ValueError('path epoch changed')
                start = epoch
                path_frames.add(msg.header.frame_id)
            elif topic.endswith('control_cycle_audit'):
                audit_frames.add(msg.header.frame_id)
                if msg.command_was_published:
                    commands[msg.command_publish_stamp.to_nsec()] = [msg.published_cmd_v, msg.published_cmd_omega]
            elif topic.endswith('raw_state'):
                poses.append([stamp.to_sec(), *msg.data[:3]])
            elif topic.endswith('slosh_observer_odom'):
                if not msg.valid:
                    invalid_observer.append(stamp.to_sec())
                    continue
                if msg.source != 1 or msg.liquid_model_version != 1 or msg.reset_epoch != 0:
                    raise ValueError('observer identity changed in ' + str(case))
                heights[msg.state_stamp.to_nsec()] = [msg.modal_height_m]
            elif topic == '/odom':
                velocities[msg.header.stamp.to_nsec()] = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
    if start is None:
        raise ValueError('missing task epoch')
    def series(records):
        return np.asarray([[t*1e-9-start, *v] for t, v in sorted(records.items())], float)
    height, velocity, command = map(series, (heights, velocities, commands))
    pose = np.asarray(poses, float)
    pose[:, 0] -= start
    # Input is the aligned robot in the reference frame (publishRawState),
    # corroborated by live frame config and ControlCycleAudit headers.
    frames_match = (audit_frames == path_frames == {task['frame_id']} and
                    params['frames']['reference_target'] == task['frame_id'])
    goal = np.asarray(task['goal_pose'], float)
    error = pose[:, 1:4] - goal
    pos_ok = np.linalg.norm(error[:, :2], axis=1) <= task.get('goal_position_tolerance', .05)
    yaw_error = np.arctan2(np.sin(error[:, 2]), np.cos(error[:, 2]))
    pos_ok &= np.abs(yaw_error) <= task.get('goal_yaw_tolerance', .1)
    vel_ok = ((np.abs(velocity[:, 1]) <= task.get('stop_speed_tolerance', .01)) &
              (np.abs(velocity[:, 2]) <= task.get('stop_omega_tolerance', .02)))
    zero = np.max(np.abs(command[:, 1:]), axis=1) <= command_epsilon
    t_pose = sustained_start(pose, pos_ok)
    t_velocity = sustained_start(velocity, vel_ok)
    t_zero = sustained_start(command, zero)
    execution = params['execution_model']
    delay = max(execution['linear_delay_sec'], execution['angular_delay_sec'])
    candidate = (max(0., t_pose, t_velocity, t_zero + delay)
                 if all(t is not None for t in (t_pose, t_velocity, t_zero)) else None)
    result = dict(case=str(case), formal=False, frame_contract_matches=frames_match,
                  pose_time_basis='raw_state bag receipt; reference-frame provenance checked in producer',
                  first_sustained_pose_sec=t_pose, first_sustained_low_speed_sec=t_velocity,
                  first_sustained_zero_publication_sec=t_zero, inferred_fifo_delay_sec=delay,
                  t_stop_candidate_sec=candidate, fifo_clearance_measured=False,
                  command_epsilon=command_epsilon, maximum_allowed_sample_gap_sec=max_gap,
                  invalid_observer_after_task_start=sum(t >= start for t in invalid_observer))
    if candidate is not None:
        end = candidate + task['stop_window']
        result['windows'] = stopping_windows(height[:, 0], height[:, 1], candidate,
                                             task['stop_window'], max_sample_gap_sec=max_gap)
        result['stop_evidence_covers_tail'] = bool(frames_match and all(
            interval_covered(a, candidate, end, max_gap) for a in (pose, velocity, command)))
        result['height_coverage_after_stop_sec'] = float(height[-1, 0] - candidate)
    review = json.loads((case.parent/'review.json').read_text())
    result['archived_anomalies'] = {key: review.get(key) for key in
        ('alignment_rejected', 'budget_exhausted', 'fault_zero_cycles', 'goal_sec')}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output must be new; do not replace previous evidence')
    result = dict(method='persistent pose/speed and first sustained zero publication + frozen maximum delay',
                  evidence='development candidate; FIFO not directly observed and raw pose has receipt timestamps',
                  units='seconds and metres', trials=[analyze_case(case) for case in args.cases])
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
