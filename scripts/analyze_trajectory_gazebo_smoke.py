#!/usr/bin/env python3
"""Summarize closed integration bags, including failed and stationary trials."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag


def distribution(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(count=len(values), min=float(values.min()),
                p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)),
                max=float(values.max()))


def analyze(directory):
    directory = Path(directory)
    report = json.loads((directory/'summary.json').read_text())
    bag_path = directory/'run.bag'
    if not report['closed_bag'] or not bag_path.is_file():
        raise ValueError('analysis requires a closed bag')
    audits, failures, horizons, snapshots = [], [], [], {}
    states, commands = Counter(), []
    solved_states, rti_counts = Counter(), Counter()
    odom_speed, odom_omega, callbacks, ages, solve_times = [], [], [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        report['topic_counts'] = {k:v.message_count for k,v in bag.get_type_and_topic_info().topics.items()}
        for topic, msg, stamp in bag.read_messages(topics=[
                '/spmpc/debug/control_cycle_audit', '/spmpc/debug/predicted_horizon',
                '/spmpc/debug/pre_solve_snapshot', '/cmd_vel', '/odom']):
            if topic.endswith('control_cycle_audit'):
                audits.append(msg.cycle_id)
                states[msg.status] += 1
                if msg.command_was_published:
                    callbacks.append((msg.command_publish_stamp-msg.cycle_start_stamp).to_sec()*1000.)
                    if msg.solver_input_epoch.to_nsec() > 0:
                        ages.append((msg.command_publish_stamp-msg.solver_input_epoch).to_sec()*1000.)
                if msg.solve_attempted:
                    solved_states[msg.solver_status] += 1
                    solve_times.append((msg.solve_end_stamp-msg.solve_start_stamp).to_sec()*1000.)
                    failures.append(dict(cycle_id=msg.cycle_id, solver_status=msg.solver_status,
                                         status=msg.status, solve_success=msg.solve_success,
                                         command_accepted=msg.command_accepted,
                                         published_v=msg.published_cmd_v, published_omega=msg.published_cmd_omega))
            elif topic.endswith('pre_solve_snapshot') and msg.rti_iterations > 0:
                snapshots[msg.cycle_id] = dict(limit=msg.max_prediction_defect, source=msg.warm_start_source)
            elif topic.endswith('predicted_horizon') and msg.rti_iterations > 0:
                rti_counts[str(msg.rti_iterations)] += 1
                horizons.append(dict(cycle_id=msg.cycle_id, status=msg.solver_status,
                                     rti_iterations=msg.rti_iterations, valid=msg.valid,
                                     dynamics_max_defect=msg.dynamics_max_defect))
            elif topic == '/cmd_vel':
                commands.append([msg.linear.x, msg.angular.z])
            elif topic == '/odom':
                odom_speed.append(abs(msg.twist.twist.linear.x))
                odom_omega.append(abs(msg.twist.twist.angular.z))
    for row in horizons:
        row.update(snapshots.get(row['cycle_id'], {}))
    report.update(audit_status_counts=dict(states), solve_attempt_status_counts=dict(solved_states),
                  first_audit_cycle=min(audits) if audits else None,
                  callback_ms=distribution(callbacks), input_age_ms=distribution(ages),
                  solve_ms=distribution(solve_times), rti_counts=dict(rti_counts),
                  dynamics_defect=distribution([r['dynamics_max_defect'] for r in horizons]),
                  nonzero_command_count=sum(abs(v)>1e-8 or abs(w)>1e-8 for v,w in commands),
                  odom_abs_v=distribution(odom_speed), odom_abs_omega=distribution(odom_omega),
                  solve_attempts=failures, solved_horizons=horizons,
                  analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (directory/'analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return {key: report[key] for key in ['profile', 'goal_reached', 'valid_fresh_lifecycle',
            'closed_bag', 'first_audit_cycle', 'solve_attempt_status_counts', 'rti_counts',
            'dynamics_defect', 'nonzero_command_count', 'callback_ms']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', nargs='+', type=Path)
    args = parser.parse_args()
    for case in args.cases:
        print(str(case), json.dumps(analyze(case), ensure_ascii=False))
