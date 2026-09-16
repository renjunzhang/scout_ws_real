#!/usr/bin/env python3
"""Offline cold-start RTI diagnosis from schema-8 bags; never publishes ROS data.

Only the first solve in a fresh capsule is reproduced: snapshots contain primal
guesses, not dual/internal history. Extra iterations and condensing choices are
diagnostic counterfactuals, not online timing or closed-loop success evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import subprocess

import casadi as ca
import numpy as np
import rosbag

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/scout_apps/control/spmpc_local_planner'
sys.path.insert(0, str(PKG / 'scripts/acados'))
from spmpc_acados_model import export_spmpc_b0_symbols, export_spmpc_slosh_symbols


def first_pair(bag_path):
    snapshots, horizons = {}, {}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=[
                '/spmpc/debug/pre_solve_snapshot', '/spmpc/debug/predicted_horizon']):
            if msg.rti_iterations <= 0:
                continue
            (snapshots if topic.endswith('pre_solve_snapshot') else horizons)[msg.cycle_id] = msg
    if not snapshots:
        raise ValueError('no executed RTI in the bag')
    cycle = min(snapshots)
    snap = snapshots[cycle]
    if snap.schema_version != 8 or snap.have_previous_solution or snap.have_previous_control:
        raise ValueError('requires the first cold-start schema-8 snapshot')
    return snap, horizons[cycle]


def apply_snapshot(solver, snap, prune_implied_bounds=False):
    n, nx, nu, np_ = snap.horizon_steps, snap.state_width, snap.control_width, snap.parameter_width
    expected = (28, 3, 104) if snap.slosh_enabled else (24, 3, 92)
    if (nx, nu, np_) != expected or snap.cost_model_version != 3 or snap.liquid_model_version != 1:
        raise ValueError('incompatible generated model/snapshot')
    params = np.asarray(snap.stage_parameters).reshape(n + 1, np_)
    xs = np.asarray(snap.initial_guess_states).reshape(n + 1, nx)
    us = np.asarray(snap.initial_guess_controls).reshape(n, nu)
    x0 = np.array([snap.robot_x, snap.robot_y, snap.robot_yaw, snap.robot_v,
                   snap.s0, snap.robot_omega, snap.actuator_v_cmd, snap.actuator_omega_cmd,
                   *snap.actuator_linear_delay_queue, *snap.actuator_angular_delay_queue,
                   snap.actuator_a_cmd_memory] +
                  ([snap.eta_x, snap.eta_x_dot, snap.eta_y, snap.eta_y_dot] if snap.slosh_enabled else []))
    for k in range(n + 1):
        solver.set(k, 'p', params[k])
        solver.set(k, 'x', xs[k])
        if k < n:
            solver.set(k, 'u', us[k])
            solver.constraints_set(k, 'lbu', np.array([snap.a_min, snap.alpha_or_omega_min, snap.v_s_min]))
            solver.constraints_set(k, 'ubu', np.array([snap.a_max, snap.alpha_or_omega_max, snap.v_s_max]))
            solver.constraints_set(k, 'lg', np.array([-snap.delta_a_max]))
            solver.constraints_set(k, 'ug', np.array([snap.delta_a_max]))
        if k == 0:
            lo, hi = x0, x0
        else:
            lo = np.array([snap.v_min, snap.omega_min, 0., snap.omega_min] + [0.] * 5 + [snap.omega_min] * 10 + [snap.a_min])
            hi = np.array([snap.v_max, snap.omega_max, snap.v_max, snap.omega_max] + [snap.v_max] * 5 + [snap.omega_max] * 10 + [snap.a_max])
            if params[k, list(snap.parameter_names).index('task_goal_active')] > .5:
                lo[2:] = hi[2:] = 0.
            elif prune_implied_bounds:
                # FIFO entries copied from a bounded command state and memory
                # copied from a bounded control already satisfy these rows.
                for j, delay, offset, bound_start in [(0, 5, 8, 4), (1, 10, 13, 9)]:
                    for q in range(delay):
                        source = q + k
                        value = x0[offset + source] if source < delay else None
                        index = bound_start + q
                        if value is None or lo[index] <= value <= hi[index]:
                            lo[index], hi[index] = -1e15, 1e15
                lo[19], hi[19] = -1e15, 1e15
        solver.constraints_set(k, 'lbx', lo)
        solver.constraints_set(k, 'ubx', hi)
    return params, xs, us


def diagnose(bag_path, cond_n, iterations, seed_mode, qp_warm_start, prune_implied_bounds, reroll_between):
    from acados_template import AcadosOcpSolver
    snap, horizon = first_pair(bag_path)
    sym = (export_spmpc_slosh_symbols if snap.slosh_enabled else export_spmpc_b0_symbols)()
    step = ca.Function('step', [sym['x'], sym['u'], sym['p']], [sym['disc_dyn']])
    gen = PKG / 'generated/acados' / sym['name']
    solver = AcadosOcpSolver(None, json_file=str(gen / ('acados_ocp_' + sym['name'] + '.json')),
                            generate=False, build=False, verbose=False)
    if cond_n:
        solver.update_qp_solver_cond_N(cond_n)
    if qp_warm_start:
        solver.options_set('qp_warm_start', qp_warm_start)
        solver.options_set('warm_start_first_qp', True)
    params, xs, us = apply_snapshot(solver, snap, prune_implied_bounds)
    if seed_mode == 'bounded_rollout':
        for k in range(snap.horizon_steps):
            us[k, 0] = np.clip(us[k, 0], max(snap.a_min, -xs[k, 6] / snap.dt),
                              min(snap.a_max, (snap.v_max - xs[k, 6]) / snap.dt))
            us[k, 1] = np.clip(us[k, 1], max(snap.alpha_or_omega_min, (snap.omega_min - xs[k, 7]) / snap.dt),
                              min(snap.alpha_or_omega_max, (snap.omega_max - xs[k, 7]) / snap.dt))
            us[k, 2] = np.clip(us[k, 2], snap.v_s_min, snap.v_s_max)
            xs[k + 1] = np.asarray(step(xs[k], us[k], params[k])).ravel()
            solver.set(k, 'u', us[k])
            solver.set(k + 1, 'x', xs[k + 1])

    def defect(x, u):
        replay = np.array([np.asarray(step(x[k], u[k], params[k])).ravel() for k in range(snap.horizon_steps)])
        errors = np.abs(x[1:] - replay)
        stage, component = np.unravel_index(np.argmax(errors), errors.shape)
        return dict(max=float(errors[stage, component]), stage=int(stage), component=int(component),
                    per_component_max=errors.max(axis=0).tolist())

    result = dict(bag=str(bag_path), cycle_id=snap.cycle_id, cond_n=cond_n or 'generated', seed_mode=seed_mode,
                  qp_warm_start=qp_warm_start,
                  prune_implied_bounds=prune_implied_bounds,
                  reroll_between=reroll_between,
                  warm_start_source=snap.warm_start_source, seed_defect=defect(xs, us),
                  recorded_iterations=snap.rti_iterations, recorded_defect=horizon.dynamics_max_defect,
                  generated_library_sha256=hashlib.sha256((gen / ('libacados_ocp_solver_' + sym['name'] + '.so')).read_bytes()).hexdigest(),
                  iterations=[])
    for i in range(iterations):
        start = time.perf_counter()
        status = solver.solve()
        elapsed = 1000. * (time.perf_counter() - start)
        xs = np.array([solver.get(k, 'x') for k in range(snap.horizon_steps + 1)])
        us = np.array([solver.get(k, 'u') for k in range(snap.horizon_steps)])
        row = dict(iteration=i + 1, status=status, wall_ms=elapsed, defect=defect(xs, us))
        row['qp_iter'] = np.asarray(solver.get_stats('qp_iter')).tolist()
        row['qp_status'] = np.asarray(solver.get_stats('qp_stat')).tolist()
        row['acados_ms'] = {key: 1000. * float(solver.get_stats(key)) for key in
                            ['time_tot', 'time_lin', 'time_sim', 'time_qp', 'time_reg']}
        if i + 1 == snap.rti_iterations:
            recorded = np.asarray(horizon.model_states).reshape(xs.shape)
            row['recorded_state_linf'] = float(np.max(np.abs(xs - recorded)))
        result['iterations'].append(row)
        if status:
            break
        if reroll_between:
            for k in range(snap.horizon_steps):
                xs[k + 1] = np.asarray(step(xs[k], us[k], params[k])).ravel()
                solver.set(k + 1, 'x', xs[k + 1])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--iterations', type=int, default=8)
    parser.add_argument('--cond-n', type=int, nargs='+', default=[0])
    parser.add_argument('--seed-mode', choices=['recorded', 'bounded_rollout'], default='recorded')
    parser.add_argument('--qp-warm-start', type=int, choices=[0, 1, 2], default=0)
    parser.add_argument('--prune-implied-bounds', action='store_true')
    parser.add_argument('--reroll-between', action='store_true')
    args = parser.parse_args()
    if args.iterations < 1 or any(n < 0 or n > 60 for n in args.cond_n):
        parser.error('iterations must be positive; cond-n must be 0 (generated) or 1..60')
    results = [diagnose(args.bag.resolve(), n, args.iterations, args.seed_mode, args.qp_warm_start, args.prune_implied_bounds, args.reroll_between) for n in args.cond_n]
    report = dict(evidence='OFFLINE_COLD_START_DIAGNOSIS', online_budget=False,
                  bag_sha256=hashlib.sha256(args.bag.read_bytes()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  git_sha=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
                  results=results)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    for result in results:
        print('cond_N:', result['cond_n'], 'seed:', result['seed_defect']['max'], flush=True)
        for row in result['iterations']:
            print(row['iteration'], row['status'], row['defect']['max'], row['wall_ms'],
                  row.get('recorded_state_linf'), row['qp_iter'], row['acados_ms'], flush=True)


if __name__ == '__main__':
    main()
