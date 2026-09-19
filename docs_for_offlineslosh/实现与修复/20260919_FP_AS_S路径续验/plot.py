#!/usr/bin/env python3
"""Replot the one S-path trial from archived plan and bag-derived arrays."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', type=Path, default=Path(
        '/data/a/scout_sim_replacement/results/20260919_fp_as_s_followup_181514'))
    parser.add_argument('--output', type=Path, default=Path(__file__).with_name('s_trial.png'))
    args = parser.parse_args()
    plan = json.loads((args.results/'fp_as_s_plan.json').read_text())
    assessment = json.loads((args.results/'fp_as_01/assessment.json').read_text())
    series = np.load(args.results/'fp_as_01/series.npz')
    t = np.array([s['t'] for s in plan['samples']])
    x = np.array([s['state'] for s in plan['samples']])
    height = plan['height_coeff']*np.linalg.norm(x[:, [24, 26]], axis=1)*1000
    stop = assessment['windows']['t_stop_candidate_sec']
    raw, velocity, liquid = [series[k] for k in ['raw', 'velocity', 'height']]
    route = np.array(plan['route'])
    fig = plt.figure(figsize=(10.5, 8.0), constrained_layout=True)
    grid = fig.add_gridspec(3, 2, height_ratios=[1.2, 1, 1.15])
    path = fig.add_subplot(grid[0, :])
    path.plot(route[:, 0], route[:, 1], ':', color='#777777', label='Original route')
    path.plot(x[:, 0], x[:, 1], color='#0072B2', label='Frozen FP-AS plan')
    path.plot(raw[:, 1], raw[:, 2], color='#D55E00', lw=1.1, label='Gazebo localization')
    path.set(xlabel='x (m)', ylabel='y (m)', title='S path: fixed geometry, 40 s nominal duration')
    path.set_aspect('equal', adjustable='datalim')
    path.legend(loc='lower left', ncol=3, fontsize=8)
    for column, state, data_column, label in [(0, 3, 1, 'Linear speed (m/s)'),
                                             (1, 5, 2, 'Yaw rate (rad/s)')]:
        ax = fig.add_subplot(grid[1, column])
        ax.plot(t, x[:, state], '--', color='#0072B2', label='Plan')
        ax.plot(velocity[:, 0], velocity[:, data_column], color='#D55E00', lw=1, label='Odometry')
        ax.set(xlim=(0, stop+5), xlabel='Task time (s)', ylabel=label)
        ax.axvline(stop, color='#555555', ls=':', lw=.8)
        ax.legend(fontsize=8)
    ax = fig.add_subplot(grid[2, :])
    ax.plot(t, height, '--', color='#0072B2', label='Offline model')
    ax.plot(liquid[:, 0], liquid[:, 1]*1000, color='#D55E00', lw=1, label='Odometry-driven model')
    ax.axvline(stop, color='#555555', ls=':', lw=.8, label='Stop candidate')
    ax.axvspan(stop, stop+5, color='#999999', alpha=.10)
    ax.set(xlim=(0, stop+5), xlabel='Task time (s)', ylabel='Modal height (mm)',
           title='Model output only; no independent liquid-surface measurement')
    ax.legend(fontsize=8, ncol=3)
    for ax in fig.axes:
        ax.grid(alpha=.2)
    fig.savefig(args.output, dpi=180)


if __name__ == '__main__':
    main()
