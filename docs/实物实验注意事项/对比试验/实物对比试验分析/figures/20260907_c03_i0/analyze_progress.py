"""C03 path-progress 10--90% I0 comparison using existing primitive caches.

Only geometric progress is interpolated; height remains on native I0 timestamps.
Planner input positions include common-epoch rollout, so progress is an estimate,
not independent mocap truth. No bag-receipt timestamps or fitted delays are used.
"""
from pathlib import Path
import argparse
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def project(xy, path):
    segment = np.diff(path, axis=0)
    length = np.linalg.norm(segment, axis=1)
    assert np.all(length > 0)
    offset = xy[:, None, :] - path[:-1][None, :, :]
    fraction = np.clip(np.sum(offset * segment[None, :, :], axis=2) / length**2, 0, 1)
    distance2 = np.sum((offset - fraction[:, :, None] * segment[None, :, :])**2, axis=2)
    idx = np.argmin(distance2, axis=1)
    return np.r_[0, np.cumsum(length)][idx] + fraction[np.arange(len(xy)), idx] * length[idx]


def stats(h):
    assert len(h) and np.isfinite(h).all()
    return dict(n=len(h), p95=float(np.percentile(h, 95)),
                rms=float(np.sqrt(np.mean(h*h))), peak=float(h.max()))


def crossing(t, s, bound):
    idx = np.flatnonzero((s[:-1] < bound) & (s[1:] >= bound))
    assert len(idx) == 1, 'Require one forward crossing per boundary'
    i = idx[0]
    assert not np.any((s[:-1] >= bound) & (s[1:] < bound)), 'Boundary recrossing'
    return float(t[i] + (bound-s[i])/(s[i+1]-s[i]) * (t[i+1]-t[i]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis-dir', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    source = args.analysis_dir
    out = args.output_dir or source
    out.mkdir(parents=True, exist_ok=True)
    comparison = json.loads((source/'comparison.json').read_text())
    rows = []
    curves = []
    for base in comparison['rows']:
        d = json.loads((source/(base['name']+'.json')).read_text())
        path = np.asarray(d['path'][0]['xy'])
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        lo, hi = .1*length, .9*length
        start, goal = base['task_start_sec'], base['goal_sec']
        ss = [r for r in d['snapshot'] if r['valid'] and start-.1 <= r['robot_state_stamp'] <= goal]
        assert all(r['frame_id'] == d['path'][0]['frame'] == 'map' for r in ss)
        assert all(abs(r['reference_length']-length) < 1e-8 for r in ss)
        st = np.array([r['robot_state_stamp'] for r in ss])
        xy = np.array([[r['robot_x'], r['robot_y']] for r in ss])
        assert np.all(np.diff(st) > 0)
        sp = project(xy, path)
        enter, leave = crossing(st, sp, lo), crossing(st, sp, hi)
        assert start < enter < leave < goal
        ii = [r for r in d['i0'] if start <= r['state_stamp'] <= goal]
        assert all(r['valid'] and r['input_status'] == 'READY' and
                   r['state_stamp'] == r['measurement_stamp'] for r in ii)
        it = np.array([r['state_stamp'] for r in ii])
        height = np.array([r['modal_height_m']*1000 for r in ii])
        assert np.all(np.diff(it) > 0)
        coverage = (it >= st[0]) & (it <= st[-1])
        # Piecewise-linear s(t) from geometric position projections, not OCP s0.
        progress = np.interp(it, st, sp)
        mask = coverage & (progress >= lo) & (progress <= hi)
        idx = np.flatnonzero(mask)
        assert len(idx) and np.all(np.diff(idx) == 1)
        assert np.array_equal(mask, (it >= enter) & (it <= leave))
        metric = stats(height[mask])
        w = np.diff(np.r_[enter, (it[mask][:-1]+it[mask][1:])/2, leave])
        assert np.all(w > 0)
        metric['time_weighted_rms_check'] = float(np.sqrt(np.sum(w*height[mask]**2)/w.sum()))
        middle = (st >= enter-.05) & (st <= leave+.05)
        steps = np.diff(sp[middle])
        quality = dict(
            snapshot_max_gap_ms=float(np.diff(st[middle]).max()*1000),
            backward_projection_steps=int(np.sum(steps < 0)),
            max_backward_projection_m=float(max(0, -steps.min())),
            i0_max_gap_ms=float(np.diff(it[mask]).max()*1000),
            i0_update_skip_count=int(np.sum(np.diff([r['observer_update_count'] for r, take in zip(ii, mask) if take]) != 1)),
            i0_reset_epochs=sorted({r['reset_epoch'] for r, take in zip(ii, mask) if take}),
            boundary_crossings=[1, 1], selected_samples_contiguous=True,
            snapshot_alignment_statuses=sorted({r['state_alignment_status'] for r in ss}),
        )
        assert quality['snapshot_max_gap_ms'] < 60
        assert quality['i0_update_skip_count'] == 0 and len(quality['i0_reset_epochs']) == 1
        # Frozen, symmetric +/- one controller period, not a per-bag fitted delay.
        sensitivity = {}
        for shift_ms in [-1000/30, 0, 1000/30]:
            shift = shift_ms/1000
            m = (it >= enter+shift) & (it <= leave+shift)
            sensitivity[f'{shift_ms:+.6f}_ms'] = stats(height[m])
        # Report old s0 convention separately, never substitute it for geometry.
        ocp_s = np.array([r['s0'] for r in ss])
        ocp_mask = coverage & (np.interp(it, st, ocp_s) >= lo) & (np.interp(it, st, ocp_s) <= hi)
        row = dict(label=base['label'], name=base['name'], bag=base['bag'],
                   path_length_m=length, progress_bounds_m=[lo, hi],
                   enter_stamp=enter, leave_stamp=leave,
                   enter_since_start_sec=enter-start, leave_since_start_sec=leave-start,
                   duration_sec=leave-enter, height_mm=metric, quality=quality,
                   common_boundary_shift_sensitivity=sensitivity,
                   ocp_s0_window_supplement_height_mm=stats(height[ocp_mask]))
        rows.append(row)
        curves.append((base['label'], progress[coverage]/length*100, height[coverage]))
        print(json.dumps(row, ensure_ascii=False))
    result = dict(scene='20260907_c03',
                  method='Project valid PreSolveSnapshot robot_x/y (map) onto fixed path; '
                         'linearly interpolate geometric s at native I0 state_stamp; '
                         'retain 10% <= s/L <= 90% within first-command-to-GOAL_REACHED. '
                         'Positions are planner common-epoch estimates including short rollout. '
                         'No receipt-time alignment or fitted delay. Sample P95/RMS/max in mm.',
                  rows=rows)
    (out/'progress_10_90.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    with (out/'progress_10_90.csv').open('w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['condition', 'start_since_command_s', 'end_since_command_s',
                         'window_s', 'samples', 'P95_mm', 'RMS_mm', 'peak_mm'])
        for r in rows:
            writer.writerow([r['label'], r['enter_since_start_sec'], r['leave_since_start_sec'],
                             r['duration_sec'], *[r['height_mm'][k] for k in ['n', 'p95', 'rms', 'peak']]])
    colors = ['#666666', '#3978b6', '#dfa12b', '#c54444']
    plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': .18})
    fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True, sharey=True)
    for ax, (label, progress, h), color in zip(axes, curves, colors):
        ax.plot(progress, h, color=color, lw=.75)
        ax.axvspan(10, 90, color='#5ea9b0', alpha=.09)
        ax.axvline(10, color='black', lw=.8, ls='--')
        ax.axvline(90, color='black', lw=.8, ls='--')
        ax.set_ylabel(label+'\nI0 height [mm]')
        ax.set_xlim(0, 100)
    axes[0].set_title('C03: I0 height vs geometric path progress; shaded = 10--90%')
    axes[-1].set_xlabel('Path progress from planner position estimate [%]')
    fig.tight_layout()
    fig.savefig(out/'03_progress_10_90.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
