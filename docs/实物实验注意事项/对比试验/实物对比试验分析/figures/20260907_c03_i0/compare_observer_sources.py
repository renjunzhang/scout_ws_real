"""Compare the already recorded odom/IMU liquid monitors on frozen C03 windows.

Read-only ROS bag extraction; no controller changes or time-shift fitting.
Run with ROS Noetic and this workspace sourced. Height remains a model metric.
"""
from pathlib import Path
import argparse
import json
import numpy as np
import rosbag


def extract_monitor(bag, cache):
    if cache.exists():
        return json.loads(cache.read_text())
    rows = []
    with rosbag.Bag(bag) as reader:
        for _, msg, _ in reader.read_messages(topics=['/spmpc/debug/slosh_observer_odom']):
            row = {}
            for key in msg.__slots__:
                value = getattr(msg, key)
                if hasattr(value, 'to_sec'):
                    row[key] = value.to_sec()
                elif isinstance(value, (float, int, str, bool)):
                    row[key] = value
            rows.append(row)
    assert rows
    cache.write_text(json.dumps(rows))
    return rows


def window_stats(rows, lo, hi):
    # Measurement time also retains invalid samples whose state_stamp is zero.
    rr = [r for r in rows if lo <= r['measurement_stamp'] <= hi]
    assert len(rr) > 2
    valid = all(r['valid'] and r['configured'] for r in rr)
    t = np.array([r['state_stamp'] for r in rr])
    dt = np.diff(t)
    quality = {
        'invalid_count': sum(not r['valid'] or not r['configured'] for r in rr),
        'input_statuses': sorted({r['input_status'] for r in rr}),
        'state_measurement_max_skew_sec': max(abs(r['state_stamp']-r['measurement_stamp']) for r in rr),
        'nonpositive_state_intervals': int(np.sum(dt <= 0)),
        'max_state_gap_ms': float(dt.max()*1000),
        'max_sample_dt_residual_ms': float(np.max(np.abs(dt-np.array([r['sample_dt_sec'] for r in rr[1:]])))*1000),
        'reset_epochs': sorted({r['reset_epoch'] for r in rr}),
        'update_skip_count': int(np.sum(np.diff([r['observer_update_count'] for r in rr]) != 1)),
        'axes': sorted({r['excitation_axes_frame'] for r in rr}),
        'reference_points': sorted({r['excitation_reference_point'] for r in rr}),
        'window_coverage': min(r['measurement_stamp'] for r in rows) <= lo and max(r['measurement_stamp'] for r in rows) >= hi,
    }
    assert valid and quality['state_measurement_max_skew_sec'] == 0
    assert np.all(dt > 0) and quality['update_skip_count'] == 0
    assert len(quality['reset_epochs']) == 1 and quality['window_coverage']
    h = np.array([r['modal_height_m']*1000 for r in rr])
    assert np.isfinite(h).all()
    return dict(n=len(rr), p95=float(np.percentile(h, 95)),
                rms=float(np.sqrt(np.mean(h*h))), peak=float(h.max()), quality=quality)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis-dir', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    source = args.analysis_dir
    task = json.loads((source/'comparison.json').read_text())['rows']
    progress = {r['label']:r for r in json.loads((source/'progress_10_90.json').read_text())['rows']}
    results = []
    for base in task:
        d = json.loads((source/(base['name']+'.json')).read_text())
        odom = extract_monitor(base['bag'], source/('odom_monitor_'+base['name']+'.json'))
        mid = progress[base['label']]
        windows = {'path10_90': [mid['enter_stamp'], mid['leave_stamp']],
                   'task': [base['task_start_sec'], base['goal_sec']],
                   'tail5s': [base['goal_sec'], base['goal_sec']+5]}
        row = dict(label=base['label'], bag=base['bag'], windows={})
        for key, (lo, hi) in windows.items():
            row['windows'][key] = {'bounds': [lo, hi], 'imu': window_stats(d['i0'], lo, hi),
                                  'odom': window_stats(odom, lo, hi)}
        results.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    report = {'method': 'Same fixed C03 task/path10-90/tail5s absolute windows; '
              'native valid observer modal_height_m * 1000 at state_stamp=measurement_stamp. '
              'No added delay or phase fitting. Odom/IMU excitation and processing differ; '
              'their model heights are complementary diagnostics, not two liquid truth sensors.',
              'rows': results}
    (source/'observer_source_comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
