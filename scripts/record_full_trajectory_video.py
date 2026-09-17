#!/usr/bin/env python3
"""Record one fresh Full Gazebo run on an owned, authenticated nested display."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def reachable(port):
    with socket.socket() as s:
        s.settimeout(.2)
        return s.connect_ex(('127.0.0.1', port)) == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ['output', 'setup', 'task', 'plan']:
        parser.add_argument('--'+field, type=Path, required=True)
    parser.add_argument('--ros-port', type=int, default=11892)
    parser.add_argument('--gazebo-port', type=int, default=11926)
    parser.add_argument('--sim-packages', type=Path,
                        default=Path('/data/a/scout_sim_replacement/classic_ws/src'))
    args = parser.parse_args()
    for field in ['setup', 'task', 'plan']:
        if not getattr(args, field).is_file():
            parser.error('missing '+field+' file')
    if args.ros_port == args.gazebo_port:
        parser.error('ROS and Gazebo need different ports')
    if not (args.sim_packages/'scout_mini_proxy_description/package.xml').is_file():
        parser.error('missing simulator robot visualization resources')
    for name in ['Xephyr', 'ffmpeg', 'ffprobe', 'xauth', 'xwininfo']:
        if shutil.which(name) is None:
            parser.error('missing '+name)
    if not os.environ.get('DISPLAY'):
        parser.error('a host X11 display is required for Xephyr')
    if reachable(args.ros_port) or reachable(args.gazebo_port):
        parser.error('fresh run requires empty ROS/Gazebo ports')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    display = next(n for n in range(190, 220) if not Path('/tmp/.X11-unix/X%d' % n).exists())
    auth = out/'Xauthority'
    auth.touch(mode=0o600)
    subprocess.run(['xauth', '-f', str(auth)], input='add :%d MIT-MAGIC-COOKIE-1 %s\n' % (
        display, secrets.token_hex(16)), text=True, check=True, capture_output=True)
    env = dict(os.environ, DISPLAY=':%d' % display, XAUTHORITY=str(auth),
               ROS_MASTER_URI='http://127.0.0.1:%d' % args.ros_port,
               GAZEBO_MASTER_URI='http://127.0.0.1:%d' % args.gazebo_port,
               ROS_LOG_DIR=str(out/'viewer_ros'), QT_X11_NO_MITSHM='1')
    viewer_env = dict(env, ROS_PACKAGE_PATH=str(args.sim_packages.resolve())+':'+
                      env.get('ROS_PACKAGE_PATH', ''))
    children = []
    manifest = dict(evidence='LIVE_FULL_GAZEBO_RVIZ_SCREEN_RECORDING', formal=False,
                    video='full_process.mp4', fps=15, width=1600, height=900,
                    controller_changes=False, replay=False,
                    plan_identity='REPLAYED_FEASIBLE_SEED_NOT_REOPTIMIZED', children=[],
                    git_sha=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip())

    def start(name, command, child_env=env):
        log = (out/(name+'.log')).open('w')
        proc = subprocess.Popen(command, env=child_env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        children.append((name, proc, log))
        manifest['children'].append(dict(name=name, pid=proc.pid))
        return proc

    def stop(child):
        name, proc, log = child
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT if name == 'screen_recorder' else signal.SIGTERM)
            try:
                proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        log.close()

    def interrupted(signum, _frame):
        raise RuntimeError('recording interrupted: %d' % signum)

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        xserver = start('display', ['Xephyr', ':%d' % display, '-auth', str(auth),
            '-screen', '1600x900', '-nolisten', 'tcp', '-noreset', '-s', '0',
            '-title', 'Full 仿真实时录屏'], os.environ.copy())
        limit = time.monotonic()+20
        while subprocess.run(['xwininfo', '-root'], env=env, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, timeout=3).returncode:
            if xserver.poll() is not None or time.monotonic()>limit:
                raise RuntimeError('nested display failed')
            time.sleep(.2)
        dashboard_script = ROOT/'scripts/trajectory_video_dashboard.py'
        shutil.copy2(dashboard_script, out/dashboard_script.name)
        shutil.copy2(Path(__file__), out/Path(__file__).name)
        viewer = start('viewer', ['python3', str(dashboard_script), '--task', str(args.task.resolve()),
            '--plan', str(args.plan.resolve()), '--ros-port', str(args.ros_port),
            '--ready', str(out/'viewer.ready'), '--view-ready', str(out/'rviz.ready'),
            '--finished', str(out/'finished'), '--events', str(out/'video_events.json')], viewer_env)
        limit = time.monotonic()+30
        while not (out/'viewer.ready').exists():
            if viewer.poll() is not None or time.monotonic()>limit:
                raise RuntimeError('video dashboard failed before simulation')
            time.sleep(.2)
        manifest['dashboard_start_monotonic'] = json.loads((out/'viewer.ready').read_text())['start_monotonic']
        recorder = start('screen_recorder', ['ffmpeg', '-hide_banner', '-nostdin',
            '-f', 'x11grab', '-draw_mouse', '0', '-framerate', '15', '-video_size', '1600x900',
            '-i', env['DISPLAY'], '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '20',
            '-threads', '2', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out/'full_process.mp4')])
        manifest['capture_start_monotonic'] = time.monotonic()
        time.sleep(1)
        if recorder.poll() is not None:
            raise RuntimeError('screen recorder failed')
        runner = start('run', ['python3', str(ROOT/'scripts/run_trajectory_gazebo_smoke.py'),
            '--output', str(out/'case'), '--setup', str(args.setup.resolve()),
            '--profile', 'planned_slosh', '--task', str(args.task.resolve()),
            '--plan', str(args.plan.resolve()), '--ros-port', str(args.ros_port),
            '--gazebo-port', str(args.gazebo_port)])
        limit = time.monotonic()+600
        master_seen = False
        capture_finished = False
        while runner.poll() is None:
            live = reachable(args.ros_port)
            master_seen |= live
            if viewer.poll() is not None:
                raise RuntimeError('dashboard exited during recording')
            if not capture_finished and recorder.poll() is not None:
                raise RuntimeError('screen recorder exited during simulation')
            if master_seen and not live and not capture_finished:
                (out/'finished').write_text('simulation ended\n')
                time.sleep(3)
                stop(next(c for c in children if c[0]=='screen_recorder'))
                capture_finished = True
                manifest['capture_end_monotonic'] = time.monotonic()
            if time.monotonic()>limit:
                raise RuntimeError('Full recording timeout')
            time.sleep(.3)
        manifest['run_exit_code'] = runner.returncode
        if not (out/'rviz.ready').exists():
            raise RuntimeError('RViz scene was not initialized')
        if not capture_finished:
            (out/'finished').write_text('run ended\n')
            time.sleep(3)
            stop(next(c for c in children if c[0]=='screen_recorder'))
            manifest['capture_end_monotonic'] = time.monotonic()
        manifest['case_summary'] = json.loads((out/'case/summary.json').read_text())
        probe = subprocess.check_output(['ffprobe', '-v', 'error', '-show_format', '-show_streams',
                                        '-of', 'json', str(out/'full_process.mp4')], text=True)
        manifest['video_probe'] = json.loads(probe)
        manifest['video_sha256'] = hashlib.sha256((out/'full_process.mp4').read_bytes()).hexdigest()
        manifest['recording_complete'] = True
    except Exception as error:
        manifest['error'] = str(error)
        manifest['recording_complete'] = False
        print(str(error), flush=True)
    finally:
        for child in reversed(children):
            stop(child)
        for item, (_, proc, _) in zip(manifest['children'], children):
            item['exit_code'] = proc.returncode
        auth.unlink(missing_ok=True)
        (out/'video_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:manifest[k] for k in ['recording_complete', 'video', 'error'] if k in manifest}), flush=True)
    return 0 if manifest.get('recording_complete') else 1


if __name__ == '__main__':
    raise SystemExit(main())
