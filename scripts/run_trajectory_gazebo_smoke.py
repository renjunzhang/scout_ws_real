#!/usr/bin/env python3
"""Current ROS1 controller integration smoke in the existing isolated Gazebo.

This is development evidence, not an R8/formal comparison. An optional vehicle
actuator adapter matches the controller's delay/response parameters. Source the
freshly built ROS1 overlay before running.
The external simulator files are read only; only owned child processes stop.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import time
import xmlrpc.client
from simulation_observation import ObservationWindow

import rospy
import roslib.packages
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

REPO = Path(__file__).resolve().parents[1]
SIM = Path('/data/a/scout_sim_replacement')
PKG = REPO / 'src/scout_apps/control/spmpc_local_planner'
MAP = SIM / 'maps/proxy_world_manual_saved_20260611_154348.pbstream'
TOPICS = [
    '/clock', '/map', '/tf', '/tf_static', '/odom', '/imu/data', '/scan_front',
    '/scout/global_path_fixed', '/cmd_vel', '/cmd_vel_drive', '/spmpc/status',
    '/spmpc/local_trajectory', '/spmpc/solver_time_ms', '/spmpc/cost_breakdown',
    '/spmpc/slosh_height', '/spmpc/slosh_horizon_summary',
    '/spmpc/terminal/debug', '/spmpc/terminal/mode',
    '/spmpc/debug/effective_config', '/spmpc/debug/planning_config',
    '/spmpc/debug/pre_solve_snapshot', '/spmpc/debug/predicted_horizon',
    '/spmpc/debug/control_cycle_audit', '/spmpc/debug/command_intervention',
    '/spmpc/debug/control_cycle_wall_timing',
    '/spmpc/debug/raw_state', '/spmpc/debug/predicted_state',
    '/spmpc/debug/solver_input_state', '/spmpc/debug/slosh_state',
    '/spmpc/debug/slosh_observer_selection', '/spmpc/debug/slosh_observer_odom',
    '/spmpc/debug/slosh_observer_imu', '/spmpc/debug/slosh_cost_monitor',
]


def reachable(port):
    with socket.socket() as stream:
        stream.settimeout(.3)
        return stream.connect_ex(('127.0.0.1', port)) == 0


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--setup', type=Path, required=True)
    parser.add_argument('--profile', choices=['raw_mpcc', 'planned_slosh'], default='raw_mpcc')
    parser.add_argument('--plan', type=Path)
    parser.add_argument('--task', type=Path)
    parser.add_argument('--reference-mode', choices=['progress', 'fixed_time'],
                        help='override this case only; preserve the profile default when omitted')
    parser.add_argument('--identified-actuator', action='store_true',
                        help='replace this case\'s guard with the controller\'s actuator response')
    parser.add_argument('--ros-port', type=int, default=11892)
    parser.add_argument('--gazebo-port', type=int, default=11926)
    args = parser.parse_args()
    if not args.setup.is_file() or not MAP.is_file():
        parser.error('missing compiled ROS1 overlay or explicit map')
    if args.profile == 'planned_slosh' and not (args.plan and args.task):
        parser.error('planned_slosh requires a validated --plan and matching --task')
    if args.reference_mode and not args.plan:
        parser.error('--reference-mode requires a --plan')
    if args.ros_port == args.gazebo_port:
        parser.error('ROS and Gazebo need different ports')
    binary = args.setup.resolve().parent/'lib/spmpc_local_planner/spmpc_local_planner_node'
    if not binary.is_file():
        parser.error('the requested overlay has no compiled current planner')
    resolved_nodes = roslib.packages.find_node('spmpc_local_planner', 'spmpc_local_planner_node') or []
    if len(resolved_nodes) != 1 or Path(resolved_nodes[0]).resolve() != binary.resolve():
        parser.error('source the requested --setup before running; roslaunch would select a different planner')
    linked = subprocess.check_output(['ldd', str(binary)], text=True)
    expected_library = args.setup.resolve().parent/'lib/libspmpc_local_planner.so'
    if not any(line.split()[:3] == ['libspmpc_local_planner.so', '=>', str(expected_library)]
               for line in linked.splitlines()):
        parser.error('LD_LIBRARY_PATH does not select the requested planner library; source --setup first')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    summary = dict(formal=False, evidence='CURRENT_MAINLINE_GAZEBO_INTEGRATION',
                   profile=args.profile, actuator_plant_matches_controller=False,
                   plant='Existing direct planar drive + acceleration guard; no FOPDT delay',
                   controller='Production fixed 5/10-step FIFO + identified actuator response',
                   map_file=str(MAP), map_sha256=hashlib.sha256(MAP.read_bytes()).hexdigest(),
                   runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   binary_file=str(binary), binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                   git_sha=subprocess.check_output(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], text=True).strip(),
                   goal_reached=False, children=[])
    (out/'source.diff').write_bytes(subprocess.check_output(['git', '-C', str(REPO), 'diff', 'HEAD']))
    (out/'runner.py').write_bytes(Path(__file__).read_bytes())
    observation_module = Path(__file__).with_name('simulation_observation.py')
    shutil.copy2(observation_module, out/observation_module.name)
    summary['observation_module_sha256'] = hashlib.sha256(observation_module.read_bytes()).hexdigest()
    if args.identified_actuator:
        adapter = REPO/'scripts/gazebo_identified_actuator.py'
        shutil.copy2(adapter, out/adapter.name)
        shutil.copy2(PKG/'config/planner/common.yaml', out/'actuator_config.yaml')
        summary.update(plant='Identified FOPDT adapter -> existing planar drive (120 Hz)',
                       actuator_adapter_sha256=hashlib.sha256(adapter.read_bytes()).hexdigest(),
                       actuator_plant_matches_controller=False,
                       actuator_model_parameters_match=False,
                       actuator_match_limit='ROS receipt timing and 120 Hz output require bag verification')
    # The ROS node is a thin executable; its hash alone does not identify the
    # controller implementation or generated model loaded for this case.
    runtime_dir = out/'runtime'
    runtime_dir.mkdir()
    summary['runtime'] = {}
    for source in [binary, args.setup.resolve().parent/'lib/libspmpc_local_planner.so'] + [
            PKG/'generated/acados'/model/('libacados_ocp_solver_'+model+'.so')
            for model in ('spmpc_b0', 'spmpc_slosh')]:
        shutil.copy2(source, runtime_dir/source.name)
        summary['runtime'][source.name] = dict(
            source_path=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    children = []
    records = []
    status_counts = Counter()
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.ros_port,
               GAZEBO_MASTER_URI='http://127.0.0.1:%d' % args.gazebo_port,
               SCOUT_PROXY_FULL_ROS_MASTER_URI='http://127.0.0.1:%d' % args.ros_port,
               SCOUT_PROXY_FULL_GAZEBO_MASTER_URI='http://127.0.0.1:%d' % args.gazebo_port,
               SCOUT_WS_SETUP=str(args.setup.resolve()), MAP_FILE=str(MAP),
               USE_RVIZ='false', GAZEBO_GUI='false', TRACKING_RVIZ='false',
               START_PATH_PUBLISHER='false', START_SPMPC='false',
               LOG_DIR=str(out/'environment'), ROS_LOG_DIR=str(out/'ros'),
               SMPCC_SIMULATOR_SEED='716')
    os.environ.update({k: env[k] for k in ('ROS_MASTER_URI', 'GAZEBO_MASTER_URI')})

    def start(name, command):
        log = (out/(name+'.log')).open('w')
        proc = subprocess.Popen(list(map(str, command)), env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, proc, log))
        summary['children'].append(dict(name=name, pid=proc.pid, command=list(map(str, command))))
        return proc

    def stop(child):
        name, proc, log = child
        if proc.poll() is None:
            # rosbag needs SIGINT to close its index. Environment launcher traps
            # TERM and stops its own tracked roslaunch/roscore children.
            proc.send_signal(signal.SIGINT if name == 'recorder' else signal.SIGTERM)
            try:
                proc.wait(timeout=35)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
        log.close()

    def on_signal(signum, _frame):
        raise RuntimeError('interrupted by signal %d' % signum)

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    try:
        summary['pre_ports'] = {str(p): reachable(p) for p in (args.ros_port, args.gazebo_port)}
        if any(summary['pre_ports'].values()):
            raise RuntimeError('case requires empty ROS/Gazebo ports')
        environment = start('environment_launcher', [SIM/'scripts/launch_proxy_sim_localization_env.sh'])
        deadline = time.monotonic()+90
        while not reachable(args.ros_port):
            if time.monotonic() > deadline or environment.poll() is not None:
                raise RuntimeError('ROS master startup failed')
            time.sleep(.2)
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        while True:
            response = master.getParam('/current_mainline_smoke', '/use_sim_time')
            if response[0] == 1 and response[2] is True:
                break
            if time.monotonic() > deadline or environment.poll() is not None:
                raise RuntimeError('simulated ROS time was not enabled')
            time.sleep(.2)
        rospy.init_node('current_mainline_smoke', disable_signals=True)
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        for topic, kind in [('/odom', Odometry), ('/scan_front', LaserScan),
                            ('/imu/data', Imu), ('/map', OccupancyGrid)]:
            rospy.wait_for_message(topic, kind, timeout=90)
        buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(15))
        actuator = None
        if args.identified_actuator:
            # Only this fresh environment's tracked child guard is stopped.
            # No simulator source/launch/world is modified and no competing
            # publisher may remain on the drive topic.
            subprocess.run(['rosnode', 'kill', '/cmd_vel_guard'], env=env,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True, timeout=10)
            deadline = time.monotonic()+10
            while dict(master.getSystemState('/current_mainline_smoke')[2][0]).get('/cmd_vel_drive'):
                if time.monotonic() > deadline:
                    raise RuntimeError('old drive publisher did not stop')
                time.sleep(.1)
            actuator = start('actuator_plant', ['python3', out/'gazebo_identified_actuator.py',
                                             '--config', out/'actuator_config.yaml'])
            rospy.wait_for_message('/cmd_vel_drive', Twist, timeout=10)
        print('Environment ready; settling for 30 seconds', flush=True)
        time.sleep(30)
        pose = buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2)).transform
        summary['initial_pose'] = [pose.translation.x, pose.translation.y, yaw(pose.rotation)]
        odom = rospy.wait_for_message('/odom', Odometry, timeout=5)
        if odom.child_frame_id != 'base_footprint':
            raise RuntimeError('unexpected isolated simulator odometry child frame')
        summary['odom_child_frame'] = odom.child_frame_id
        if args.task:
            task = json.loads(args.task.read_text())
            path = RosPath()
            path.header.frame_id = task['frame_id']
            for x, y in task['route']:
                point = PoseStamped()
                point.header.frame_id = task['frame_id']
                point.pose.position.x, point.pose.position.y = x, y
                point.pose.orientation.w = 1.
                path.poses.append(point)
            path.poses[-1].pose.orientation.z = math.sin(task['goal_pose'][2]/2)
            path.poses[-1].pose.orientation.w = math.cos(task['goal_pose'][2]/2)
        else:
            start('path_publisher', ['python3', SIM/'classic_ws/src/scout_mini_proxy_nav_adapter/scripts/proxy_fixed_path_publisher.py',
                  '_frame_id:=map', '_goal_x:=5.0', '_goal_y:=0.0', '_goal_yaw:=0.0',
                  '_template:=s_curve', '_start_heading:=current', '_amplitude_ratio:=0.18',
                  '_min_amplitude:=0.25', '_max_amplitude:=1.20', '_side:=left', '_smooth_iterations:=3'])
            path = rospy.wait_for_message('/scout/global_path_fixed', RosPath, timeout=20)
            stop(children.pop())
            task = json.loads((REPO/'test/native/scenarios/corner.json').read_text())
            route = [[p.pose.position.x, p.pose.position.y] for p in path.poses]
            length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(route, route[1:]))
            # Cartographer's odom origin need not equal Gazebo's world origin.
            # Pair stationary model/world pose with map TF explicitly.
            models = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
            world_pose = models.pose[models.name.index('scout_mini_proxy_stable')]
            angle = yaw(pose.rotation)-yaw(world_pose.orientation)
            tx = pose.translation.x-math.cos(angle)*world_pose.position.x+math.sin(angle)*world_pose.position.y
            ty = pose.translation.y-math.sin(angle)*world_pose.position.x-math.cos(angle)*world_pose.position.y
            vertices = [[tx+math.cos(angle)*x-math.sin(angle)*y,
                         ty+math.sin(angle)*x+math.cos(angle)*y]
                        for x, y in [(-5.8,-3.8),(5.8,-3.8),(5.8,3.8),(-5.8,3.8)]]
            summary['world_to_map_at_rest'] = [tx, ty, angle]
            task.update(task_id='gazebo_s_curve_development', frame_id='map', route=route,
                        goal_pose=[5.,0.,0.], deadline=45., transport_duration=40., stop_window=5.)
            task['start_state'] = [0.]*28
            task['start_state'][:3] = summary['initial_pose']
            task['region'] = dict(id='proxy_open_room', frame_id='map', footprint_radius=.45,
                                  margin=.02, cells=[dict(id='room', s_begin=0., s_end=length, vertices=vertices)])
        (out/'task.json').write_text(json.dumps(task, indent=2)+'\n')
        summary['task_sha256'] = hashlib.sha256((out/'task.json').read_bytes()).hexdigest()
        if args.plan:
            summary['plan_file'] = str(args.plan.resolve())
            summary['plan_sha256'] = hashlib.sha256(args.plan.read_bytes()).hexdigest()
        region = dict(planning=dict(region=dict(task['region'], enabled=True)))
        (out/'region.yaml').write_text(yaml.safe_dump(region))
        (out/'task.yaml').write_text(yaml.safe_dump(dict(planning=dict(task_deadline_sec=task['deadline'], evaluation_window_sec=5.))))
        overlay = dict(frames=dict(reference_target='map', robot_base='base_footprint'),
                       delay_phase=dict(mode='off'))
        if args.reference_mode:
            overlay['planning'] = dict(reference=dict(mode=args.reference_mode))
        (out/'overlay.yaml').write_text(yaml.safe_dump(overlay))
        publisher = rospy.Publisher('/scout/global_path_fixed', RosPath, queue_size=1, latch=True)
        # Explicit subscriptions exist before the planner advertises diagnostics;
        # rosbag -a discovers new topics too late to capture startup failures.
        start('recorder', ['rosbag', 'record', '-O', out/'run.bag', '__name:=current_mainline_recorder'] + TOPICS)
        deadline = time.monotonic()+15
        while True:
            subscriptions = dict(master.getSystemState('/current_mainline_smoke')[2][1])
            if all('/current_mainline_recorder' in subscriptions.get(topic, []) for topic in TOPICS):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('recorder subscriptions did not register before motion')
            time.sleep(.2)
        time.sleep(1)
        def on_status(msg):
            status_counts[msg.data] += 1
            records.append([rospy.Time.now().to_sec(), msg.data])
        status_sub = rospy.Subscriber('/spmpc/status', String, on_status, queue_size=200)
        command = ['roslaunch', 'spmpc_local_planner', 'trajectory_mpcc.launch',
                   'profile:='+args.profile, 'region_config:='+str(out/'region.yaml'),
                   'task_overlay_file:='+str(out/'task.yaml'), 'planner_overlay_file:='+str(out/'overlay.yaml')]
        if args.plan:
            command.append('plan_file:='+str(args.plan.resolve()))
        planner = start('planner', command)
        time.sleep(2)
        (out/'live_params.yaml').write_text(yaml.safe_dump(rospy.get_param('/spmpc_local_planner', {})))
        if args.identified_actuator:
            configured = yaml.safe_load((out/'actuator_config.yaml').read_text())['execution_model']
            live_actuator = rospy.get_param('/spmpc_local_planner/execution_model')
            if any(live_actuator.get(key) != value for key, value in configured.items()):
                raise RuntimeError('plant/controller actuator parameters differ')
            summary['actuator_model_parameters_match'] = True
            publishers = dict(master.getSystemState('/current_mainline_smoke')[2][0])
            if publishers.get('/cmd_vel_drive') != ['/identified_actuator_plant']:
                raise RuntimeError('drive must have exactly the owned actuator publisher')
        path.header.stamp = rospy.Time.now()
        publisher.publish(path)
        begin = rospy.Time.now().to_sec()
        window = ObservationWindow(begin, task['deadline'], task['stop_window'])
        summary['observation_target_sim_sec'] = window.duration_sec
        wall_begin = time.monotonic()
        print('Recording and running '+args.profile, flush=True)
        while not window.complete(rospy.Time.now().to_sec()) and time.monotonic()-wall_begin < 180:
            if planner.poll() is not None:
                raise RuntimeError('planner exited before observation completed')
            if actuator is not None and actuator.poll() is not None:
                raise RuntimeError('actuator plant exited during observation')
            if 'GOAL_REACHED' in status_counts and not summary['goal_reached']:
                summary['goal_reached'] = True
                summary['goal_time_sec'] = next(t for t, status in records if status == 'GOAL_REACHED')-begin
            time.sleep(.2)
        summary['observation_sim_window_complete'] = window.complete(rospy.Time.now().to_sec())
        summary['observation_sim_sec'] = rospy.Time.now().to_sec()-begin
        summary['observation_wall_sec'] = time.monotonic()-wall_begin
        final = buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2)).transform
        summary['final_pose'] = [final.translation.x, final.translation.y, yaw(final.rotation)]
    except Exception as error:
        summary['error'] = str(error)
        print('Case error: '+str(error), flush=True)
    finally:
        summary['status_counts'] = dict(status_counts)
        (out/'statuses.json').write_text(json.dumps(records, indent=2)+'\n')
        for child in reversed(children):
            stop(child)
        if children:
            time.sleep(30)
        summary['post_ports'] = {str(p): reachable(p) for p in (args.ros_port, args.gazebo_port)}
        summary['valid_fresh_lifecycle'] = not any(summary.get('pre_ports', {}).values()) and not any(summary['post_ports'].values())
        summary['closed_bag'] = (out/'run.bag').is_file() and not (out/'run.bag.active').exists()
        summary['passed'] = bool(summary['goal_reached'] and summary['valid_fresh_lifecycle'] and summary['closed_bag'] and summary.get('observation_sim_window_complete', False) and 'error' not in summary)
        (out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
