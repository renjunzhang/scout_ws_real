#!/usr/bin/env python3
"""Read-only ROS/RViz view for a live trajectory-MPCC screen recording."""
import argparse
import json
from pathlib import Path
import socket
import threading
import time
import xmlrpc.client

from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from rviz import bindings as rviz
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String
from spmpc_local_planner.msg import ControlCycleAudit, ControlCycleWallTiming


TRANSLATIONS = {
    'WAITING_FOR_REFERENCE_PATH': '控制器已启动，等待任务',
    'PREDICTION_DYNAMICS_VIOLATION': '预测一致性检查未通过',
    'B_slosh_ACADOS_OK': 'Full 正在跟踪',
    'B0_ACADOS_OK': '控制器正在跟踪',
    'TERMINAL_DRAINING': '停车保持 / 执行队列清空',
    'TASK_DEADLINE_MISSED': '任务超时，未到点',
    'GOAL_REACHED': '已到点',
}


def describe(status):
    if 'SOLVE_BUDGET_EXHAUSTED' in status:
        return '计算预算不足，进入安全停车'
    if status.startswith('STATE_TIME_ALIGNMENT_FAILED'):
        return '状态时间对齐检查未通过'
    return TRANSLATIONS.get(status, status)


class Dashboard(QtWidgets.QWidget):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.task = json.loads(args.task.read_text())
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.data = dict(status='等待 Gazebo 和定位启动', odom=None, command=(0., 0.),
                         task_start=None, ros_ready=False, attempted=0, accepted=0,
                         first_failure=None, rows=[], last_rti=None, finished=False)
        self.events = []
        self.subscribers = []
        self.trace = RosPath()
        self.rviz_frame = None
        self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
        self.setWindowTitle('Full 仿真全过程')
        self.setFixedSize(1600, 900)
        self.move(0, 0)
        self.setStyleSheet('QWidget {background:#111b2b; color:#e7eef9; font-family:"Noto Sans CJK SC";}'
                           'QLabel {background:transparent;}')
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(22, 14, 22, 14)
        self.title = QtWidgets.QLabel('FULL  /  Gazebo 实时仿真')
        self.title.setStyleSheet('font-size:28px; font-weight:600;')
        root.addWidget(self.title)
        self.clock = QtWidgets.QLabel('正在准备画面')
        self.clock.setStyleSheet('font-size:17px; color:#9fb4ce;')
        root.addWidget(self.clock)
        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)
        self.scene = QtWidgets.QVBoxLayout()
        body.addLayout(self.scene, 1)
        legend = QtWidgets.QLabel('实时 RViz 视图    绿色：参考路线    蓝色：Full 计划    橙色：里程计轨迹')
        legend.setStyleSheet('font-size:16px; padding:6px;')
        self.scene.addWidget(legend)
        self.placeholder = QtWidgets.QLabel('正在启动独立 Gazebo 场景…\n画面就绪后自动显示机器人和地图')
        self.placeholder.setAlignment(QtCore.Qt.AlignCenter)
        self.placeholder.setStyleSheet('font-size:23px; background:#1d2c40;')
        self.scene.addWidget(self.placeholder, 1)
        side = QtWidgets.QVBoxLayout()
        body.addLayout(side)
        self.status = QtWidgets.QLabel('等待仿真启动')
        self.status.setFixedWidth(460)
        self.status.setWordWrap(True)
        self.status.setStyleSheet('font-size:24px; color:#70c6ff; padding:14px; background:#1d2c40;')
        side.addWidget(self.status)
        self.raw_status = QtWidgets.QLabel('')
        self.raw_status.setWordWrap(True)
        self.raw_status.setFixedWidth(460)
        self.raw_status.setStyleSheet('font-size:13px; color:#aabbd1; padding:8px;')
        side.addWidget(self.raw_status)
        self.metrics = QtWidgets.QLabel('等待反馈')
        self.metrics.setStyleSheet('font-size:19px; padding:10px;')
        self.metrics.setFixedWidth(460)
        side.addWidget(self.metrics)
        label = QtWidgets.QLabel('启动时的求解记录（墙钟毫秒）')
        label.setStyleSheet('font-size:18px; padding:8px;')
        side.addWidget(label)
        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(['周期', 'RTI耗时', '剩余预算', '迭代数'])
        self.table.setFixedWidth(460)
        self.table.setFixedHeight(255)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setStyleSheet('QTableWidget {font-size:16px; background:#18263a; gridline-color:#34445b;}'
                                 'QHeaderView::section {font-size:14px; background:#243750; color:#e7eef9; padding:7px;}')
        side.addWidget(self.table)
        self.failure = QtWidgets.QLabel('第一次拒绝 / 停车事件将保留在这里')
        self.failure.setFixedWidth(460)
        self.failure.setWordWrap(True)
        self.failure.setStyleSheet('font-size:17px; padding:12px; color:#ffc985; background:#2b2730;')
        side.addWidget(self.failure)
        side.addStretch()
        self.footer = QtWidgets.QLabel('实时录屏 · 原速播放 · 仅观察仿真 / 不向控制器发送指令')
        self.footer.setStyleSheet('font-size:16px; color:#9fb4ce; padding-top:5px;')
        root.addWidget(self.footer)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.show()
        QtCore.QTimer.singleShot(300, lambda: args.ready.write_text(
            json.dumps(dict(start_monotonic=self.started))+'\n'))
        threading.Thread(target=self.connect_ros, daemon=True).start()

    def connect_ros(self):
        while True:
            with socket.socket() as s:
                s.settimeout(.2)
                if s.connect_ex(('127.0.0.1', self.args.ros_port)) == 0:
                    try:
                        master = xmlrpc.client.ServerProxy('http://127.0.0.1:%d' % self.args.ros_port)
                        response = master.getParam('/full_video_dashboard', '/use_sim_time')
                        if response[0] == 1 and response[2] is True:
                            break
                    except (OSError, xmlrpc.client.Error):
                        pass
            time.sleep(.2)
        rospy.init_node('full_video_dashboard', disable_signals=True)
        self.trace_pub = rospy.Publisher('/full_video/actual_path', RosPath, queue_size=1)
        self.preview_pub = rospy.Publisher('/full_video/reference_path', RosPath, queue_size=1, latch=True)
        self.plan_pub = rospy.Publisher('/full_video/plan', RosPath, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=20),
            rospy.Subscriber('/cmd_vel', Twist, self.on_command, queue_size=20),
            rospy.Subscriber('/spmpc/status', String, self.on_status, queue_size=200),
            rospy.Subscriber('/scout/global_path_fixed', RosPath, self.on_task, queue_size=1),
            rospy.Subscriber('/spmpc/debug/control_cycle_audit', ControlCycleAudit, self.on_audit, queue_size=200),
            rospy.Subscriber('/spmpc/debug/control_cycle_wall_timing', ControlCycleWallTiming, self.on_wall, queue_size=200),
        ]
        with self.lock:
            self.data['ros_ready'] = True
            self.data['status'] = '定位静置 / 等待 Full 任务'

    def on_odom(self, msg):
        with self.lock:
            self.data['odom'] = msg

    def on_command(self, msg):
        with self.lock:
            self.data['command'] = msg.linear.x, msg.angular.z

    def on_task(self, msg):
        with self.lock:
            if self.data['task_start'] is None:
                self.data['task_start'] = msg.header.stamp.to_sec()
                self.events.append(dict(wall_sec=time.monotonic()-self.started,
                                        sim_sec=rospy.Time.now().to_sec(), event='TASK_PUBLISHED'))

    def on_status(self, msg):
        with self.lock:
            if msg.data != self.data['status']:
                self.events.append(dict(wall_sec=time.monotonic()-self.started,
                                        sim_sec=rospy.Time.now().to_sec(), event=msg.data))
            self.data['status'] = msg.data
            if ('VIOLATION' in msg.data or 'BUDGET_EXHAUSTED' in msg.data) and self.data['first_failure'] is None:
                self.data['first_failure'] = (rospy.Time.now().to_sec(), describe(msg.data))

    def on_audit(self, msg):
        with self.lock:
            self.data['attempted'] += int(msg.solve_attempted)
            self.data['accepted'] += int(msg.solve_attempted and msg.solve_success and msg.command_accepted)

    def on_wall(self, msg):
        if not msg.solve_attempted:
            return
        with self.lock:
            row = (msg.cycle_id, msg.rti_wall_ms, msg.remaining_budget_wall_ms, msg.rti_iterations)
            if len(self.data['rows']) < 6:
                self.data['rows'].append(row)
            elif 'BUDGET_EXHAUSTED' in msg.status:
                self.data['rows'][-1] = row
            self.data['last_rti'] = msg

    def make_path(self, points, frame):
        path = RosPath()
        path.header.frame_id = frame
        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.position.z = .04
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        return path

    def initialize_rviz(self):
        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.setSplashPath('')
        self.rviz_frame.initialize()
        self.rviz_frame.setMenuBar(None)
        self.rviz_frame.setStatusBar(None)
        self.rviz_frame.setHideButtonVisibility(False)
        # Hide configuration docks; the recording focuses on the actual view.
        for dock in self.rviz_frame.findChildren(QtWidgets.QDockWidget):
            dock.hide()
        for toolbar in self.rviz_frame.findChildren(QtWidgets.QToolBar):
            toolbar.hide()
        self.rviz_frame.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        manager = self.rviz_frame.getManager()
        manager.setFixedFrame(self.task['frame_id'])
        root = manager.getRootDisplayGroup()
        root.subProp('Global Options').subProp('Frame Rate').setValue(15)
        map_display = manager.createDisplay('rviz/Map', 'Map', True)
        map_display.subProp('Topic').setValue('/map')
        map_display.subProp('Alpha').setValue(.75)
        manager.createDisplay('rviz/RobotModel', 'Robot', True)
        for topic, name, color, width in [
                ('/full_video/reference_path', 'Reference', QtGui.QColor('#36c96f'), .055),
                ('/full_video/plan', 'Full plan', QtGui.QColor('#36a8ff'), .03),
                ('/full_video/actual_path', 'Actual odometry', QtGui.QColor('#ff8f40'), .06),
                ('/spmpc/local_trajectory', 'MPC horizon', QtGui.QColor('#ed5cce'), .035)]:
            display = manager.createDisplay('rviz/Path', name, True)
            display.subProp('Topic').setValue(topic)
            display.subProp('Color').setValue(color)
            display.subProp('Line Style').setValue('Billboards')
            display.subProp('Line Width').setValue(width)
        views = manager.getViewManager()
        views.setCurrentViewControllerType('rviz/TopDownOrtho')
        view = views.getCurrent()
        route = self.task['route']
        view.subProp('X').setValue((min(p[0] for p in route)+max(p[0] for p in route))/2)
        view.subProp('Y').setValue(0.)
        view.subProp('Scale').setValue(130.)
        view.subProp('Angle').setValue(0.)
        self.scene.replaceWidget(self.placeholder, self.rviz_frame)
        self.placeholder.hide()
        self.preview_pub.publish(self.make_path(route, self.task['frame_id']))
        plan = json.loads(self.args.plan.read_text())
        self.plan_pub.publish(self.make_path([(s['state'][0], s['state'][1]) for s in plan['samples']], self.task['frame_id']))
        self.args.view_ready.write_text('rviz ready\n')

    def refresh(self):
        with self.lock:
            data = dict(self.data)
            data['rows'] = list(self.data['rows'])
            events = list(self.events)
        wall = time.monotonic()-self.started
        sim = rospy.Time.now().to_sec() if data['ros_ready'] else 0.
        elapsed = sim-data['task_start'] if data['task_start'] is not None else None
        self.clock.setText('录屏时间 %.1f s    |    %s    |    任务期限 %.0f s' % (
            wall, '任务尚未下发' if elapsed is None else '任务时间 %.2f s（仿真）' % elapsed,
            self.task['deadline']))
        if data['ros_ready'] and data['odom'] is not None and self.rviz_frame is None:
            self.initialize_rviz()
        status = data['status']
        if self.args.finished.exists():
            self.status.setText('本次仿真结束')
            self.footer.setText('全过程录制结束 · 最后状态：'+describe(status))
        else:
            self.status.setText(describe(status))
        self.raw_status.setText(status)
        v, w = data['command']
        odom = data['odom']
        self.metrics.setText('已下发速度   %.4f m/s\n已下发角速度 %.4f rad/s\n实际反馈速度 %s\n\n尝试求解 %d 次\n成功接纳 %d 次' % (
            v, w, '--' if odom is None else '%.4f m/s' % odom.twist.twist.linear.x,
            data['attempted'], data['accepted']))
        if odom is not None and self.rviz_frame is not None and not rospy.is_shutdown():
            self.trace.header = odom.header
            if not self.trace.poses or odom.header.stamp != self.trace.poses[-1].header.stamp:
                pose = PoseStamped()
                pose.header, pose.pose = odom.header, odom.pose.pose
                pose.pose.position.z = .05
                self.trace.poses.append(pose)
                self.trace_pub.publish(self.trace)
        if self.table.rowCount() != len(data['rows']):
            self.table.setRowCount(len(data['rows']))
        for i, row in enumerate(data['rows']):
            for j, value in enumerate(row):
                item = QtWidgets.QTableWidgetItem(str(value) if j in (0, 3) else '%.2f' % value)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.table.setItem(i, j, item)
        if data['first_failure']:
            stamp, cause = data['first_failure']
            self.failure.setText('首次拒绝：任务 %.3f s\n%s' % (stamp-(data['task_start'] or stamp), cause))
        if data['last_rti'] is not None and 'BUDGET_EXHAUSTED' in data['last_rti'].status:
            m = data['last_rti']
            self.failure.setText(self.failure.text()+'\n\n停车周期 %d\n预计迭代 %.2f ms > 剩余 %.2f ms' % (
                m.cycle_id, m.iteration_estimate_wall_ms, m.remaining_budget_wall_ms))
        self.args.events.write_text(json.dumps(events, ensure_ascii=False, indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['task', 'plan', 'ready', 'view-ready', 'finished', 'events']:
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--ros-port', type=int, default=11892)
    args = p.parse_args()
    app = QtWidgets.QApplication([])
    dashboard = Dashboard(args)
    app.exec_()


if __name__ == '__main__':
    main()
