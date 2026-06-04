#!/usr/bin/env python3

import copy

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


FORBIDDEN_TOPIC_EXACT = {
    "/odom",
    "/cmd_vel",
    "/tf",
    "/tf_static",
}
FORBIDDEN_TOPIC_PREFIX = (
    "/spmpc/",
    "/scout/global_path",
)
FORBIDDEN_TF_FRAMES = {
    "map",
    "odom",
    "base_link",
    "base_footprint",
}


class UnsafeConfiguration(RuntimeError):
    pass


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def normalized_topic(name):
    return rospy.resolve_name(name).rstrip("/") or "/"


class MocapPoseMonitor:
    def __init__(self):
        self.tracker = rospy.get_param("~tracker", "Scout")
        self.input_pose_topic = rospy.get_param(
            "~input_pose_topic", "/vrpn_client_node/{}/pose".format(self.tracker)
        )
        self.pose_topic = rospy.get_param("~pose_topic", "/mocap/scout_pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/mocap/scout_odom")
        self.path_topic = rospy.get_param("~path_topic", "/mocap/scout_path")
        self.status_topic = rospy.get_param("~status_topic", "/mocap/status")

        self.publish_pose = as_bool(rospy.get_param("~publish_pose", True))
        self.publish_odom = as_bool(rospy.get_param("~publish_odom", True))
        self.publish_path = as_bool(rospy.get_param("~publish_path", True))
        self.publish_tf = as_bool(rospy.get_param("~publish_tf", False))

        self.output_frame_id = rospy.get_param("~output_frame_id", "mocap_world")
        self.mocap_world_frame = rospy.get_param("~mocap_world_frame", "mocap_world")
        self.mocap_body_frame = rospy.get_param("~mocap_body_frame", "mocap_scout")
        self.path_max_length = max(1, int(rospy.get_param("~path_max_length", 5000)))
        self.stale_timeout_sec = max(0.0, float(rospy.get_param("~stale_timeout_sec", 1.0)))
        self.status_publish_hz = max(0.1, float(rospy.get_param("~status_publish_hz", 1.0)))

        self._validate_isolation()

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10) if self.publish_pose else None
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10) if self.publish_odom else None
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True) if self.publish_path else None
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.output_frame_id
        self.last_stamp = None
        self.last_wall_time = None
        self.message_count = 0
        self.dropped_nonmonotonic = 0

        self.sub = rospy.Subscriber(self.input_pose_topic, PoseStamped, self.pose_callback, queue_size=50)
        self.status_timer = rospy.Timer(rospy.Duration(1.0 / self.status_publish_hz), self.publish_status)

        rospy.loginfo(
            "[nokov_mocap_monitor] tracker=%s input=%s outputs=(%s,%s,%s,%s) publish_tf=%s",
            self.tracker,
            self.input_pose_topic,
            self.pose_topic if self.publish_pose else "disabled",
            self.odom_topic if self.publish_odom else "disabled",
            self.path_topic if self.publish_path else "disabled",
            self.status_topic,
            self.publish_tf,
        )

    def _validate_isolation(self):
        output_topics = []
        if self.publish_pose:
            output_topics.append(("pose_topic", self.pose_topic))
        if self.publish_odom:
            output_topics.append(("odom_topic", self.odom_topic))
        if self.publish_path:
            output_topics.append(("path_topic", self.path_topic))
        output_topics.append(("status_topic", self.status_topic))

        for param_name, topic in output_topics:
            resolved = normalized_topic(topic)
            if resolved in FORBIDDEN_TOPIC_EXACT or any(resolved.startswith(prefix) for prefix in FORBIDDEN_TOPIC_PREFIX):
                rospy.logfatal(
                    "[nokov_mocap_monitor] refusing unsafe %s=%s; mocap monitor must publish only isolated monitoring topics",
                    param_name,
                    resolved,
                )
                raise UnsafeConfiguration("unsafe Nokov monitor topic configuration")

        if self.output_frame_id in ("base_link", "base_footprint"):
            rospy.logfatal(
                "[nokov_mocap_monitor] refusing output_frame_id=%s; do not publish mocap messages in robot base frames",
                self.output_frame_id,
            )
            raise UnsafeConfiguration("unsafe Nokov monitor frame configuration")

        if self.output_frame_id in ("map", "odom"):
            rospy.logwarn(
                "[nokov_mocap_monitor] output_frame_id=%s is a control-stack frame name. "
                "This is allowed only for monitoring messages; no control TF will be published.",
                self.output_frame_id,
            )

        if self.publish_tf:
            for param_name, frame_id in (
                ("mocap_world_frame", self.mocap_world_frame),
                ("mocap_body_frame", self.mocap_body_frame),
            ):
                if frame_id in FORBIDDEN_TF_FRAMES:
                    rospy.logfatal(
                        "[nokov_mocap_monitor] refusing unsafe %s=%s with publish_tf=true; "
                        "allowed TF is monitoring-only, e.g. mocap_world -> mocap_scout",
                        param_name,
                        frame_id,
                    )
                    raise rospy.ROSInitException("unsafe Nokov monitor TF configuration")

    def normalized_pose(self, msg):
        pose_msg = copy.deepcopy(msg)
        if pose_msg.header.stamp == rospy.Time(0):
            pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.output_frame_id or msg.header.frame_id or "mocap_world"
        return pose_msg

    def pose_callback(self, msg):
        pose_msg = self.normalized_pose(msg)
        if self.last_stamp is not None and pose_msg.header.stamp < self.last_stamp:
            self.dropped_nonmonotonic += 1
            rospy.logwarn_throttle(
                2.0,
                "[nokov_mocap_monitor] dropping non-monotonic pose stamp; dropped=%d",
                self.dropped_nonmonotonic,
            )
            return

        self.last_stamp = pose_msg.header.stamp
        self.last_wall_time = rospy.Time.now()
        self.message_count += 1

        if self.pose_pub is not None:
            self.pose_pub.publish(pose_msg)
        if self.odom_pub is not None:
            self.odom_pub.publish(self.make_odom(pose_msg))
        if self.path_pub is not None:
            self.publish_path_msg(pose_msg)
        if self.tf_broadcaster is not None:
            self.publish_isolated_tf(pose_msg)

    def make_odom(self, pose_msg):
        odom = Odometry()
        odom.header = pose_msg.header
        odom.child_frame_id = self.mocap_body_frame
        odom.pose.pose = pose_msg.pose
        return odom

    def publish_path_msg(self, pose_msg):
        path_pose = copy.deepcopy(pose_msg)
        self.path_msg.header.stamp = pose_msg.header.stamp
        self.path_msg.header.frame_id = pose_msg.header.frame_id
        self.path_msg.poses.append(path_pose)
        if len(self.path_msg.poses) > self.path_max_length:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_length:]
        self.path_pub.publish(self.path_msg)

    def publish_isolated_tf(self, pose_msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = pose_msg.header.stamp
        tf_msg.header.frame_id = self.mocap_world_frame
        tf_msg.child_frame_id = self.mocap_body_frame
        tf_msg.transform.translation.x = pose_msg.pose.position.x
        tf_msg.transform.translation.y = pose_msg.pose.position.y
        tf_msg.transform.translation.z = pose_msg.pose.position.z
        tf_msg.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_status(self, _event):
        now = rospy.Time.now()
        if self.last_wall_time is None:
            status = "WAITING tracker={} input={} count=0".format(self.tracker, self.input_pose_topic)
        else:
            age = (now - self.last_wall_time).to_sec()
            state = "STALE" if age > self.stale_timeout_sec else "OK"
            status = (
                "{} tracker={} input={} count={} age_sec={:.3f} frame={} dropped_nonmonotonic={}"
                .format(
                    state,
                    self.tracker,
                    self.input_pose_topic,
                    self.message_count,
                    age,
                    self.output_frame_id,
                    self.dropped_nonmonotonic,
                )
            )
        self.status_pub.publish(String(data=status))


if __name__ == "__main__":
    rospy.init_node("mocap_pose_monitor")
    try:
        MocapPoseMonitor()
    except UnsafeConfiguration as ex:
        rospy.logfatal("[nokov_mocap_monitor] %s", ex)
        raise SystemExit(1)
    rospy.spin()
