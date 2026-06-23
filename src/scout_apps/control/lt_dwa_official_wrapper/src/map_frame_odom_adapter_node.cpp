#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace {

void LoadStringParam(const ros::NodeHandle& nh,
                     const std::string& name,
                     std::string* value) {
  nh.param<std::string>(name, *value, *value);
}

class MapFrameOdomAdapter {
 public:
  MapFrameOdomAdapter(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    LoadConfig();
    odom_sub_ = nh_.subscribe(input_odom_topic_, 10, &MapFrameOdomAdapter::OdomCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 10);
  }

 private:
  void LoadConfig() {
    LoadStringParam(private_nh_, "input_odom_topic", &input_odom_topic_);
    LoadStringParam(private_nh_, "output_odom_topic", &output_odom_topic_);
    LoadStringParam(private_nh_, "target_frame", &target_frame_);
    LoadStringParam(private_nh_, "base_frame", &base_frame_);
    private_nh_.param("lookup_timeout_sec", lookup_timeout_sec_, lookup_timeout_sec_);
    private_nh_.param("use_latest_tf", use_latest_tf_, use_latest_tf_);
  }

  bool LookupRobotTransform(const nav_msgs::Odometry& odom, tf::StampedTransform* transform) {
    std::string base_frame = base_frame_;
    if (base_frame.empty()) {
      base_frame = odom.child_frame_id;
    }
    if (base_frame.empty()) {
      ROS_WARN_THROTTLE(1.0, "map-frame odom adapter has no base frame");
      return false;
    }

    const ros::Time lookup_stamp = use_latest_tf_ || odom.header.stamp.isZero()
                                       ? ros::Time(0)
                                       : odom.header.stamp;
    try {
      tf_listener_.waitForTransform(target_frame_,
                                    base_frame,
                                    lookup_stamp,
                                    ros::Duration(lookup_timeout_sec_));
      tf_listener_.lookupTransform(target_frame_, base_frame, lookup_stamp, *transform);
      return true;
    } catch (const tf::TransformException& ex) {
      if (!odom.child_frame_id.empty() && odom.child_frame_id != base_frame) {
        try {
          tf_listener_.waitForTransform(target_frame_,
                                        odom.child_frame_id,
                                        ros::Time(0),
                                        ros::Duration(lookup_timeout_sec_));
          tf_listener_.lookupTransform(target_frame_, odom.child_frame_id, ros::Time(0), *transform);
          return true;
        } catch (const tf::TransformException& fallback_ex) {
          ROS_WARN_THROTTLE(1.0,
                            "map-frame odom adapter TF lookup failed: %s; fallback failed: %s",
                            ex.what(),
                            fallback_ex.what());
          return false;
        }
      }
      ROS_WARN_THROTTLE(1.0, "map-frame odom adapter TF lookup failed: %s", ex.what());
      return false;
    }
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    tf::StampedTransform transform;
    if (!LookupRobotTransform(*msg, &transform)) {
      return;
    }

    nav_msgs::Odometry out = *msg;
    out.header.frame_id = target_frame_;
    out.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    out.child_frame_id = transform.child_frame_id_;
    out.pose.pose.position.x = transform.getOrigin().x();
    out.pose.pose.position.y = transform.getOrigin().y();
    out.pose.pose.position.z = transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation(), out.pose.pose.orientation);
    odom_pub_.publish(out);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;

  std::string input_odom_topic_{"/odom"};
  std::string output_odom_topic_{"/baseline/official_lt_dwa/odom_map"};
  std::string target_frame_{"map"};
  std::string base_frame_{"base_link"};
  double lookup_timeout_sec_{0.05};
  bool use_latest_tf_{true};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "lt_dwa_map_frame_odom_adapter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  MapFrameOdomAdapter node(nh, private_nh);
  ros::spin();
  return 0;
}
