#include "slosh_models/liquid_slosh_model.h"

#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

class SloshMonitorNode {
public:
  SloshMonitorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
      : nh_(std::move(nh)), pnh_(std::move(pnh)) {
    loadParams();

    if (!model_.configure(model_params_)) {
      throw std::runtime_error("failed to configure LiquidSloshModel");
    }
    resetState();

    height_pub_ = nh_.advertise<std_msgs::Float32>("height", 10);
    state_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("state", 10);
    debug_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug", 10);
    reset_srv_ = nh_.advertiseService("reset", &SloshMonitorNode::resetCb, this);

    odom_sub_ = nh_.subscribe(odom_topic_, 50, &SloshMonitorNode::odomCb, this);
    if (subscribe_cmd_vel_debug_) {
      cmd_sub_ = nh_.subscribe(cmd_vel_topic_, 50, &SloshMonitorNode::cmdCb, this);
    }

    ROS_INFO_STREAM("[slosh_monitor] started. odom_topic=" << odom_topic_
                    << ", cmd_vel_topic=" << cmd_vel_topic_
                    << ", cmd_vel_debug=" << (subscribe_cmd_vel_debug_ ? "true" : "false")
                    << ", height_unit=m");
  }

private:
  void loadParams() {
    pnh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh_.param("subscribe_cmd_vel_debug", subscribe_cmd_vel_debug_, true);

    pnh_.param("container_radius", model_params_.R, 0.0185);
    pnh_.param("liquid_height", model_params_.h, 0.058);
    pnh_.param("liquid_density", model_params_.rho, 1000.0);
    pnh_.param("model_dt", model_params_.dt, 0.02);
    pnh_.param("mode_index", model_params_.mode_index, 1);
    pnh_.param("damping_ratio", model_params_.zeta, 0.05);
    pnh_.param("offset_x", model_params_.r_x, 0.0);
    pnh_.param("offset_y", model_params_.r_y, 0.0);
    pnh_.param("gravity", model_params_.g, 9.81);
    pnh_.param("use_linear_model", model_params_.use_linear_model, true);
    pnh_.param("use_parabola_term", model_params_.use_parabola_term, false);

    pnh_.param("accel_filter_alpha", accel_filter_alpha_, 0.3);
    pnh_.param("min_dt", min_dt_, 0.001);
    pnh_.param("max_dt", max_dt_, 0.1);
    accel_filter_alpha_ = std::max(0.0, std::min(1.0, accel_filter_alpha_));
  }

  void resetState() {
    model_.reset();
    have_prev_odom_ = false;
    prev_stamp_ = ros::Time(0);
    prev_v_ = 0.0;
    prev_omega_ = 0.0;
    ax_filt_ = 0.0;
    ay_filt_ = 0.0;
    alpha_filt_ = 0.0;
    last_dt_ = 0.0;
    last_v_ = 0.0;
    last_omega_ = 0.0;
    last_height_m_ = 0.0;
    last_cmd_v_ = 0.0;
    last_cmd_omega_ = 0.0;
    update_count_ = 0;
    episode_start_stamp_ = ros::Time::now();
  }

  bool resetCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    ++reset_count_;
    resetState();
    publish(ros::Time::now());
    ROS_INFO("[slosh_monitor] reset");
    return true;
  }

  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_v_ = msg->linear.x;
    last_cmd_omega_ = msg->angular.z;
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const double v = msg->twist.twist.linear.x;
    const double omega = msg->twist.twist.angular.z;

    last_v_ = v;
    last_omega_ = omega;

    if (!std::isfinite(v) || !std::isfinite(omega)) {
      ROS_WARN_THROTTLE(1.0, "[slosh_monitor] ignore non-finite odom twist");
      publish(stamp);
      return;
    }

    if (!have_prev_odom_) {
      have_prev_odom_ = true;
      prev_stamp_ = stamp;
      prev_v_ = v;
      prev_omega_ = omega;
      publish(stamp);
      return;
    }

    const double dt = (stamp - prev_stamp_).toSec();
    last_dt_ = dt;
    if (dt < min_dt_ || dt > max_dt_) {
      ROS_WARN_THROTTLE(1.0, "[slosh_monitor] skip update for dt=%.4f", dt);
      prev_stamp_ = stamp;
      prev_v_ = v;
      prev_omega_ = omega;
      publish(stamp);
      return;
    }

    const double dt_tolerance = std::max(0.002, 0.1 * model_params_.dt);
    if (std::fabs(dt - model_params_.dt) > dt_tolerance) {
      ROS_WARN_THROTTLE(1.0,
                        "[slosh_monitor] odom dt=%.4f differs from fixed model_dt=%.4f",
                        dt,
                        model_params_.dt);
    }

    const double ax_raw = (v - prev_v_) / dt;
    const double alpha_raw = (omega - prev_omega_) / dt;
    const double ay_raw = v * omega;

    ax_filt_ = filter(ax_raw, ax_filt_);
    ay_filt_ = filter(ay_raw, ay_filt_);
    alpha_filt_ = filter(alpha_raw, alpha_filt_);

    if (std::isfinite(ax_filt_) && std::isfinite(ay_filt_) && std::isfinite(alpha_filt_)) {
      model_.update(Eigen::Vector2d(ax_filt_, ay_filt_), omega, alpha_filt_);
      ++update_count_;
    } else {
      ROS_WARN_THROTTLE(1.0, "[slosh_monitor] skip non-finite acceleration estimate");
    }

    prev_stamp_ = stamp;
    prev_v_ = v;
    prev_omega_ = omega;
    publish(stamp);
  }

  double filter(double raw, double prev) const {
    return accel_filter_alpha_ * raw + (1.0 - accel_filter_alpha_) * prev;
  }

  void publish(const ros::Time& stamp) {
    const double height_m = std::max(0.0, model_.getSloshHeight());
    const double height_mm = 1000.0 * height_m;
    last_height_m_ = height_m;

    std_msgs::Float32 height_msg;
    height_msg.data = static_cast<float>(height_m);
    height_pub_.publish(height_msg);

    const Eigen::Vector4d state = model_.getState();
    std_msgs::Float32MultiArray state_msg;
    state_msg.layout.dim.resize(1);
    state_msg.layout.dim[0].label = "x_m,vx_mps,y_m,vy_mps";
    state_msg.layout.dim[0].size = 4;
    state_msg.layout.dim[0].stride = 4;
    state_msg.data = {
        static_cast<float>(state(0)),
        static_cast<float>(state(1)),
        static_cast<float>(state(2)),
        static_cast<float>(state(3)),
    };
    state_pub_.publish(state_msg);

    std_msgs::Float32MultiArray debug_msg;
    debug_msg.layout.dim.resize(1);
    debug_msg.layout.dim[0].label =
        "stamp_rel_sec,dt,v_odom,omega_odom,ax_est,ay_est,alpha_est,height_m,height_mm,cmd_v,cmd_omega,update_count,reset_count";
    debug_msg.layout.dim[0].size = 13;
    debug_msg.layout.dim[0].stride = 13;
    debug_msg.data = {
        static_cast<float>(std::max(0.0, (stamp - episode_start_stamp_).toSec())),
        static_cast<float>(last_dt_),
        static_cast<float>(last_v_),
        static_cast<float>(last_omega_),
        static_cast<float>(ax_filt_),
        static_cast<float>(ay_filt_),
        static_cast<float>(alpha_filt_),
        static_cast<float>(last_height_m_),
        static_cast<float>(height_mm),
        static_cast<float>(last_cmd_v_),
        static_cast<float>(last_cmd_omega_),
        static_cast<float>(update_count_),
        static_cast<float>(reset_count_),
    };
    debug_pub_.publish(debug_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher height_pub_;
  ros::Publisher state_pub_;
  ros::Publisher debug_pub_;
  ros::ServiceServer reset_srv_;

  slosh_models::LiquidSloshModel model_;
  slosh_models::LiquidSloshModel::Params model_params_;

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  bool subscribe_cmd_vel_debug_ = true;
  double accel_filter_alpha_ = 0.3;
  double min_dt_ = 0.001;
  double max_dt_ = 0.5;

  bool have_prev_odom_ = false;
  ros::Time prev_stamp_;
  ros::Time episode_start_stamp_;
  double prev_v_ = 0.0;
  double prev_omega_ = 0.0;
  double ax_filt_ = 0.0;
  double ay_filt_ = 0.0;
  double alpha_filt_ = 0.0;
  double last_dt_ = 0.0;
  double last_v_ = 0.0;
  double last_omega_ = 0.0;
  double last_cmd_v_ = 0.0;
  double last_cmd_omega_ = 0.0;
  double last_height_m_ = 0.0;
  int update_count_ = 0;
  int reset_count_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "slosh_monitor_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    SloshMonitorNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("[slosh_monitor] " << e.what());
    return 1;
  }

  return 0;
}
