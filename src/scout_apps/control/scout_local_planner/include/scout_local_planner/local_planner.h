#ifndef SCOUT_LOCAL_PLANNER_LOCAL_PLANNER_H
#define SCOUT_LOCAL_PLANNER_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <slosh_models/liquid_slosh_model.h>
#include <slosh_models/mpc_vel_tracker.h>

#include <memory>
#include <vector>

namespace scout_local_planner {

/**
 * @brief Scout机器人MPC局部规划器
 *
 * 功能:
 * - 接收全局路径，生成局部速度指令
 * - 集成液体晃动抑制软约束
 * - 支持实时避障（待扩展）
 */
class LocalPlanner {
public:
  LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~LocalPlanner() = default;

  /**
   * @brief 初始化规划器
   * @return true 成功, false 失败
   */
  bool initialize();

  /**
   * @brief 设置全局路径
   * @param path 全局路径
   */
  void setGlobalPath(const nav_msgs::Path& path);

  /**
   * @brief 计算速度指令
   * @param cmd_vel 输出速度指令
   * @return true 成功, false 失败或到达目标
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief 检查是否到达目标
   */
  bool isGoalReached() const;

  /**
   * @brief 获取当前液面晃动高度
   */
  double getCurrentSloshHeight() const;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 订阅和发布
  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher local_path_pub_;
  ros::Publisher slosh_height_pub_;

  // MPC控制器
  std::shared_ptr<slosh_models::LiquidSloshModel> slosh_model_;
  std::unique_ptr<slosh_models::MPCVelTracker> mpc_tracker_;

  // 状态
  nav_msgs::Path global_path_;
  nav_msgs::Odometry current_odom_;
  bool has_odom_ = false;
  bool has_path_ = false;
  int current_waypoint_idx_ = 0;

  // 参数
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;
  double goal_tolerance_;
  double lookahead_distance_;
  double control_rate_;

  // 回调
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);

  // 辅助函数
  bool getRobotPose(geometry_msgs::PoseStamped& pose);
  int findClosestWaypoint();
  std::vector<Eigen::Vector2d> generateReferenceVelocities(int horizon);
};

}  // namespace scout_local_planner

#endif  // SCOUT_LOCAL_PLANNER_LOCAL_PLANNER_H
