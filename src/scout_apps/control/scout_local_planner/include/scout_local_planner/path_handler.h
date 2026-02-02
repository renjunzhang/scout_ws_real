/**
 * @file path_handler.h
 * @brief 路径处理器
 * 
 * 负责：
 * 1. 接收全局路径
 * 2. TF 变换（map → base_link）
 * 3. 局部三次样条拟合
 * 4. 计算 Frenet 误差、曲率、切向角
 */

#pragma once

#include "scout_local_planner/types.h"
#include "scout_local_planner/cubic_spline.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>

#include <vector>
#include <memory>
#include <mutex>

namespace scout_local_planner {

class PathHandler {
public:
    PathHandler();
    
    /**
     * @brief 设置参数
     */
    void setParams(const PathHandlerParams& params);
    
    /**
     * @brief 设置 TF buffer
     */
    void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);
    
    /**
     * @brief 更新全局路径
     * @param path 全局路径（frame_id 应为 "map"）
     * @param v_des 期望速度（用于一次性生成 v(s)）
     * @return 是否成功
     */
    bool updateGlobalPath(const nav_msgs::Path& path, double v_des);
    
    /**
     * @brief 更新机器人位姿
     * @param pose 当前位姿（base_link 在 map 中）
     * @param v 当前线速度
     * @param omega 当前角速度
     */
    void updateRobotState(const geometry_msgs::PoseStamped& pose,
                          double v, double omega);
    
    /**
     * @brief 获取 MPC 所需的参考点序列
     * @param N 预测步数
     * @param dt 时间步长
     * @param v_des 期望速度
     * @param ref_points 输出：参考点序列
     * @return 是否成功
     */
    bool getReferencePoints(int N, double dt, double v_des,
                            std::vector<ReferencePoint>& ref_points);
    
    /**
     * @brief 计算当前 Frenet 误差
     * @param frenet 输出：Frenet 误差状态
     * @return 是否成功
     */
    bool getFrenetState(FrenetState& frenet);
    
    /**
     * @brief 检查是否到达目标
     */
    bool isGoalReached() const;
    
    /**
     * @brief 检查路径是否有效
     */
    bool isPathValid() const;

    /**
     * @brief 消费 reset 提示（路径跳变/重规划）
     */
    bool consumeResetHint();
    
    /**
     * @brief 获取路径上最近点的弧长
     */
    double getCurrentArcLength() const { return current_s_; }

    /**
     * @brief 获取全局路径上的弧长进度（从起点累计）
     */
    double getGlobalProgress() const;
    
    /**
     * @brief 获取局部样条的总长度
     */
    double getSplineTotalLength() const;

    /**
     * @brief 获取平滑后的路径（基于局部样条，base_link 坐标系）
     * @param path_out 输出路径
     * @param num_samples 采样点数
     * @return 是否成功
     */
    bool getSmoothedPath(nav_msgs::Path& path_out, int num_samples) const;

private:
    /**
     * @brief 将路径从 map 变换到 base_link
     */
    bool transformPathToBaseLink(const nav_msgs::Path& path_in,
                                 std::vector<Eigen::Vector2d>& points_out);
    
    /**
     * @brief 找到路径上距离机器人最近的点
     * @return 最近点的索引
     */
    int findClosestPointIndex(const std::vector<Eigen::Vector2d>& points,
                              const Eigen::Vector2d& robot_pos) const;
    
    /**
     * @brief 计算点到样条的投影（Frenet 坐标）
     */
    void computeFrenetProjection(const Eigen::Vector2d& point,
                                 double robot_theta,
                                 double& e_l, double& e_c, double& e_theta);
    
    /**
     * @brief 对窗口内的点进行样条拟合
     */
    bool fitLocalSpline(const std::vector<Eigen::Vector2d>& points,
                        int start_idx, int end_idx);

    /**
     * @brief 对给定窗口点拟合样条
     */
    bool fitLocalSpline(const std::vector<Eigen::Vector2d>& window_points);

    /**
     * @brief 更新路径速度曲线 v(s)
     */
    void updateSpeedProfile(double v_des);

    /**
     * @brief 将点投影到路径线段上，返回连续弧长 s
     */
    double projectToPathS(const Eigen::Vector2d& point,
                          int closest_idx,
                          const std::vector<Eigen::Vector2d>& points,
                          const std::vector<double>& path_s) const;

    /**
     * @brief 查询弧长 s 处的参考速度
     */
    double getSpeedAtS(double s) const;

private:
    PathHandlerParams params_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string base_frame_ = "base_link";  // 可配置的 base frame
    
    // 全局路径（map 坐标系）
    nav_msgs::Path global_path_;
    ros::Time path_timestamp_;
    bool has_path_ = false;

    // 全局路径缓存（map 坐标系）
    std::vector<Eigen::Vector2d> global_points_map_;
    std::vector<double> global_path_s_;
    bool global_cache_valid_ = false;
    bool reset_hint_ = false;
    
    // 机器人状态
    geometry_msgs::PoseStamped robot_pose_;
    double robot_v_ = 0.0;
    double robot_omega_ = 0.0;
    bool has_robot_state_ = false;
    
    // 局部样条
    CubicSpline2D local_spline_;
    double current_s_ = 0.0;  // 当前在样条上的弧长位置
    int closest_idx_ = 0;     // 最近点索引

    // 全局样条（用于一次性速度曲线）
    CubicSpline2D global_spline_;
    double global_spline_length_ = 0.0;

    // 速度曲线（全局 v(s)）
    std::vector<double> speed_profile_s_;
    std::vector<double> speed_profile_v_;
    double speed_profile_v_des_ = 0.0;
    bool speed_profile_valid_ = false;

    // 全局弧长（用于平滑推进）
    double s_global_ = 0.0;
    double last_projection_s_ = 0.0;
    bool s_initialized_ = false;
    
    // 线程安全
    mutable std::mutex mutex_;
};

}  // namespace scout_local_planner
