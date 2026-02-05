/**
 * @file simple_global_planner_node.cpp
 * @brief Simple A* global planner for /scout/global_path
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace scout_global_planner {

class SimpleGlobalPlanner {
public:
    SimpleGlobalPlanner()
        : nh_(),
          pnh_("~"),
          tf_listener_(tf_buffer_) {
        loadParams();

        map_sub_ = nh_.subscribe(map_topic_, 1, &SimpleGlobalPlanner::mapCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic_, 1, &SimpleGlobalPlanner::goalCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);

        if (planner_frequency_ > 1e-3) {
            timer_ = nh_.createTimer(ros::Duration(1.0 / planner_frequency_),
                                     &SimpleGlobalPlanner::timerCallback, this);
        }
    }

private:
    struct OpenNode {
        int idx = -1;
        double f = 0.0;
        double g = 0.0;
    };

    struct CompareOpen {
        bool operator()(const OpenNode& a, const OpenNode& b) const {
            return a.f > b.f;  // min-heap
        }
    };

    struct Point2D {
        double x = 0.0;
        double y = 0.0;
    };

    void loadParams() {
        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));
        pnh_.param("map_topic", map_topic_, std::string("map"));
        pnh_.param("goal_topic", goal_topic_, std::string("goal"));
        pnh_.param("path_topic", path_topic_, std::string("global_path"));
        pnh_.param("planner_frequency", planner_frequency_, 1.0);
        pnh_.param("allow_unknown", allow_unknown_, false);
        pnh_.param("occ_threshold", occ_threshold_, 65);
        pnh_.param("inflation_radius", inflation_radius_, 0.0);
        pnh_.param("shortcut_smoothing", shortcut_smoothing_, true);
        pnh_.param("smooth_path", smooth_path_, true);
        pnh_.param("smooth_weight_data", smooth_weight_data_, 0.2);
        pnh_.param("smooth_weight_smooth", smooth_weight_smooth_, 0.4);
        pnh_.param("smooth_tolerance", smooth_tolerance_, 1e-4);
        pnh_.param("smooth_max_iterations", smooth_max_iterations_, 200);
        pnh_.param("path_resolution", path_resolution_, 0.1);  // 路径点最大间隔(米)
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        map_ = *msg;
        has_map_ = true;
        updateInflatedMap();
        need_replan_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_ = *msg;
        has_goal_ = true;
        need_replan_ = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!has_map_ || !has_goal_) {
            return;
        }
        planAndPublish();
    }

    bool getStartPose(geometry_msgs::PoseStamped& start_map) {
        try {
            geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
                map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
            start_map.header.stamp = ros::Time::now();
            start_map.header.frame_id = map_frame_;
            start_map.pose.position.x = tf.transform.translation.x;
            start_map.pose.position.y = tf.transform.translation.y;
            start_map.pose.position.z = 0.0;
            start_map.pose.orientation = tf.transform.rotation;
            return true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "[SimpleGlobalPlanner] TF error: %s", ex.what());
            return false;
        }
    }

    bool transformGoalToMap(geometry_msgs::PoseStamped& goal_map) {
        if (goal_.header.frame_id == map_frame_ || goal_.header.frame_id.empty()) {
            goal_map = goal_;
            goal_map.header.frame_id = map_frame_;
            return true;
        }
        try {
            goal_map = tf_buffer_.transform(goal_, map_frame_, ros::Duration(0.1));
            return true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "[SimpleGlobalPlanner] Goal TF error: %s", ex.what());
            return false;
        }
    }

    bool worldToMap(double wx, double wy, int& mx, int& my) const {
        const double res = map_.info.resolution;
        if (res <= 0.0) {
            return false;
        }
        tf2::Transform origin_tf;
        tf2::fromMsg(map_.info.origin, origin_tf);
        tf2::Transform origin_inv = origin_tf.inverse();

        tf2::Vector3 p_map(wx, wy, 0.0);
        tf2::Vector3 p_grid = origin_inv * p_map;

        mx = static_cast<int>(std::floor(p_grid.x() / res));
        my = static_cast<int>(std::floor(p_grid.y() / res));

        if (mx < 0 || my < 0 ||
            mx >= static_cast<int>(map_.info.width) ||
            my >= static_cast<int>(map_.info.height)) {
            return false;
        }
        return true;
    }

    void mapToWorld(int mx, int my, double& wx, double& wy) const {
        const double res = map_.info.resolution;
        tf2::Transform origin_tf;
        tf2::fromMsg(map_.info.origin, origin_tf);

        const double gx = (static_cast<double>(mx) + 0.5) * res;
        const double gy = (static_cast<double>(my) + 0.5) * res;
        tf2::Vector3 p_grid(gx, gy, 0.0);
        tf2::Vector3 p_map = origin_tf * p_grid;
        wx = p_map.x();
        wy = p_map.y();
    }

    bool isOccupiedRaw(int mx, int my) const {
        const int idx = my * static_cast<int>(map_.info.width) + mx;
        const int8_t occ = map_.data[static_cast<size_t>(idx)];
        if (occ < 0) {
            return !allow_unknown_;
        }
        return occ >= occ_threshold_;
    }

    bool isOccupied(int mx, int my) const {
        const int idx = my * static_cast<int>(map_.info.width) + mx;
        if (!inflated_occ_.empty() && idx >= 0 &&
            static_cast<size_t>(idx) < inflated_occ_.size()) {
            return inflated_occ_[static_cast<size_t>(idx)] != 0;
        }
        return isOccupiedRaw(mx, my);
    }

    void computeInflationOffsets(double radius_cells, int r) {
        inflation_offsets_.clear();
        const double r2 = radius_cells * radius_cells + 1e-6;
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (dx * dx + dy * dy <= r2) {
                    inflation_offsets_.push_back(std::make_pair(dx, dy));
                }
            }
        }
    }

    void updateInflatedMap() {
        if (map_.data.empty()) {
            inflated_occ_.clear();
            return;
        }
        if (inflation_radius_ <= 1e-6) {
            inflated_occ_.clear();
            return;
        }
        const int width = static_cast<int>(map_.info.width);
        const int height = static_cast<int>(map_.info.height);
        const double res = map_.info.resolution;
        if (res <= 0.0 || width <= 0 || height <= 0) {
            inflated_occ_.clear();
            return;
        }

        const double radius_cells = inflation_radius_ / res;
        const int r = std::max(0, static_cast<int>(std::ceil(radius_cells)));
        if (r != inflation_radius_cells_ || std::abs(res - inflation_resolution_) > 1e-9) {
            computeInflationOffsets(radius_cells, r);
            inflation_radius_cells_ = r;
            inflation_resolution_ = res;
        }

        inflated_occ_.assign(static_cast<size_t>(width * height), 0);
        for (int idx = 0; idx < width * height; ++idx) {
            const int x = idx % width;
            const int y = idx / width;
            if (!isOccupiedRaw(x, y)) {
                continue;
            }
            for (const auto& off : inflation_offsets_) {
                const int nx = x + off.first;
                const int ny = y + off.second;
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    continue;
                }
                const int nidx = ny * width + nx;
                inflated_occ_[static_cast<size_t>(nidx)] = 1;
            }
        }
    }

    bool isLineFree(int x0, int y0, int x1, int y1) const {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;
        while (true) {
            if (isOccupied(x, y)) {
                return false;
            }
            if (x == x1 && y == y1) {
                break;
            }
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
        return true;
    }

    bool isLineFreeWorld(double wx0, double wy0, double wx1, double wy1) const {
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (!worldToMap(wx0, wy0, x0, y0) || !worldToMap(wx1, wy1, x1, y1)) {
            return false;
        }
        return isLineFree(x0, y0, x1, y1);
    }

    bool isPointFreeWorld(double wx, double wy) const {
        int mx = 0, my = 0;
        if (!worldToMap(wx, wy, mx, my)) {
            return false;
        }
        return !isOccupied(mx, my);
    }

    std::vector<int> shortcutPath(const std::vector<int>& path) const {
        if (path.size() < 3) {
            return path;
        }
        const int width = static_cast<int>(map_.info.width);
        std::vector<int> out;
        size_t i = 0;
        while (i < path.size() - 1) {
            out.push_back(path[i]);
            size_t best = i + 1;
            const int x0 = path[i] % width;
            const int y0 = path[i] / width;
            for (size_t j = path.size() - 1; j > i + 1; --j) {
                const int x1 = path[j] % width;
                const int y1 = path[j] / width;
                if (isLineFree(x0, y0, x1, y1)) {
                    best = j;
                    break;
                }
            }
            i = best;
        }
        out.push_back(path.back());
        return out;
    }

    /**
     * @brief 对路径进行重采样，确保相邻点间距不超过 path_resolution_
     */
    std::vector<Point2D> resamplePath(const std::vector<Point2D>& path) const {
        if (path.size() < 2 || path_resolution_ <= 0.0) {
            return path;
        }
        std::vector<Point2D> resampled;
        resampled.push_back(path.front());

        for (size_t i = 1; i < path.size(); ++i) {
            const Point2D& prev = resampled.back();
            const Point2D& curr = path[i];
            const double dx = curr.x - prev.x;
            const double dy = curr.y - prev.y;
            const double dist = std::hypot(dx, dy);

            if (dist > path_resolution_) {
                // 需要插值
                const int n_segments = static_cast<int>(std::ceil(dist / path_resolution_));
                for (int j = 1; j < n_segments; ++j) {
                    const double t = static_cast<double>(j) / n_segments;
                    resampled.push_back(Point2D{prev.x + t * dx, prev.y + t * dy});
                }
            }
            resampled.push_back(curr);
        }
        return resampled;
    }

    std::vector<Point2D> smoothPath(const std::vector<Point2D>& path) const {
        if (path.size() < 3) {
            return path;
        }
        std::vector<Point2D> new_path = path;
        for (int iter = 0; iter < smooth_max_iterations_; ++iter) {
            double change = 0.0;
            for (size_t i = 1; i + 1 < new_path.size(); ++i) {
                Point2D prev = new_path[i];
                new_path[i].x += smooth_weight_data_ * (path[i].x - new_path[i].x) +
                                 smooth_weight_smooth_ * (new_path[i - 1].x + new_path[i + 1].x - 2.0 * new_path[i].x);
                new_path[i].y += smooth_weight_data_ * (path[i].y - new_path[i].y) +
                                 smooth_weight_smooth_ * (new_path[i - 1].y + new_path[i + 1].y - 2.0 * new_path[i].y);

                if (!isPointFreeWorld(new_path[i].x, new_path[i].y) ||
                    !isLineFreeWorld(new_path[i - 1].x, new_path[i - 1].y,
                                     new_path[i].x, new_path[i].y) ||
                    !isLineFreeWorld(new_path[i].x, new_path[i].y,
                                     new_path[i + 1].x, new_path[i + 1].y)) {
                    new_path[i] = prev;
                    continue;
                }
                change += std::hypot(new_path[i].x - prev.x, new_path[i].y - prev.y);
            }
            if (change < smooth_tolerance_) {
                break;
            }
        }
        return new_path;
    }

    bool planAStar(const geometry_msgs::PoseStamped& start_map,
                   const geometry_msgs::PoseStamped& goal_map,
                   nav_msgs::Path& path_out) {
        if (map_.data.empty()) {
            return false;
        }
        int sx = 0, sy = 0, gx = 0, gy = 0;
        if (!worldToMap(start_map.pose.position.x, start_map.pose.position.y, sx, sy)) {
            ROS_WARN_THROTTLE(1.0, "[SimpleGlobalPlanner] Start out of map");
            return false;
        }
        if (!worldToMap(goal_map.pose.position.x, goal_map.pose.position.y, gx, gy)) {
            ROS_WARN_THROTTLE(1.0, "[SimpleGlobalPlanner] Goal out of map");
            return false;
        }
        if (isOccupied(sx, sy) || isOccupied(gx, gy)) {
            ROS_WARN_THROTTLE(1.0, "[SimpleGlobalPlanner] Start or goal occupied");
            return false;
        }

        const int width = static_cast<int>(map_.info.width);
        const int height = static_cast<int>(map_.info.height);
        const int start_idx = sy * width + sx;
        const int goal_idx = gy * width + gx;

        const int total = width * height;
        std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
        std::vector<int> parent(total, -1);
        std::vector<bool> closed(total, false);

        auto heuristic = [&](int ix, int iy) {
            const double dx = static_cast<double>(ix - gx);
            const double dy = static_cast<double>(iy - gy);
            return std::hypot(dx, dy);
        };

        std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpen> open;
        g_score[start_idx] = 0.0;
        open.push(OpenNode{start_idx, heuristic(sx, sy), 0.0});

        const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        const double costs[8] = {1.0, 1.0, 1.0, 1.0,
                                 std::sqrt(2.0), std::sqrt(2.0),
                                 std::sqrt(2.0), std::sqrt(2.0)};

        bool found = false;
        while (!open.empty()) {
            OpenNode cur = open.top();
            open.pop();
            if (closed[cur.idx]) {
                continue;
            }
            closed[cur.idx] = true;
            if (cur.idx == goal_idx) {
                found = true;
                break;
            }

            const int cx = cur.idx % width;
            const int cy = cur.idx / width;

            for (int i = 0; i < 8; ++i) {
                const int nx = cx + dxs[i];
                const int ny = cy + dys[i];
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    continue;
                }
                if (isOccupied(nx, ny)) {
                    continue;
                }
                const int nidx = ny * width + nx;
                const double tentative = g_score[cur.idx] + costs[i];
                if (tentative < g_score[nidx]) {
                    g_score[nidx] = tentative;
                    parent[nidx] = cur.idx;
                    const double f = tentative + heuristic(nx, ny);
                    open.push(OpenNode{nidx, f, tentative});
                }
            }
        }

        if (!found) {
            ROS_WARN_THROTTLE(1.0, "[SimpleGlobalPlanner] No path found");
            return false;
        }

        std::vector<int> path_indices;
        for (int idx = goal_idx; idx != -1; idx = parent[idx]) {
            path_indices.push_back(idx);
        }
        std::reverse(path_indices.begin(), path_indices.end());

        if (shortcut_smoothing_) {
            path_indices = shortcutPath(path_indices);
        }

        std::vector<Point2D> points;
        points.reserve(path_indices.size());
        for (int idx : path_indices) {
            const int mx = idx % width;
            const int my = idx / width;
            double wx = 0.0;
            double wy = 0.0;
            mapToWorld(mx, my, wx, wy);
            points.push_back(Point2D{wx, wy});
        }

        if (smooth_path_) {
            points = smoothPath(points);
        }

        // 对路径重采样，确保点间距不超过 path_resolution_
        points = resamplePath(points);

        path_out.header.stamp = ros::Time::now();
        path_out.header.frame_id = map_frame_;
        path_out.poses.clear();
        path_out.poses.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_out.header;
            pose.pose.position.x = points[i].x;
            pose.pose.position.y = points[i].y;
            pose.pose.position.z = 0.0;

            double yaw = 0.0;
            if (i + 1 < points.size()) {
                yaw = std::atan2(points[i + 1].y - points[i].y,
                                 points[i + 1].x - points[i].x);
            } else if (i > 0) {
                const geometry_msgs::PoseStamped& prev = path_out.poses.back();
                yaw = tf2::getYaw(prev.pose.orientation);
            }

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);

            path_out.poses.push_back(pose);
        }

        return true;
    }

    void planAndPublish() {
        geometry_msgs::PoseStamped start_map;
        geometry_msgs::PoseStamped goal_map;
        if (!getStartPose(start_map) || !transformGoalToMap(goal_map)) {
            return;
        }

        nav_msgs::Path path;
        if (planAStar(start_map, goal_map, path)) {
            path_pub_.publish(path);
            need_replan_ = false;
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber map_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Timer timer_;

    nav_msgs::OccupancyGrid map_;
    geometry_msgs::PoseStamped goal_;

    bool has_map_ = false;
    bool has_goal_ = false;
    bool need_replan_ = false;

    std::string map_frame_;
    std::string base_frame_;
    std::string map_topic_;
    std::string goal_topic_;
    std::string path_topic_;
    double planner_frequency_ = 1.0;
    bool allow_unknown_ = false;
    int occ_threshold_ = 65;
    bool shortcut_smoothing_ = true;
    bool smooth_path_ = true;
    double smooth_weight_data_ = 0.2;
    double smooth_weight_smooth_ = 0.4;
    double smooth_tolerance_ = 1e-4;
    int smooth_max_iterations_ = 200;
    double inflation_radius_ = 0.0;
    double inflation_resolution_ = 0.0;
    int inflation_radius_cells_ = 0;
    double path_resolution_ = 0.1;  // 路径点最大间隔(米)
    std::vector<uint8_t> inflated_occ_;
    std::vector<std::pair<int, int>> inflation_offsets_;
};

}  // namespace scout_global_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_global_planner");
    scout_global_planner::SimpleGlobalPlanner node;
    ros::spin();
    return 0;
}
