/**
 * @file mbf_path_publisher_node.cpp
 * @brief MBF GetPath action client that publishes nav_msgs/Path to /scout/global_path
 */

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <mbf_msgs/GetPathAction.h>

namespace scout_global_planner {

class MbfPathPublisher {
public:
    MbfPathPublisher()
        : nh_(),
          pnh_("~"),
          tf_listener_(tf_buffer_),
          action_client_(resolveActionName(), true) {
        loadParams();

        path_pub_ = nh_.advertise<nav_msgs::Path>(global_path_topic_, 1, true);
        goal_sub_ = nh_.subscribe(goal_topic_, 1, &MbfPathPublisher::goalCallback, this);

        if (replan_frequency_ > 1e-3) {
            timer_ = nh_.createTimer(ros::Duration(1.0 / replan_frequency_),
                                     &MbfPathPublisher::timerCallback, this);
        }

        if (!action_client_.waitForServer(ros::Duration(2.0))) {
            ROS_WARN("[MBFPathPublisher] GetPath action server not ready: %s",
                     action_name_.c_str());
        }
    }

private:
    void loadParams() {
        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("goal_topic", goal_topic_, std::string("goal"));
        pnh_.param("global_path_topic", global_path_topic_, std::string("global_path"));
        pnh_.param("action_name", action_name_, std::string("mbf_costmap_nav/get_path"));
        pnh_.param("planner_name", planner_name_, std::string(""));
        pnh_.param("tolerance", tolerance_, 0.0);
        pnh_.param("replan_frequency", replan_frequency_, 0.0);
        pnh_.param("cancel_before_replan", cancel_before_replan_, true);
    }

    std::string resolveActionName() const {
        std::string name;
        ros::NodeHandle pnh("~");
        pnh.param("action_name", name, std::string("mbf_costmap_nav/get_path"));
        return ros::names::resolve(name);
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_ = *msg;
        has_goal_ = true;
        requestPlan();
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!has_goal_) {
            return;
        }
        requestPlan();
    }

    bool transformGoalToMap(const geometry_msgs::PoseStamped& in,
                            geometry_msgs::PoseStamped& out) {
        if (in.header.frame_id.empty() || in.header.frame_id == map_frame_) {
            out = in;
            out.header.frame_id = map_frame_;
            return true;
        }
        try {
            out = tf_buffer_.transform(in, map_frame_, ros::Duration(0.2));
            return true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "[MBFPathPublisher] Goal TF error: %s", ex.what());
            return false;
        }
    }

    void requestPlan() {
        if (!action_client_.isServerConnected()) {
            if (!action_client_.waitForServer(ros::Duration(0.5))) {
                ROS_WARN_THROTTLE(1.0, "[MBFPathPublisher] GetPath action server not available");
                return;
            }
        }

        if (cancel_before_replan_ && action_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
            action_client_.cancelAllGoals();
        }

        geometry_msgs::PoseStamped goal_map;
        if (!transformGoalToMap(goal_, goal_map)) {
            return;
        }

        mbf_msgs::GetPathGoal goal;
        goal.use_start_pose = false;
        goal.target_pose = goal_map;
        goal.tolerance = tolerance_;
        goal.planner = planner_name_;
        goal.concurrency_slot = 0;

        action_client_.sendGoal(goal,
                                boost::bind(&MbfPathPublisher::doneCb, this, _1, _2));
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const mbf_msgs::GetPathResultConstPtr& result) {
        if (!result) {
            ROS_WARN_THROTTLE(1.0, "[MBFPathPublisher] GetPath result is null");
            return;
        }
        if (result->outcome != mbf_msgs::GetPathResult::SUCCESS) {
            ROS_WARN_THROTTLE(1.0, "[MBFPathPublisher] GetPath failed: %u %s",
                              result->outcome, result->message.c_str());
            return;
        }
        if (result->path.poses.empty()) {
            ROS_WARN_THROTTLE(1.0, "[MBFPathPublisher] GetPath returned empty path");
            return;
        }

        nav_msgs::Path path = result->path;
        path.header.stamp = ros::Time::now();
        if (path.header.frame_id.empty()) {
            path.header.frame_id = map_frame_;
        }
        path_pub_.publish(path);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Timer timer_;

    actionlib::SimpleActionClient<mbf_msgs::GetPathAction> action_client_;

    geometry_msgs::PoseStamped goal_;
    bool has_goal_ = false;

    std::string map_frame_;
    std::string goal_topic_;
    std::string global_path_topic_;
    std::string action_name_;
    std::string planner_name_;
    double tolerance_ = 0.0;
    double replan_frequency_ = 0.0;
    bool cancel_before_replan_ = true;
};

}  // namespace scout_global_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "mbf_path_publisher");
    scout_global_planner::MbfPathPublisher node;
    ros::spin();
    return 0;
}
