#include "spmpc_sim_local_planner/ros/spmpc_sim_local_planner_ros.h"
#include "spmpc_sim_local_planner/ros/sim_node_admission.h"

#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <ros/param.h>

namespace {

std::string environmentText(const char* name) {
    const char* value = std::getenv(name);
    return value == nullptr ? std::string() : std::string(value);
}

template <typename T>
void readPrivateParam(const std::string& node_name,
                      const std::string& key,
                      T& value) {
    // ros::param communicates with the already selected master but does not
    // create this process's NodeHandle, publishers, subscribers, timers, or
    // controller object.  Missing parameters retain their fail-closed default.
    ros::param::get(node_name + "/" + key, value);
}

}  // namespace

// This node is linked to libspmpc_sim_local_planner.so and uses the
// spmpc_sim_local_planner C++/message ABI.  It never links or includes the
// real-robot controller package.
int main(int argc, char** argv) {
    ros::init(argc, argv, "spmpc_sim_local_planner");

    // The Python launch gate is useful for declaring and hashing the complete
    // condition vector, but it must not be the sole boundary.  A direct
    // `rosrun` of this binary would otherwise be able to attach to a real ROS
    // master.  Refuse before constructing the controller or a NodeHandle.
    spmpc_sim_local_planner::SimNodeAdmissionInput admission;
    admission.gate_token = environmentText("SMPCC_SIM_CONTROLLER_GATE_HASH");
    admission.ros_master_uri = environmentText("ROS_MASTER_URI");
    const std::string node_name = ros::this_node::getName();
    // The global /use_sim_time parameter is intentionally read as an absolute
    // path.  Avoid depending on a private namespace default that could mask a
    // non-simulation master.
    ros::param::get("/use_sim_time", admission.use_sim_time);
    readPrivateParam(node_name, "sim_adapter/gate_hash",
                     admission.parameter_gate_token);
    readPrivateParam(node_name, "sim_adapter/target_id", admission.target_id);
    readPrivateParam(node_name, "sim_adapter/gate_id", admission.gate_id);
    // The gate token must be handed off on the fresh environment master that
    // this fork launched.  A real-robot master never publishes this
    // simulation-owned marker, so a direct binary invocation cannot attach
    // merely by reproducing private controller parameters.
    ros::param::get("/smpcc_sim_environment/owner_package",
                    admission.environment_owner_package);
    readPrivateParam(node_name, "sim_adapter/launch_marker",
                     admission.launch_marker);
    readPrivateParam(node_name, "sim_adapter/release_ack",
                     admission.release_ack);

    std::string admission_error;
    if (!spmpc_sim_local_planner::validateSimNodeAdmission(admission,
                                                            admission_error)) {
        std::cerr << "[spmpc_sim_local_planner][FAIL-CLOSED] self-admission "
                  << "refused before controller initialization: "
                  << admission_error << std::endl;
        return 64;
    }

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    spmpc_sim_local_planner::SpmpcLocalPlannerROS planner;
    if (!planner.initialize(nh, pnh)) {
        return 1;
    }
    planner.spin();
    return 0;
}
