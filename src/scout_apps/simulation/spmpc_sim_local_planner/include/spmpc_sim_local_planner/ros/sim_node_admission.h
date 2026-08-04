#pragma once

#include <string>

namespace spmpc_sim_local_planner {

// These constants intentionally belong to the simulation fork.  The real
// controller neither includes this header nor observes these parameters.
constexpr const char* kSimNodeAdmissionTargetId =
    "SMPCC_SIM_LOCAL_PLANNER_TARGET_R8";
constexpr const char* kSimNodeAdmissionGateId =
    "SMPCC_SIM_LOCAL_PLANNER_GATE_R8";
constexpr const char* kSimNodeAdmissionEnvironmentOwnerPackage =
    "spmpc_sim_local_planner";

// Inputs collected by the simulation node before it creates a NodeHandle,
// publisher, subscriber, timer, or controller object.  Keeping validation
// independent of ROS makes the bypass-resistant boundary unit-testable.
struct SimNodeAdmissionInput {
    std::string gate_token;
    std::string parameter_gate_token;
    std::string ros_master_uri;
    bool use_sim_time = false;
    std::string target_id;
    std::string gate_id;
    std::string environment_owner_package;
    bool launch_marker = false;
    bool release_ack = false;
};

// Accept only the exact self-admission handoff made by
// smpcc_sim_controller_gate.py.  This is deliberately a second gate: direct
// invocation of the binary cannot attach to a real ROS master merely by
// bypassing the Python launch wrapper.
bool validateSimNodeAdmission(const SimNodeAdmissionInput& input,
                              std::string& reason);

}  // namespace spmpc_sim_local_planner
