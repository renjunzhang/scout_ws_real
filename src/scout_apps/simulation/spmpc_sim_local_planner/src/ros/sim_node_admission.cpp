#include "spmpc_sim_local_planner/ros/sim_node_admission.h"

#include <cctype>
#include <sstream>

namespace spmpc_sim_local_planner {

namespace {

void appendViolation(std::string& reason, const std::string& violation) {
    if (!reason.empty()) {
        reason += "; ";
    }
    reason += violation;
}

bool isLowerSha256(const std::string& value) {
    if (value.size() != 64U) {
        return false;
    }
    for (const unsigned char character : value) {
        if (!std::isdigit(character) &&
            !(character >= static_cast<unsigned char>('a') &&
              character <= static_cast<unsigned char>('f'))) {
            return false;
        }
    }
    return true;
}

bool isCanonicalLoopbackMasterUri(const std::string& uri) {
    const std::string prefix = "http://127.0.0.1:";
    if (uri.compare(0U, prefix.size(), prefix) != 0 ||
        uri.size() == prefix.size()) {
        return false;
    }
    unsigned long port = 0U;
    for (std::size_t index = prefix.size(); index < uri.size(); ++index) {
        const unsigned char character =
            static_cast<unsigned char>(uri[index]);
        if (!std::isdigit(character)) {
            return false;
        }
        port = port * 10U + static_cast<unsigned long>(character - '0');
        if (port > 65535U) {
            return false;
        }
    }
    return port >= 1024U;
}

}  // namespace

bool validateSimNodeAdmission(const SimNodeAdmissionInput& input,
                              std::string& reason) {
    reason.clear();

    if (!isLowerSha256(input.gate_token)) {
        appendViolation(reason,
                        "SMPCC_SIM_CONTROLLER_GATE_HASH must be a lowercase SHA-256 token");
    }
    if (!isLowerSha256(input.parameter_gate_token)) {
        appendViolation(reason,
                        "sim_adapter/gate_hash must be a lowercase SHA-256 token");
    }
    if (!input.gate_token.empty() &&
        !input.parameter_gate_token.empty() &&
        input.gate_token != input.parameter_gate_token) {
        appendViolation(reason,
                        "environment gate token and sim_adapter/gate_hash differ");
    }
    if (!isCanonicalLoopbackMasterUri(input.ros_master_uri)) {
        appendViolation(reason,
                        "ROS_MASTER_URI must be exact loopback http://127.0.0.1:<port>");
    }
    if (!input.use_sim_time) {
        appendViolation(reason, "/use_sim_time must be true");
    }
    if (input.target_id != kSimNodeAdmissionTargetId) {
        appendViolation(reason,
                        "sim_adapter/target_id is not the simulation R8 target");
    }
    if (input.gate_id != kSimNodeAdmissionGateId) {
        appendViolation(reason,
                        "sim_adapter/gate_id is not the simulation R8 gate");
    }
    if (input.environment_owner_package !=
        kSimNodeAdmissionEnvironmentOwnerPackage) {
        appendViolation(
            reason,
            "/smpcc_sim_environment/owner_package is not the simulation owner");
    }
    if (!input.launch_marker) {
        appendViolation(reason,
                        "sim_adapter/launch_marker must be explicitly true");
    }
    if (!input.release_ack) {
        appendViolation(reason,
                        "sim_adapter/release_ack must be explicitly true");
    }
    return reason.empty();
}

}  // namespace spmpc_sim_local_planner
