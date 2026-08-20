#pragma once

#include "spmpc_local_planner/phase_rejoin/nominal_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/types.h"

#include <cstddef>
#include <map>
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct DevelopmentNominalPoint {
    double x = 0.0;
    double y = 0.0;
};

struct DevelopmentNominalGeneratorConfig {
    double dt = 0.0333333333;
    double cruise_speed = 0.34;
    double ramp_sec = 2.5;
    double lookahead = 0.30;
    double heading_gain = 3.0;
    double omega_max = 1.0;
    double alpha_max = 1.0;
    double omega_n = 0.0;
    double damping_ratio = 0.05;
    double kappa_x = 1.0;
    double kappa_y = 1.0;
    double zero_hold_sec = 2.0;
    double terminal_eta_norm_max = 2.0e-6;
    double terminal_eta_dot_norm_max = 1.0e-4;
    EmpiricalRecoveryRadii radii;
    std::string contract_id;
    std::string frame_id = "map";
    std::string source_bag_sha256;
    std::string path_topic = "/scout/global_path_fixed";
};

struct DevelopmentNominalGenerationResult {
    bool success = false;
    std::string status = "NOT_RUN";
    std::string detail;
    std::vector<PhaseNominalSample> samples;
    std::map<std::string, std::string> metadata;
    std::size_t zero_hold_steps = 0;
    double max_path_deviation = 0.0;
    double path_length = 0.0;
};

class DevelopmentNominalGenerator {
public:
    DevelopmentNominalGenerationResult generate(
        const std::vector<DevelopmentNominalPoint>& points,
        const DevelopmentNominalGeneratorConfig& config) const;
};

}  // namespace spmpc_local_planner
