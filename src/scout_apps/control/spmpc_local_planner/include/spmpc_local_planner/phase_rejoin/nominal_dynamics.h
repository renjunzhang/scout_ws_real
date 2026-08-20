#pragma once

#include "spmpc_local_planner/phase_rejoin/types.h"

namespace spmpc_local_planner {

struct NominalDynamicsState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double s = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

struct NominalDynamicsControl {
    double a = 0.0;
    double alpha = 0.0;
    double v_s = 0.0;
};

struct NominalDynamicsModel {
    double dt = 0.0;
    double two_zeta_omega_n = 0.0;
    double omega_n_sq = 0.0;
    double kappa_x = 0.0;
    double kappa_y = 0.0;
};

NominalDynamicsState phaseNominalRk4Step(
    const NominalDynamicsState& state,
    const NominalDynamicsControl& control,
    const NominalDynamicsModel& model);

NominalDynamicsState phaseNominalRk4Step(
    const PhaseNominalSample& sample,
    const NominalArtifactMetadata& metadata);

}  // namespace spmpc_local_planner
