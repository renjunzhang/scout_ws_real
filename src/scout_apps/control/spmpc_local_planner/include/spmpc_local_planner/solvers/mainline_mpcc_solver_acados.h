#pragma once

#include <array>
#include <memory>
#include <string>

#include "model_contract_generated.h"
#include "spmpc_local_planner/solvers/mainline_artifact_identity.h"

namespace spmpc_local_planner {
namespace mainline {

using MainlineState = std::array<double, generated::NX>;
using MainlineControl = std::array<double, generated::NU>;
using MainlineStageParameters = std::array<double, generated::NP>;
using MainlineParameterHorizon =
    std::array<MainlineStageParameters, generated::PARAMETER_VECTOR_COUNT>;
using MainlineStateHorizon =
    std::array<MainlineState, generated::PARAMETER_VECTOR_COUNT>;
using MainlineControlHorizon = std::array<MainlineControl, generated::N>;

struct MainlinePrimalWarmStart {
  MainlineStateHorizon states{};
  MainlineControlHorizon controls{};
};

struct MainlineSolveRequest {
  MainlineState initial_state{};
  MainlineParameterHorizon stage_parameters{};
  bool has_primal_warm_start{false};
  MainlinePrimalWarmStart primal_warm_start{};
};

struct MainlineSolveResult {
  bool success{false};
  int acados_status{-1};
  int sqp_iterations{0};
  double solver_time_sec{0.0};
  double total_cost{0.0};
  std::string failure_reason;
  MainlineStateHorizon states{};
  MainlineControlHorizon controls{};
};

// Sole wrapper for spmpc_actuator_slosh_discrete_v1. It has no ROS or legacy
// SpmpcSolver dependency and never chooses between B0/Bslosh artifacts.
class MainlineMpccSolverAcados final {
 public:
  explicit MainlineMpccSolverAcados(const std::string& artifact_directory);
  ~MainlineMpccSolverAcados();

  MainlineMpccSolverAcados(const MainlineMpccSolverAcados&) = delete;
  MainlineMpccSolverAcados& operator=(const MainlineMpccSolverAcados&) = delete;

  MainlineSolveResult solve(const MainlineSolveRequest& request);
  const VerifiedArtifactIdentity& artifactIdentity() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mainline
}  // namespace spmpc_local_planner
