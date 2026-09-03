#include "spmpc_local_planner/solvers/mainline_mpcc_solver_acados.h"

#include "acados_solver_spmpc_actuator_slosh_discrete_v1.h"
#include "acados_c/ocp_nlp_interface.h"

#include <cmath>
#include <stdexcept>

namespace spmpc_local_planner {
namespace mainline {
namespace {

using Capsule = spmpc_actuator_slosh_discrete_v1_solver_capsule;

static_assert(generated::N == SPMPC_ACTUATOR_SLOSH_DISCRETE_V1_N,
              "generated horizon differs from solver C API");
static_assert(generated::NX == SPMPC_ACTUATOR_SLOSH_DISCRETE_V1_NX,
              "generated state dimension differs from solver C API");
static_assert(generated::NU == SPMPC_ACTUATOR_SLOSH_DISCRETE_V1_NU,
              "generated control dimension differs from solver C API");
static_assert(generated::NP == SPMPC_ACTUATOR_SLOSH_DISCRETE_V1_NP,
              "generated parameter dimension differs from solver C API");
static_assert(SPMPC_ACTUATOR_SLOSH_DISCRETE_V1_NBX0 ==
                  SPMPC_ACTUATOR_SLOSH_DISCRETE_V1_NX,
              "stage-zero bounds must cover the complete initial state");

ArtifactIdentityExpectation compiledExpectation() {
  ArtifactIdentityExpectation expected;
  expected.model_id = generated::kModelId;
  expected.model_contract_semantic_sha256 =
      generated::kModelContractSemanticSha256;
  expected.artifact_sha256 = generated::kArtifactSha256;
  expected.model_contract_filename = generated::kModelContractFilename;
  expected.model_contract_raw_sha256 =
      generated::kModelContractJsonRawSha256;
  expected.solver_library_relative_path =
      generated::kSolverLibraryRelativePath;
  expected.solver_library_size_bytes = generated::kSolverLibrarySizeBytes;
  expected.solver_library_raw_sha256 =
      generated::kSolverLibraryRawSha256;
  return expected;
}

template <typename Values>
bool allFinite(const Values& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

void requireFiniteRequest(const MainlineSolveRequest& request) {
  if (!allFinite(request.initial_state)) {
    throw std::invalid_argument("mainline initial state must be finite");
  }
  for (const auto& parameters : request.stage_parameters) {
    if (!allFinite(parameters)) {
      throw std::invalid_argument("mainline stage parameters must be finite");
    }
  }
  if (!request.has_primal_warm_start) {
    return;
  }
  for (const auto& state : request.primal_warm_start.states) {
    if (!allFinite(state)) {
      throw std::invalid_argument("mainline warm-start states must be finite");
    }
  }
  for (const auto& control : request.primal_warm_start.controls) {
    if (!allFinite(control)) {
      throw std::invalid_argument("mainline warm-start controls must be finite");
    }
  }
}

}  // namespace

struct MainlineMpccSolverAcados::Impl {
  VerifiedArtifactIdentity artifact_identity;
  Capsule* capsule{nullptr};

  explicit Impl(const std::string& artifact_directory)
      : artifact_identity(
            verifyArtifactDirectory(artifact_directory, compiledExpectation())) {
    capsule = spmpc_actuator_slosh_discrete_v1_acados_create_capsule();
    if (capsule == nullptr) {
      throw std::runtime_error("cannot allocate mainline Acados capsule");
    }
    if (spmpc_actuator_slosh_discrete_v1_acados_create(capsule) != 0) {
      spmpc_actuator_slosh_discrete_v1_acados_free_capsule(capsule);
      capsule = nullptr;
      throw std::runtime_error("cannot create mainline Acados solver");
    }
  }

  ~Impl() {
    if (capsule != nullptr) {
      spmpc_actuator_slosh_discrete_v1_acados_free(capsule);
      spmpc_actuator_slosh_discrete_v1_acados_free_capsule(capsule);
    }
  }
};

MainlineMpccSolverAcados::MainlineMpccSolverAcados(
    const std::string& artifact_directory)
    : impl_(new Impl(artifact_directory)) {}

MainlineMpccSolverAcados::~MainlineMpccSolverAcados() = default;

const VerifiedArtifactIdentity& MainlineMpccSolverAcados::artifactIdentity()
    const {
  return impl_->artifact_identity;
}

MainlineSolveResult MainlineMpccSolverAcados::solve(
    const MainlineSolveRequest& request) {
  requireFiniteRequest(request);
  MainlineSolveResult result;
  Capsule* const capsule = impl_->capsule;
  const int reset_status =
      spmpc_actuator_slosh_discrete_v1_acados_reset(capsule, 1);
  if (reset_status != 0) {
    result.failure_reason = "ACADOS_RESET_FAILED_" +
                            std::to_string(reset_status);
    return result;
  }

  ocp_nlp_config* const config =
      spmpc_actuator_slosh_discrete_v1_acados_get_nlp_config(capsule);
  ocp_nlp_dims* const dimensions =
      spmpc_actuator_slosh_discrete_v1_acados_get_nlp_dims(capsule);
  ocp_nlp_in* const input =
      spmpc_actuator_slosh_discrete_v1_acados_get_nlp_in(capsule);
  ocp_nlp_out* const output =
      spmpc_actuator_slosh_discrete_v1_acados_get_nlp_out(capsule);
  ocp_nlp_solver* const solver =
      spmpc_actuator_slosh_discrete_v1_acados_get_nlp_solver(capsule);
  if (config == nullptr || dimensions == nullptr || input == nullptr ||
      output == nullptr || solver == nullptr) {
    result.failure_reason = "ACADOS_OBJECT_GRAPH_INCOMPLETE";
    return result;
  }

  MainlineState initial_state = request.initial_state;
  ocp_nlp_constraints_model_set(config, dimensions, input, output, 0, "lbx",
                                initial_state.data());
  ocp_nlp_constraints_model_set(config, dimensions, input, output, 0, "ubx",
                                initial_state.data());
  for (std::size_t stage = 0; stage < generated::PARAMETER_VECTOR_COUNT;
       ++stage) {
    MainlineStageParameters parameters = request.stage_parameters[stage];
    const int parameter_status =
        spmpc_actuator_slosh_discrete_v1_acados_update_params(
            capsule, static_cast<int>(stage), parameters.data(),
            static_cast<int>(generated::NP));
    if (parameter_status != 0) {
      result.failure_reason = "ACADOS_PARAMETER_UPDATE_FAILED_STAGE_" +
                              std::to_string(stage) + "_STATUS_" +
                              std::to_string(parameter_status);
      return result;
    }
  }

  const MainlineControl zero_control{};
  for (std::size_t stage = 0; stage <= generated::N; ++stage) {
    MainlineState state_guess =
        request.has_primal_warm_start
            ? request.primal_warm_start.states[stage]
            : request.initial_state;
    ocp_nlp_out_set(config, dimensions, output, input, static_cast<int>(stage),
                    "x", state_guess.data());
    if (stage < generated::N) {
      MainlineControl control_guess =
          request.has_primal_warm_start
              ? request.primal_warm_start.controls[stage]
              : zero_control;
      ocp_nlp_out_set(config, dimensions, output, input,
                      static_cast<int>(stage), "u", control_guess.data());
    }
  }

  result.acados_status =
      spmpc_actuator_slosh_discrete_v1_acados_solve(capsule);
  ocp_nlp_get(solver, "time_tot", &result.solver_time_sec);
  ocp_nlp_get(solver, "sqp_iter", &result.sqp_iterations);
  if (result.acados_status != 0) {
    result.failure_reason =
        "ACADOS_SOLVE_FAILED_" + std::to_string(result.acados_status);
    return result;
  }

  ocp_nlp_eval_cost(solver, input, output);
  ocp_nlp_get(solver, "cost_value", &result.total_cost);
  for (std::size_t stage = 0; stage <= generated::N; ++stage) {
    ocp_nlp_out_get(config, dimensions, output, static_cast<int>(stage), "x",
                    result.states[stage].data());
    if (!allFinite(result.states[stage])) {
      result.failure_reason = "ACADOS_RETURNED_NONFINITE_STATE";
      return result;
    }
    if (stage < generated::N) {
      ocp_nlp_out_get(config, dimensions, output, static_cast<int>(stage), "u",
                      result.controls[stage].data());
      if (!allFinite(result.controls[stage])) {
        result.failure_reason = "ACADOS_RETURNED_NONFINITE_CONTROL";
        return result;
      }
    }
  }
  if (!std::isfinite(result.solver_time_sec) ||
      !std::isfinite(result.total_cost)) {
    result.failure_reason = "ACADOS_RETURNED_NONFINITE_DIAGNOSTICS";
    return result;
  }
  result.success = true;
  return result;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
