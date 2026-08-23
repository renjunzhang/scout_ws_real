#include "spmpc_local_planner/solver/acados/stage_parameter_builder.h"

#include "spmpc_parameter_manifest.h"

#include <algorithm>
#include <cstddef>

namespace spmpc_local_planner {
namespace {

using namespace acados_manifest::mainline;

std::vector<std::string> parameterNames(int width) {
    const int available = static_cast<int>(
        sizeof(kParameterNames) / sizeof(kParameterNames[0]));
    const int count = std::max(0, std::min(width, available));
    std::vector<std::string> names;
    names.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        names.emplace_back(kParameterNames[index]);
    }
    return names;
}

void setCommonParameters(const AcadosStageParameterInput& input,
                         std::vector<double>& parameters) {
    parameters[RX0] = input.reference_x_coeffs[0];
    parameters[RX1] = input.reference_x_coeffs[1];
    parameters[RX2] = input.reference_x_coeffs[2];
    parameters[RX3] = input.reference_x_coeffs[3];
    parameters[RY0] = input.reference_y_coeffs[0];
    parameters[RY1] = input.reference_y_coeffs[1];
    parameters[RY2] = input.reference_y_coeffs[2];
    parameters[RY3] = input.reference_y_coeffs[3];
    parameters[W_CONTOUR] = input.variant.w_contour;
    parameters[W_LAG] = input.variant.w_lag;
    parameters[W_PROGRESS] = input.variant.w_progress;
    parameters[W_HEADING] = input.variant.w_heading;
    parameters[W_PROGRESS_COUPLING] = input.variant.w_progress_coupling;
    parameters[W_YAW_RATE_TRACKING] = input.variant.w_yaw_rate_tracking;
    parameters[HEADING_FEEDBACK_GAIN] = input.variant.heading_feedback_gain;
    parameters[W_A] = input.variant.w_control + input.variant.w_accel;
    parameters[W_OMEGA] = input.variant.w_control;
    parameters[W_V] = input.variant.w_v;
    parameters[W_VS] = input.variant.w_vs;
    parameters[W_ALPHA] = input.variant.w_alpha;
    parameters[E_C_REF] = input.contour_error_ref;
    parameters[E_L_REF] = input.lag_error_ref;
    parameters[V_REF] = input.effective_v_ref;
}

void setSloshBaseParameters(const AcadosStageParameterInput& input,
                            std::vector<double>& parameters) {
    parameters[TWO_ZETA_OMEGA_N] = input.slosh.two_zeta_omega_n;
    parameters[OMEGA_N_SQ] = input.slosh.omega_n_sq;
    parameters[KAPPA_X] = 1.0;
    parameters[KAPPA_Y] = 1.0;
    parameters[ETA_REF] = input.slosh.eta_ref;
    parameters[ETA_DOT_REF] = input.slosh.eta_dot_ref;
    parameters[ETA_MAX_SQ] = input.slosh.eta_max_sq;
    parameters[GATE_R_X] = 1.0;
    parameters[GATE_R_Y] = 1.0;
    parameters[GATE_R_YAW] = 1.0;
    parameters[GATE_R_V] = 1.0;
    parameters[GATE_R_OMEGA] = 1.0;
    parameters[GATE_R_ETA_X] = 1.0;
    parameters[GATE_R_ETA_X_DOT] = 1.0;
    parameters[GATE_R_ETA_Y] = 1.0;
    parameters[GATE_R_ETA_Y_DOT] = 1.0;
}

void resetPhaseStageParameters(std::vector<double>& parameters) {
    parameters[PHASE_REJOIN_ACTIVE] = 0.0;
    parameters[EMPIRICAL_GATE_ACTIVE] = 0.0;
    parameters[NOM_X] = 0.0;
    parameters[NOM_Y] = 0.0;
    parameters[NOM_YAW] = 0.0;
    parameters[NOM_V] = 0.0;
    parameters[NOM_OMEGA] = 0.0;
    parameters[NOM_ETA_X] = 0.0;
    parameters[NOM_ETA_X_DOT] = 0.0;
    parameters[NOM_ETA_Y] = 0.0;
    parameters[NOM_ETA_Y_DOT] = 0.0;
    parameters[NOM_A] = 0.0;
    parameters[NOM_ALPHA] = 0.0;
    parameters[NOM_V_S] = 0.0;
    parameters[GATE_R_X] = 1.0;
    parameters[GATE_R_Y] = 1.0;
    parameters[GATE_R_YAW] = 1.0;
    parameters[GATE_R_V] = 1.0;
    parameters[GATE_R_OMEGA] = 1.0;
    parameters[GATE_R_ETA_X] = 1.0;
    parameters[GATE_R_ETA_X_DOT] = 1.0;
    parameters[GATE_R_ETA_Y] = 1.0;
    parameters[GATE_R_ETA_Y_DOT] = 1.0;
}

void setPhaseNominalParameters(const PhaseNominalStage& nominal,
                               std::vector<double>& parameters) {
    parameters[PHASE_REJOIN_ACTIVE] = 1.0;
    parameters[NOM_X] = nominal.x;
    parameters[NOM_Y] = nominal.y;
    parameters[NOM_YAW] = nominal.yaw;
    parameters[NOM_V] = nominal.v;
    parameters[NOM_OMEGA] = nominal.omega;
    parameters[NOM_ETA_X] = nominal.eta_x;
    parameters[NOM_ETA_X_DOT] = nominal.eta_x_dot;
    parameters[NOM_ETA_Y] = nominal.eta_y;
    parameters[NOM_ETA_Y_DOT] = nominal.eta_y_dot;
    parameters[NOM_A] = nominal.a;
    parameters[NOM_ALPHA] = nominal.alpha;
    parameters[NOM_V_S] = nominal.v_s;
    parameters[EMPIRICAL_GATE_ACTIVE] = nominal.gate_active ? 1.0 : 0.0;
    parameters[GATE_R_X] = nominal.radii.x;
    parameters[GATE_R_Y] = nominal.radii.y;
    parameters[GATE_R_YAW] = nominal.radii.yaw;
    parameters[GATE_R_V] = nominal.radii.v;
    parameters[GATE_R_OMEGA] = nominal.radii.omega;
    parameters[GATE_R_ETA_X] = nominal.radii.eta_x;
    parameters[GATE_R_ETA_X_DOT] = nominal.radii.eta_x_dot;
    parameters[GATE_R_ETA_Y] = nominal.radii.eta_y;
    parameters[GATE_R_ETA_Y_DOT] = nominal.radii.eta_y_dot;
}

}  // namespace

double* AcadosStageParameterMatrix::stageData(int stage) {
    if (!valid || stage < 0 || stage >= stage_count || parameter_width <= 0) {
        return nullptr;
    }
    return values.data() +
        static_cast<std::size_t>(stage * parameter_width);
}

const double* AcadosStageParameterMatrix::stageData(int stage) const {
    if (!valid || stage < 0 || stage >= stage_count || parameter_width <= 0) {
        return nullptr;
    }
    return values.data() +
        static_cast<std::size_t>(stage * parameter_width);
}

double AcadosStageParameterMatrix::value(int stage,
                                         int parameter_index) const {
    const double* data = stageData(stage);
    if (data == nullptr || parameter_index < 0 ||
        parameter_index >= parameter_width) {
        return 0.0;
    }
    return data[parameter_index];
}

AcadosStageParameterMatrix AcadosStageParameterBuilder::build(
    const AcadosStageParameterInput& input) {
    AcadosStageParameterMatrix output;
    if (input.horizon_steps < 0) {
        output.status = "NEGATIVE_HORIZON";
        return output;
    }

    const bool phase_rejoin_enforce =
        input.phase_rejoin.active && input.phase_rejoin.enforce;
    if (phase_rejoin_enforce && !input.slosh_enabled) {
        output.status = "PHASE_REJOIN_REQUIRES_SLOSH";
        return output;
    }
    if (phase_rejoin_enforce &&
        (input.phase_rejoin.liquid_steps <= 0 ||
         input.phase_rejoin.liquid_steps > input.horizon_steps ||
         input.phase_rejoin.stages.size() != static_cast<std::size_t>(
             input.phase_rejoin.liquid_steps + 1))) {
        output.status = "PHASE_REJOIN_STAGE_COUNT";
        return output;
    }

    output.stage_count = input.horizon_steps + 1;
    output.parameter_width = input.slosh_enabled
        ? kSloshParameterCount : kB0ParameterCount;
    output.parameter_names = parameterNames(output.parameter_width);
    output.values.reserve(static_cast<std::size_t>(
        output.stage_count * output.parameter_width));

    std::vector<double> parameters(
        static_cast<std::size_t>(output.parameter_width), 0.0);
    setCommonParameters(input, parameters);
    if (input.slosh_enabled) {
        setSloshBaseParameters(input, parameters);
    }

    for (int stage = 0; stage <= input.horizon_steps; ++stage) {
        if (input.slosh_enabled) {
            const double stage_scale = phase_rejoin_enforce
                ? (stage <= input.phase_rejoin.liquid_steps ? 1.0 : 0.0)
                : sloshCostStageScale(
                    input.variant, stage, input.horizon_steps);
            parameters[W_SLOSH_ETA] =
                input.variant.w_slosh * stage_scale;
            parameters[W_SLOSH_ETA_DOT] =
                input.variant.w_slosh *
                input.slosh.eta_dot_weight_ratio * stage_scale;
            parameters[ETA_MAX_SQ] =
                phase_rejoin_enforce &&
                stage > input.phase_rejoin.liquid_steps
                    ? kAcadosDisabledEtaMaxSq
                    : input.slosh.eta_max_sq;

            resetPhaseStageParameters(parameters);
            if (phase_rejoin_enforce &&
                stage <= input.phase_rejoin.liquid_steps) {
                setPhaseNominalParameters(
                    input.phase_rejoin.stages[
                        static_cast<std::size_t>(stage)],
                    parameters);
            }
        }

        if (stage == 0 && input.have_previous_control) {
            parameters[W_DU_A] = input.variant.w_du_a;
            parameters[W_DU_VS] = input.variant.w_du_vs;
            parameters[A_PREV] = input.previous_control[0];
            parameters[VS_PREV] = input.previous_control[2];
        } else {
            parameters[W_DU_A] = 0.0;
            parameters[W_DU_VS] = 0.0;
            parameters[A_PREV] = 0.0;
            parameters[VS_PREV] = 0.0;
        }

        output.values.insert(
            output.values.end(), parameters.begin(), parameters.end());
    }

    output.valid = true;
    output.status = "OK";
    return output;
}

}  // namespace spmpc_local_planner
