#include "spmpc_local_planner/controller/control_cycle_input_preparer.h"

#include "spmpc_local_planner/runtime/state_alignment.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

bool ControlCycleInputPreparer::configureObserver(
    const SloshObserverSelectorParams& params) {
    return observer_selector_.configure(params);
}

bool ControlCycleInputPreparer::configurePrediction(
    const SloshModelParams& params) {
    prediction_configured_ = execution_predictor_.configure(params);
    if (prediction_configured_) {
        slosh_params_ = params;
    }
    return prediction_configured_;
}

bool ControlCycleInputPreparer::configureExecutionHorizon(
    const ExecutionModelContract& contract,
    const ExecutionHorizonBuilderConfig& config,
    std::string& error) {
    if (!prediction_configured_) {
        error = "slosh prediction must be configured first";
        return false;
    }
    return execution_horizon_builder_.configure(
        contract, slosh_params_, config, error);
}

bool ControlCycleInputPreparer::executionTiming(
    const DelayPhaseParams& params,
    double& required_history_sec,
    double& execution_lead_sec,
    int& grid_execution_lead_steps) const {
    return execution_predictor_.executionTiming(
        params, required_history_sec, execution_lead_sec,
        grid_execution_lead_steps);
}

void ControlCycleInputPreparer::resetObserver() {
    observer_selector_.reset();
}

std::string ControlCycleInputPreparer::observerFailureStatus(
    const SloshObserverSelection& selection) {
    return std::string("WAITING_FOR_SLOSH_OBSERVER_") +
        sloshObserverSelectionStatusName(selection.status) + "_" +
        sloshObserverSelectionReasonName(selection.reason);
}

bool ControlCycleInputPreparer::odomFresh(
    StampNs evaluation_time_ns,
    StampNs last_receive_ns,
    double timeout_sec) {
    if (last_receive_ns <= 0) {
        return false;
    }
    const double age_sec = secondsBetween(
        evaluation_time_ns, last_receive_ns);
    return !std::isfinite(timeout_sec) || timeout_sec <= 0.0 ||
        age_sec <= timeout_sec;
}

ControlCycleInputResult ControlCycleInputPreparer::prepareState(
    const ControlCycleInputRequest& request) {
    ControlCycleInputResult result;
    result.timing.cycle_id = request.cycle_id;
    result.timing.cycle_start_stamp_ns = request.cycle_start_ns;
    result.timing.raw_robot_state_stamp_ns =
        request.raw_robot_state_stamp_ns;

    CycleTimingContract cycle;
    cycle.cycle_id = request.cycle_id;
    cycle.cycle_start_stamp_ns = request.cycle_start_ns;
    cycle.control_period_sec = request.dt;
    const bool estimate_supplied =
        request.publish_epoch_estimate.status != "NOT_EVALUATED";
    if (estimate_supplied && !publishEpochEstimateMatchesCycle(
            request.publish_epoch_estimate, cycle)) {
        result.failure = ControlInputFailure::PublishEpochContract;
        result.status = "PUBLISH_EPOCH_CONTRACT_MISMATCH";
        result.timing.publish_timing_status = result.status;
        return result;
    }
    result.raw_input.publish_epoch_estimate =
        request.publish_epoch_estimate;
    if (estimate_supplied) {
        applyPublishEpochEstimate(
            request.publish_epoch_estimate, result.timing);
    }

    result.observer_selection = observer_selector_.select(
        request.odom_observer,
        request.imu_observer,
        request.selection_time_ns);
    result.timing.raw_liquid_state_stamp_ns =
        result.observer_selection.selected_state_stamp_ns;
    result.timing.state_alignment_required =
        request.solver_consumes_selected_state &&
        request.state_timing.require_common_epoch;

    if (request.solver_consumes_selected_state &&
        !result.observer_selection.valid) {
        result.failure = ControlInputFailure::ObserverUnavailable;
        result.status = observerFailureStatus(result.observer_selection);
        return result;
    }
    if (result.observer_selection.valid) {
        result.raw_input.slosh = result.observer_selection.state;
    } else if (request.odom_observer.snapshot.configured &&
               request.odom_observer.snapshot.valid) {
        // Preserve the historical paired-diagnostic state for a comparator
        // that does not consume liquid state in its solver.
        result.raw_input.slosh = request.odom_observer.snapshot.state;
    }

    if (!request.robot_state_lookup) {
        result.failure = result.timing.state_alignment_required
            ? ControlInputFailure::CommonEpochRobotUnavailable
            : ControlInputFailure::LatestRobotUnavailable;
        result.status = result.timing.state_alignment_required
            ? "STATE_TIME_ALIGNMENT_FAILED_LOOKUP_PORT_UNAVAILABLE"
            : "WAITING_FOR_TF_POSE";
        result.publish_early_delay_status =
            !result.timing.state_alignment_required;
        result.delay_phase_status = DelayPhaseStatusCode::NoTfPose;
        return result;
    }

    if (result.timing.state_alignment_required) {
        double raw_skew_sec = 0.0;
        if (!stateSkewWithinContract(
                result.timing.raw_robot_state_stamp_ns,
                result.timing.raw_liquid_state_stamp_ns,
                request.state_timing.max_raw_skew_sec,
                raw_skew_sec)) {
            result.timing.raw_state_skew_sec = raw_skew_sec;
            result.timing.state_alignment_status =
                "RAW_STATE_SKEW_CONTRACT_FAILED";
            result.failure = ControlInputFailure::RawStateSkew;
            result.status = "STATE_TIME_ALIGNMENT_FAILED_RAW_SKEW";
            return result;
        }
        result.timing.raw_state_skew_sec = raw_skew_sec;
        const RobotStateLookupResult lookup = request.robot_state_lookup(
            result.timing.raw_liquid_state_stamp_ns);
        if (!lookup.valid) {
            result.timing.state_alignment_status = lookup.status;
            result.failure =
                ControlInputFailure::CommonEpochRobotUnavailable;
            result.status = "STATE_TIME_ALIGNMENT_FAILED_" + lookup.status;
            return result;
        }
        result.raw_input.robot = lookup.state;
        result.timing.robot_state_stamp_ns =
            result.timing.raw_liquid_state_stamp_ns;
        result.timing.liquid_state_stamp_ns =
            result.timing.raw_liquid_state_stamp_ns;
        result.timing.solver_input_epoch_ns =
            result.timing.raw_liquid_state_stamp_ns;
        result.timing.aligned_state_skew_sec = 0.0;
        result.timing.state_time_aligned = true;
        result.timing.robot_state_interpolated = lookup.interpolated;
        result.timing.robot_state_extrapolated = lookup.extrapolated;
        result.timing.state_alignment_status = lookup.status;
    } else {
        const RobotStateLookupResult lookup = request.robot_state_lookup(0);
        if (!lookup.valid) {
            result.timing.state_alignment_status =
                "LATEST_TF_UNAVAILABLE";
            result.failure = ControlInputFailure::LatestRobotUnavailable;
            result.status = "WAITING_FOR_TF_POSE";
            result.delay_phase_status = DelayPhaseStatusCode::NoTfPose;
            result.publish_early_delay_status = true;
            return result;
        }
        result.raw_input.robot = lookup.state;
        result.timing.robot_state_stamp_ns =
            result.timing.raw_robot_state_stamp_ns;
        result.timing.liquid_state_stamp_ns =
            result.timing.raw_liquid_state_stamp_ns;
        result.timing.solver_input_epoch_ns =
            result.timing.raw_robot_state_stamp_ns;
        result.timing.aligned_state_skew_sec =
            result.timing.liquid_state_stamp_ns > 0
                ? secondsBetween(
                    result.timing.robot_state_stamp_ns,
                    result.timing.liquid_state_stamp_ns)
                : 0.0;
        result.timing.raw_state_skew_sec =
            result.timing.raw_liquid_state_stamp_ns > 0
                ? secondsBetween(
                    result.timing.raw_robot_state_stamp_ns,
                    result.timing.raw_liquid_state_stamp_ns)
                : 0.0;
        result.timing.state_time_aligned =
            !request.solver_consumes_selected_state ||
            std::abs(result.timing.aligned_state_skew_sec) <= 1e-6;
        result.timing.state_alignment_status =
            request.solver_consumes_selected_state
                ? "COMMON_EPOCH_DISABLED"
                : "LIQUID_NOT_CONSUMED";
    }

    result.raw_input.cycle_timing = result.timing;
    result.raw_input.dt = request.dt;
    result.raw_input.horizon_steps = request.horizon_steps;
    result.solver_input = result.raw_input;
    result.ready = true;
    result.status = "STATE_READY";
    return result;
}

ControlCycleInputResult ControlCycleInputPreparer::completePrediction(
    const ControlCycleInputRequest& request,
    StampNs prediction_evaluation_ns,
    ControlCycleInputResult result) {
    if (!result.ready) {
        return result;
    }
    result.ready = false;
    result.status = "PREDICTION_NOT_PREPARED";
    result.failure = ControlInputFailure::None;
    result.raw_input.cycle_timing = result.timing;
    result.solver_input = result.raw_input;
    result.prediction.raw_robot = result.raw_input.robot;
    result.prediction.raw_slosh = result.raw_input.slosh;
    result.prediction.predicted_robot = result.raw_input.robot;
    result.prediction.predicted_slosh = result.raw_input.slosh;
    result.prediction.status_code = DelayPhaseStatusCode::Off;
    result.delay_phase_status = DelayPhaseStatusCode::MonitorOk;

    CycleTimingContract cycle;
    cycle.cycle_id = request.cycle_id;
    cycle.cycle_start_stamp_ns = request.cycle_start_ns;
    cycle.control_period_sec = request.dt;
    const bool estimate_supplied =
        request.publish_epoch_estimate.status != "NOT_EVALUATED";
    if (estimate_supplied && !publishEpochEstimateMatchesCycle(
            request.publish_epoch_estimate, cycle)) {
        result.failure = ControlInputFailure::PublishEpochContract;
        result.status = "PUBLISH_EPOCH_CONTRACT_MISMATCH";
        result.timing.publish_timing_status = result.status;
        return result;
    }
    result.prediction_uses_expected_publish_epoch =
        request.publish_epoch_estimate.valid;
    result.prediction_evaluation_epoch_ns =
        result.prediction_uses_expected_publish_epoch
            ? request.publish_epoch_estimate.expected_publish_stamp_ns
            : prediction_evaluation_ns;
    result.raw_input.publish_epoch_estimate =
        request.publish_epoch_estimate;
    result.solver_input.publish_epoch_estimate =
        request.publish_epoch_estimate;
    if (estimate_supplied) {
        applyPublishEpochEstimate(
            request.publish_epoch_estimate, result.timing);
    }

    if (request.execution_horizon_requested) {
        ExecutionHorizonBuildRequest horizon_request;
        horizon_request.source_robot = result.raw_input.robot;
        horizon_request.source_slosh = result.raw_input.slosh;
        horizon_request.source_epoch_ns =
            result.raw_input.cycle_timing.solver_input_epoch_ns;
        horizon_request.publish_epoch_estimate =
            request.publish_epoch_estimate;
        horizon_request.command_history = request.command_history;
        horizon_request.expected_execution_contract_hash =
            request.execution_contract_hash;
        horizon_request.initial_progress_s =
            request.execution_initial_progress_s;
        horizon_request.liquid_horizon_steps =
            request.execution_liquid_horizon_steps;
        result.execution_horizon_build =
            execution_horizon_builder_.build(horizon_request);
        if (!result.execution_horizon_build.valid) {
            result.failure =
                ControlInputFailure::ExecutionHorizonContext;
            result.status = result.execution_horizon_build.status;
            return result;
        }
        result.solver_input.execution_horizon =
            result.execution_horizon_build.context;
        result.execution_horizon_active = true;
    }

    const bool prediction_requested =
        delayPhaseUsesPrediction(request.delay_phase.mode) ||
        request.phase_rejoin_needs_prediction;
    if (prediction_requested) {
        DelayPhaseParams predictor_params = request.delay_phase;
        if (predictor_params.mode == DelayPhaseMode::Off) {
            predictor_params.mode = DelayPhaseMode::Shadow;
        }
        if (request.command_history) {
            result.prediction = execution_predictor_.predict(
                result.raw_input.robot,
                result.raw_input.slosh,
                *request.command_history,
                result.raw_input.cycle_timing.solver_input_epoch_ns,
                result.prediction_evaluation_epoch_ns,
                predictor_params);
        } else {
            result.prediction.status = "NO_COMMAND_HISTORY_PORT";
            result.prediction.status_code =
                DelayPhaseStatusCode::NoCmdHistory;
        }
        result.have_prediction = true;
        result.delay_phase_status = result.prediction.status_code;
    }

    if (delayPhaseUsesClosedLoop(request.delay_phase.mode) &&
        result.have_prediction) {
        const bool odom_is_fresh = odomFresh(
            result.prediction_evaluation_epoch_ns,
            request.last_odom_receive_ns,
            request.delay_phase.odom_timeout_sec);
        const DelayPhaseApplication application =
            composeDelayPhaseState(
                result.raw_input.robot,
                result.raw_input.slosh,
                result.prediction,
                request.delay_phase.mode,
                odom_is_fresh);
        result.solver_input.robot = application.robot;
        result.solver_input.slosh = application.slosh;
        result.robot_delay_compensation_applied =
            application.robot_applied;
        result.liquid_delay_compensation_applied =
            application.liquid_applied;
        if (application.robot_applied && application.liquid_applied) {
            const StampNs predicted_epoch_ns =
                result.prediction.prediction_epoch_ns;
            result.timing.robot_state_stamp_ns = predicted_epoch_ns;
            result.timing.liquid_state_stamp_ns = predicted_epoch_ns;
            result.timing.solver_input_epoch_ns = predicted_epoch_ns;
            result.timing.aligned_state_skew_sec = 0.0;
            result.timing.state_time_aligned = true;
            result.timing.state_alignment_status =
                "DELAY_PREDICTED_COMMON_EPOCH";
        } else if (result.timing.state_alignment_required &&
                   application.anyApplied()) {
            result.timing.state_time_aligned = false;
            result.timing.state_alignment_status =
                "PARTIAL_DELAY_STATE_APPLICATION_FORBIDDEN";
            result.failure =
                ControlInputFailure::PartialDelayStateApplication;
            result.status = "STATE_TIME_ALIGNMENT_FAILED_DELAY_PHASE";
            return result;
        }
        if (!odom_is_fresh) {
            result.delay_phase_status = DelayPhaseStatusCode::OdomStale;
        }
    }

    result.solver_origin_at_execution_front =
        result.robot_delay_compensation_applied &&
        result.liquid_delay_compensation_applied;
    result.solver_input.cycle_timing = result.timing;
    result.execution_front_steps = result.have_prediction
        ? result.prediction.grid_execution_lead_steps
        : 0;
    result.ready = true;
    result.status = "READY";
    return result;
}

ControlCycleInputResult ControlCycleInputPreparer::prepare(
    const ControlCycleInputRequest& request) {
    ControlCycleInputResult result = prepareState(request);
    const StampNs prediction_evaluation_ns =
        request.prediction_evaluation_ns > 0
            ? request.prediction_evaluation_ns
            : request.selection_time_ns;
    return completePrediction(
        request, prediction_evaluation_ns, result);
}

}  // namespace spmpc_local_planner
