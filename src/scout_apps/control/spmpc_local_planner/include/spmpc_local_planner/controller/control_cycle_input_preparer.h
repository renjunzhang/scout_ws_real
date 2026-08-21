#pragma once

#include "spmpc_local_planner/solver/api/solver_input.h"
#include "spmpc_local_planner/estimation/slosh_observer_selector.h"
#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_state_predictor.h"

#include <cstdint>
#include <functional>
#include <string>

namespace spmpc_local_planner {

struct RobotStateLookupResult {
    bool valid = false;
    RobotState state;
    bool interpolated = false;
    bool extrapolated = false;
    std::string status = "UNAVAILABLE";
};

using RobotStateLookup =
    std::function<RobotStateLookupResult(StampNs target_epoch_ns)>;

enum class ControlInputFailure {
    None,
    PublishEpochContract,
    ObserverUnavailable,
    RawStateSkew,
    CommonEpochRobotUnavailable,
    LatestRobotUnavailable,
    PartialDelayStateApplication,
};

struct ControlCycleInputRequest {
    std::uint64_t cycle_id = 0;
    StampNs cycle_start_ns = 0;
    StampNs selection_time_ns = 0;
    // Optional one-shot prediction clock.  ROS uses the explicit two-stage
    // API below so runtime v-ref/governor work remains between state lookup
    // and prediction exactly as it was before this extraction.
    StampNs prediction_evaluation_ns = 0;
    PublishEpochEstimate publish_epoch_estimate;
    StampNs raw_robot_state_stamp_ns = 0;
    StampNs last_odom_receive_ns = 0;
    SloshObserverHealth odom_observer;
    SloshObserverHealth imu_observer;
    bool solver_consumes_selected_state = false;
    StateTimingParams state_timing;
    double dt = 0.0;
    int horizon_steps = 0;
    DelayPhaseParams delay_phase;
    bool phase_rejoin_needs_prediction = false;
    const CommandHistoryBuffer* command_history = nullptr;
    RobotStateLookup robot_state_lookup;
};

struct ControlCycleInputResult {
    bool ready = false;
    ControlInputFailure failure = ControlInputFailure::None;
    std::string status = "NOT_PREPARED";
    SloshObserverSelection observer_selection;
    ControlCycleTimingDebug timing;
    SolverInput raw_input;
    SolverInput solver_input;
    ExecutionStatePrediction prediction;
    bool have_prediction = false;
    DelayPhaseStatusCode delay_phase_status = DelayPhaseStatusCode::Off;
    bool publish_early_delay_status = false;
    bool robot_delay_compensation_applied = false;
    bool liquid_delay_compensation_applied = false;
    bool solver_origin_at_execution_front = false;
    int execution_front_steps = 0;
    StampNs prediction_evaluation_epoch_ns = 0;
    bool prediction_uses_expected_publish_epoch = false;
};

// Stateful, ROS-independent front half of one control cycle.  It owns observer
// source selection and execution prediction, while robot pose lookup is an
// injected port so ROS TF and offline replay can share the same decisions.
class ControlCycleInputPreparer {
public:
    bool configureObserver(const SloshObserverSelectorParams& params);
    bool configurePrediction(const SloshModelParams& params);
    bool executionTiming(const DelayPhaseParams& params,
                         double& required_history_sec,
                         double& execution_lead_sec,
                         int& grid_execution_lead_steps) const;
    void resetObserver();

    // Select the observer and establish the solver's raw common epoch.  This
    // is a complete fail-closed gate, but intentionally does not predict the
    // execution front yet.
    ControlCycleInputResult prepareState(
        const ControlCycleInputRequest& request);

    // Finish a state-ready result at an explicit evaluation time after any
    // caller-owned input decoration (for example runtime v_ref) is complete.
    ControlCycleInputResult completePrediction(
        const ControlCycleInputRequest& request,
        StampNs prediction_evaluation_ns,
        ControlCycleInputResult state_result);

    // Convenience one-shot entry point for offline replay and unit tests.
    ControlCycleInputResult prepare(
        const ControlCycleInputRequest& request);

private:
    static std::string observerFailureStatus(
        const SloshObserverSelection& selection);
    static bool odomFresh(
        StampNs evaluation_time_ns,
        StampNs last_receive_ns,
        double timeout_sec);

    SloshObserverSelector observer_selector_;
    ExecutionStatePredictor execution_predictor_;
};

}  // namespace spmpc_local_planner
