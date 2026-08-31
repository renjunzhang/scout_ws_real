#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/estimation/motion_excitation.h"

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

enum class LiquidNowcastStatusCode : std::uint8_t {
    Disabled = 0,
    ReadyPassThrough = 1,
    ReadyPredicted = 2,
    NotConfigured = 3,
    InvalidSnapshot = 4,
    InvalidState = 5,
    InvalidStamp = 6,
    TargetBeforeState = 7,
    PredictionTooLong = 8,
    InvalidExcitation = 9,
    ExcitationStateSkew = 10,
    ExcitationFromFuture = 11,
    ExcitationStale = 12,
    DynamicsFailure = 13,
};

const char* liquidNowcastStatusName(LiquidNowcastStatusCode status);

struct LiquidStateNowcasterParams {
    bool enable = false;
    double max_prediction_sec = 0.050;
    double max_excitation_age_sec = 0.060;
    double max_future_skew_sec = 0.005;
    double max_state_excitation_skew_sec = 0.001;
    double max_integration_step_sec = 0.020;
};

struct LiquidStateNowcastInput {
    bool snapshot_valid = false;
    SloshState state;
    std::int64_t state_stamp_ns = 0;
    MotionExcitation excitation;
};

struct LiquidStateNowcastResult {
    SloshState input_state;
    SloshState predicted_state;
    std::int64_t input_state_stamp_ns = 0;
    std::int64_t output_state_stamp_ns = 0;
    std::int64_t excitation_effective_stamp_ns = 0;
    std::uint32_t reset_epoch = 0;
    double propagation_sec = 0.0;
    double excitation_age_sec = 0.0;
    bool configured = false;
    bool valid = false;
    LiquidNowcastStatusCode status_code = LiquidNowcastStatusCode::NotConfigured;
    std::string status;
};

// Short, timestamp-driven propagation of one already-observed liquid state.
//
// This class intentionally does not know about odometry, command history,
// observer selection, ROS messages, or solver modes.  It advances q only from
// the observer state stamp to one explicit target epoch, using the latest
// accepted processed-IMU excitation under a bounded zero-order hold.
class LiquidStateNowcaster {
public:
    bool configure(const SloshModelParams& slosh_params,
                   const LiquidStateNowcasterParams& params,
                   std::string* error = nullptr);

    LiquidStateNowcastResult predict(const LiquidStateNowcastInput& input,
                                     std::int64_t target_stamp_ns) const;

    const LiquidStateNowcasterParams& params() const { return params_; }
    bool configured() const { return configured_; }

private:
    static bool finiteState(const SloshState& state);
    static bool finiteExcitation(const MotionExcitation& excitation);
    static std::int64_t effectiveExcitationStamp(const MotionExcitation& excitation);
    static double secondsBetween(std::int64_t newer_ns, std::int64_t older_ns);

    SloshDynamics dynamics_;
    LiquidStateNowcasterParams params_;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
