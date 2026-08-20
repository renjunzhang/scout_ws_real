#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/domain/command.h"
#include "spmpc_local_planner/domain/time.h"
#include <algorithm>
#include <cctype>
#include <string>

namespace spmpc_local_planner {

enum class DelayPhaseMode {
    Off = 0,
    Monitor = 1,
    Shadow = 2,
    FixedClosedLoop = 3,
    // Predict the robot execution pose/twist, but keep the measured liquid
    // state untouched.  This prevents command-history rollout from replacing
    // the current processed-IMU state with a differently timed model state.
    FixedRobotOnly = 4,
};

enum class DelayPhaseStatusCode {
    Off = 0,
    MonitorOk = 1,
    ShadowOk = 2,
    NoOdom = 3,
    NoReference = 4,
    NoTfPose = 5,
    NoCmdHistory = 6,
    PartialHistory = 7,
    CmdStale = 8,
    OdomStale = 9,
    InvalidParams = 10,
    FixedClosedLoopOk = 11,
    FixedRobotOnlyOk = 12,
};

struct DelayPhaseParams {
    DelayPhaseMode mode = DelayPhaseMode::Off;
    bool publish_diagnostics = true;
    double history_window_sec = 2.0;
    double cmd_timeout_sec = 0.5;
    double odom_timeout_sec = 0.5;
    double linear_delay_sec = 0.15;
    double angular_delay_sec = 0.22;
    // Optional identified first-order execution inertia after each pure-delay
    // channel.  Zero preserves the historical pure-delay behavior.
    double linear_time_constant_sec = 0.0;
    double angular_time_constant_sec = 0.0;
    double max_prediction_sec = 0.40;
    double max_integration_step_sec = 0.02;
    double min_integration_step_sec = 0.001;
    bool require_complete_history = false;
};

struct CommandPublishMeta {
    bool is_zero_cmd = false;
    bool linear_limited = false;
    bool angular_rate_limited = false;
    bool angular_accel_limited = false;
};

struct StateTimingParams {
    bool require_common_epoch = true;
    double max_raw_skew_sec = 0.080;
    double odom_history_sec = 1.0;
    double max_interpolation_gap_sec = 0.050;
    double max_robot_extrapolation_sec = 0.010;
};

struct CommandExecutionContractParams {
    bool fail_closed_on_post_limit_change = false;
    double max_post_limit_delta_v = 1e-4;
    double max_post_limit_delta_omega = 1e-4;
};

struct OdomTimingDebug {
    double recv_age_ms = 0.0;
    double stamp_dt_ms = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double omega = 0.0;
    bool have_prev_odom = false;
    bool dt_clamped = false;
};

struct ExecutionStatePrediction {
    bool valid = false;
    RobotState raw_robot;
    SloshState raw_slosh;
    RobotState predicted_robot;
    SloshState predicted_slosh;
    double linear_delay_sec = 0.0;
    double angular_delay_sec = 0.0;
    double linear_time_constant_sec = 0.0;
    double angular_time_constant_sec = 0.0;
    std::int64_t prediction_origin_epoch_ns = 0;
    std::int64_t prediction_epoch_ns = 0;
    double integrated_duration_sec = 0.0;
    double covered_history_sec = 0.0;
    double missing_history_sec = 0.0;
    double history_span_sec = 0.0;
    bool history_complete = false;
    DelayPhaseStatusCode status_code = DelayPhaseStatusCode::Off;
    std::string status = "OFF";
};

struct DelayPhaseApplication {
    SolverInput solver_input;
    bool robot_applied = false;
    bool liquid_applied = false;

    bool anyApplied() const {
        return robot_applied || liquid_applied;
    }
};

struct DelayPhaseDebugSummary {
    DelayPhaseMode mode = DelayPhaseMode::Off;
    double cmd_age_ms = 0.0;
    double cmd_period_ms = 0.0;
    double odom_age_ms = 0.0;
    double solver_time_ms = 0.0;
    double linear_delay_ms = 0.0;
    double angular_delay_ms = 0.0;
    double history_span_ms = 0.0;
    bool history_complete = false;
    bool shadow_valid = false;
    bool closed_loop_enabled = false;
    DelayPhaseStatusCode status_code = DelayPhaseStatusCode::Off;
};

struct CmdOdomAlignmentDebug {
    DelayPhaseMode mode = DelayPhaseMode::Off;
    double cmd_age_ms = 0.0;
    double cmd_period_ms = 0.0;
    double odom_age_ms = 0.0;
    double odom_period_ms = 0.0;
    double linear_delay_ms = 0.0;
    double angular_delay_ms = 0.0;
    double history_span_ms = 0.0;
    double covered_history_ms = 0.0;
    double missing_history_ms = 0.0;
    bool history_complete = false;
    bool shadow_valid = false;
    bool fixed_closed_loop_configured = false;
    bool fixed_closed_loop_applied = false;
    DelayPhaseStatusCode status_code = DelayPhaseStatusCode::Off;
    double dx_pred_raw = 0.0;
    double dy_pred_raw = 0.0;
    double dyaw_pred_raw = 0.0;
    double dv_pred_raw = 0.0;
    double domega_pred_raw = 0.0;
    double deta_norm_pred_raw = 0.0;
    double deta_dot_norm_pred_raw = 0.0;
};

inline std::string delayPhaseModeName(DelayPhaseMode mode) {
    switch (mode) {
    case DelayPhaseMode::Monitor:
        return "monitor";
    case DelayPhaseMode::Shadow:
        return "shadow";
    case DelayPhaseMode::FixedClosedLoop:
        return "fixed_closed_loop";
    case DelayPhaseMode::FixedRobotOnly:
        return "fixed_robot_only";
    case DelayPhaseMode::Off:
    default:
        return "off";
    }
}

inline std::string delayPhaseStatusName(DelayPhaseStatusCode status) {
    switch (status) {
    case DelayPhaseStatusCode::MonitorOk:
        return "MONITOR_OK";
    case DelayPhaseStatusCode::ShadowOk:
        return "SHADOW_OK";
    case DelayPhaseStatusCode::FixedClosedLoopOk:
        return "FIXED_CLOSED_LOOP_OK";
    case DelayPhaseStatusCode::FixedRobotOnlyOk:
        return "FIXED_ROBOT_ONLY_OK";
    case DelayPhaseStatusCode::NoOdom:
        return "NO_ODOM";
    case DelayPhaseStatusCode::NoReference:
        return "NO_REFERENCE";
    case DelayPhaseStatusCode::NoTfPose:
        return "WAITING_FOR_TF_POSE";
    case DelayPhaseStatusCode::NoCmdHistory:
        return "NO_CMD_HISTORY";
    case DelayPhaseStatusCode::PartialHistory:
        return "PARTIAL_HISTORY";
    case DelayPhaseStatusCode::CmdStale:
        return "CMD_STALE";
    case DelayPhaseStatusCode::OdomStale:
        return "ODOM_STALE";
    case DelayPhaseStatusCode::InvalidParams:
        return "INVALID_PARAMS";
    case DelayPhaseStatusCode::Off:
    default:
        return "OFF";
    }
}

inline DelayPhaseMode parseDelayPhaseMode(const std::string& mode_text) {
    std::string value = mode_text;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "monitor" || value == "p0" || value == "diagnostics") {
        return DelayPhaseMode::Monitor;
    }
    if (value == "shadow" || value == "p1_shadow" || value == "p1-shadow") {
        return DelayPhaseMode::Shadow;
    }
    if (value == "fixed_closed_loop" || value == "fixed-closed-loop" ||
        value == "p1_fixed_closed_loop" || value == "p1-fixed-closed-loop") {
        return DelayPhaseMode::FixedClosedLoop;
    }
    if (value == "fixed_robot_only" || value == "fixed-robot-only" ||
        value == "robot_only_closed_loop" || value == "robot-only-closed-loop") {
        return DelayPhaseMode::FixedRobotOnly;
    }
    return DelayPhaseMode::Off;
}

/// 判断 mode_text 是否是已知合法字符串（忽略大小写）。
/// 用于在 ROS 节点加载参数时检测拼写错误（例如 "fixed_closedloop" 静默退化为 Off）。
inline bool isKnownDelayPhaseMode(const std::string& mode_text) {
    std::string value = mode_text;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value == "off" || value == "monitor" || value == "p0" || value == "diagnostics" ||
           value == "shadow" || value == "p1_shadow" || value == "p1-shadow" ||
           value == "fixed_closed_loop" || value == "fixed-closed-loop" ||
           value == "p1_fixed_closed_loop" || value == "p1-fixed-closed-loop" ||
           value == "fixed_robot_only" || value == "fixed-robot-only" ||
           value == "robot_only_closed_loop" || value == "robot-only-closed-loop";
}

inline bool delayPhaseUsesClosedLoop(DelayPhaseMode mode) {
    return mode == DelayPhaseMode::FixedClosedLoop ||
           mode == DelayPhaseMode::FixedRobotOnly;
}

inline bool delayPhaseUsesPrediction(DelayPhaseMode mode) {
    return mode == DelayPhaseMode::Shadow || delayPhaseUsesClosedLoop(mode);
}

inline DelayPhaseStatusCode delayPhaseReadyStatus(DelayPhaseMode mode) {
    if (mode == DelayPhaseMode::FixedClosedLoop) {
        return DelayPhaseStatusCode::FixedClosedLoopOk;
    }
    if (mode == DelayPhaseMode::FixedRobotOnly) {
        return DelayPhaseStatusCode::FixedRobotOnlyOk;
    }
    return DelayPhaseStatusCode::ShadowOk;
}

/// Build the exact SolverInput selected by a delay mode after external
/// freshness guards (notably odom receive age) have passed.
///
/// FixedClosedLoop preserves the historical behavior and replaces both robot
/// and liquid state. FixedRobotOnly replaces only robot state; its liquid
/// state remains byte-for-byte sourced from the current observer selection.
inline DelayPhaseApplication composeDelayPhaseSolverInput(
    const SolverInput& raw_input,
    const ExecutionStatePrediction& prediction,
    DelayPhaseMode mode,
    bool external_guards_passed) {
    DelayPhaseApplication out;
    out.solver_input = raw_input;
    if (!external_guards_passed || !delayPhaseUsesClosedLoop(mode) ||
        !prediction.valid || !prediction.history_complete ||
        prediction.status_code != delayPhaseReadyStatus(mode)) {
        return out;
    }

    out.solver_input.robot = prediction.predicted_robot;
    out.robot_applied = true;
    if (mode == DelayPhaseMode::FixedClosedLoop) {
        out.solver_input.slosh = prediction.predicted_slosh;
        out.liquid_applied = true;
    }
    return out;
}

}  // namespace spmpc_local_planner
