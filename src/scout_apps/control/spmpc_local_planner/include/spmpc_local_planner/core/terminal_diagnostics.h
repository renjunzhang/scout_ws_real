#pragma once

#include <limits>
#include <string>

namespace spmpc_local_planner {

// 终点控制诊断量（纯 POD）。单独成头，避免 core/types.h（被 reference / dynamics /
// warm_start 等广泛 include）仅为放置 SolverOutput.terminal_diagnostics，就把整个
// TerminalController 类定义拖进这些与终点逻辑无关的模块。
struct TerminalDiagnostics {
    bool enabled = false;
    bool terminal_phase = false;
    bool pre_terminal_phase = true;
    bool envelope_active = false;
    bool stop_pending = false;
    bool position_reached = false;
    bool speed_gate_reached = false;
    bool omega_gate_reached = false;
    bool reached_latch_allowed = true;
    bool reached_latch_blocked = false;
    bool reached = false;
    double distance_to_goal = std::numeric_limits<double>::infinity();
    double remaining_s = std::numeric_limits<double>::infinity();
    double dx_robot = std::numeric_limits<double>::quiet_NaN();
    double v_envelope = std::numeric_limits<double>::infinity();
    double cmd_v_pre_clamp = std::numeric_limits<double>::quiet_NaN();
    double cmd_v_post_clamp = std::numeric_limits<double>::quiet_NaN();
    std::string mode = "NONE";
};

}  // namespace spmpc_local_planner
