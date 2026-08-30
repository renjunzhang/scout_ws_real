#pragma once

#include <string>

namespace spmpc_local_planner {

// Independent hard ceiling for real-vehicle diagnostic runs.  This contract is
// deliberately separate from VariantConfig::v_ref: v_ref is a tracking target,
// while v_safe_max is a fail-closed safety boundary shared by the solver and
// the final command publication path.
struct SpeedSafetyParams {
    bool enable = false;
    double v_safe_max = 0.15;
    double tolerance = 1e-4;
};

struct SpeedSafetyDecision {
    bool enabled = false;
    bool solver_violation = false;
    bool post_gate_violation = false;
    bool publish_candidate_violation = false;
    bool violation = false;
    bool newly_latched = false;
    bool latched = false;
    double v_safe_max = 0.0;
    double tolerance = 0.0;
    std::string status = "DISABLED";
};

class SpeedSafetyContract {
public:
    // When enabled, v_safe_max must be a real restriction of the platform
    // ceiling.  Invalid configurations are rejected at node startup instead of
    // silently pretending that a safety contract is active.
    bool configure(const SpeedSafetyParams& params,
                   double platform_v_max,
                   std::string* error = nullptr);

    // Inspect the three authoritative command stages.  Non-finite values are
    // unsafe.  Any violation latches until configure() or reset() is called;
    // the ROS node intentionally never resets it during a process lifetime.
    SpeedSafetyDecision inspect(double solver_cmd_v,
                                double post_gate_cmd_v,
                                double publish_candidate_v);

    void reset();

    bool configured() const { return configured_; }
    bool enabled() const { return params_.enable; }
    bool latched() const { return latched_; }
    double platformVMax() const { return platform_v_max_; }
    double effectiveVMax() const { return effective_v_max_; }
    const SpeedSafetyParams& params() const { return params_; }

private:
    bool exceedsLimit(double value) const;

    SpeedSafetyParams params_;
    double platform_v_max_ = 0.0;
    double effective_v_max_ = 0.0;
    bool configured_ = false;
    bool latched_ = false;
};

}  // namespace spmpc_local_planner
