#pragma once

#include "spmpc_local_planner/core/slosh_risk_governor.h"
#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/reference/speed_profile.h"

#include <string>

namespace spmpc_local_planner {

struct SpeedReferenceControllerConfig {
    bool runtime_override_enable = false;
    double runtime_override_mps = 0.0;
    bool profile_enable = false;
    std::string profile_path;
    double profile_lookahead_m = 0.0;
    double variant_v_ref = 0.0;
    bool slosh_variant_enabled = false;
    SloshModelParams slosh_model;
    SloshRiskGovernorParams slosh_governor;
};

struct SpeedReferenceConfigureResult {
    bool profile_requested = false;
    SpeedProfileLoadResult profile_load;
    bool governor_configured = false;
};

struct SpeedReferenceEvaluation {
    bool applied = false;
    bool has_v_ref_current = false;
    double v_ref_current = 0.0;
    std::string v_ref_status = "NOT_CONFIGURED";
    SloshRiskGovernorOutput governor;
};

// Pure C++ owner of all stateful speed-reference selection.  Profile lookup
// uses the previous solver progress, while the risk governor intentionally
// evaluates the raw observer state supplied before delay prediction.
class SpeedReferenceController {
public:
    SpeedReferenceConfigureResult configure(
        const SpeedReferenceControllerConfig& config);

    SpeedReferenceEvaluation apply(const RobotState& raw_robot,
                                   const SloshState& raw_slosh,
                                   SolverInput& input);
    void commitProgress(double progress_abs_s);
    void resetForReference();

private:
    static std::string appendStatus(const std::string& current,
                                    const std::string& suffix);

    SpeedReferenceControllerConfig config_;
    SpeedProfile profile_;
    SloshRiskGovernor governor_;
    double last_progress_abs_s_ = 0.0;
    bool have_progress_ = false;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
