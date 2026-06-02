#include "spmpc_local_planner/core/variant_config.h"

namespace spmpc_local_planner {

VariantConfig makeVariantConfig(const std::string& variant_name) {
    VariantConfig cfg;
    cfg.name = variant_name;

    if (variant_name == "B_slosh" || variant_name == "B_slosh_linear") {
        cfg.slosh_enable = true;
        cfg.w_slosh = 1.0;
    } else if (variant_name == "B_slosh_anti") {
        cfg.slosh_enable = true;
        cfg.primitive_mode = "anti_slosh";
        cfg.w_slosh = 1.0;
    } else if (variant_name == "B_smooth") {
        cfg.smooth_priority_enable = true;
        cfg.w_smooth = 1.0;
        cfg.w_control = 0.3;
    } else if (variant_name == "B_ours") {
        cfg.slosh_enable = true;
        cfg.smooth_priority_enable = true;
        cfg.w_slosh = 1.0;
        cfg.w_smooth = 1.0;
        cfg.w_control = 0.3;
    } else if (variant_name == "B_ours_anti") {
        cfg.slosh_enable = true;
        cfg.smooth_priority_enable = true;
        cfg.primitive_mode = "anti_slosh";
        cfg.w_slosh = 1.0;
        cfg.w_smooth = 1.0;
        cfg.w_control = 0.3;
    } else {
        cfg.name = "B0";
    }

    return cfg;
}

}  // namespace spmpc_local_planner
