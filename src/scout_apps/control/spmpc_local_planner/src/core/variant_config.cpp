#include "spmpc_local_planner/core/variant_config.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

// 变体参数（slosh / smooth 开关 + 各权重）的唯一来源是 config/planner/variants.yaml：
// launch 用 rosparam 把它加载到 variants/<name>/*，由 SpmpcLocalPlannerROS::loadVariantOverrides
// 逐字段覆盖；generate_spmpc_acados.py 也从同一个 yaml 读取 codegen 默认权重。
// 这里只做「变体名规范化」，不再保存任何权重数字，避免 C++ 与 yaml 维护两份会漂移的权重表
// （历史上 C++ 的 w_slosh=1.0 与 yaml 的 5.0 已经漂移过）。
// 未知变体名归一到 "B0"；缺少 yaml 覆盖时回退到 VariantConfig 的 B0 默认值（ROS 层会 WARN）。
VariantConfig makeVariantConfig(const std::string& variant_name) {
    VariantConfig cfg;
    if (variant_name == "B_accel" || variant_name == "B_inst_excitation") {
        cfg.name = "B_accel";  // 兼容旧别名 B_inst_excitation -> B_accel
    } else if (variant_name == "B0" || variant_name == "B_slosh" ||
               variant_name == "B_slosh_hard" || variant_name == "B_slosh_linear" ||
               variant_name == "B_slosh_anti" || variant_name == "B_smooth" ||
               variant_name == "B_ours" || variant_name == "B_ours_hard" ||
               variant_name == "B_ours_anti" ||
               variant_name == "B_slosh_matched0" ||
               variant_name == "B_slosh_matched5") {
        cfg.name = variant_name;
    } else {
        cfg.name = "B0";
    }
    return cfg;
}

double sloshCostStageScale(const VariantConfig& config,
                           int stage,
                           int horizon_steps) {
    if (stage < 0 || horizon_steps < 0 || stage > horizon_steps) {
        return 0.0;
    }
    if (config.slosh_cost_horizon_steps < 0) {
        return 1.0;
    }
    const int trusted_last_stage = std::min(
        horizon_steps, std::max(0, config.slosh_cost_horizon_steps));
    if (stage <= trusted_last_stage) {
        return 1.0;
    }
    if (!std::isfinite(config.slosh_cost_tail_discount)) {
        return 0.0;
    }
    return std::max(0.0, std::min(1.0, config.slosh_cost_tail_discount));
}

bool matchedVariantCommonConfigEqual(const VariantConfig& lhs,
                                     const VariantConfig& rhs) {
    const auto same = [](double a, double b) {
        return std::abs(a - b) <= 1e-12;
    };
    return lhs.slosh_enable == rhs.slosh_enable &&
           lhs.smooth_priority_enable == rhs.smooth_priority_enable &&
           lhs.slosh_constraint_enable == rhs.slosh_constraint_enable &&
           lhs.primitive_mode == rhs.primitive_mode &&
           same(lhs.w_contour, rhs.w_contour) &&
           same(lhs.w_lag, rhs.w_lag) &&
           same(lhs.w_progress, rhs.w_progress) &&
           same(lhs.w_v, rhs.w_v) &&
           same(lhs.w_vs, rhs.w_vs) &&
           same(lhs.v_ref, rhs.v_ref) &&
           same(lhs.w_control, rhs.w_control) &&
           same(lhs.w_accel, rhs.w_accel) &&
           same(lhs.w_smooth, rhs.w_smooth) &&
           same(lhs.w_alpha, rhs.w_alpha) &&
           same(lhs.w_du_a, rhs.w_du_a) &&
           same(lhs.w_du_vs, rhs.w_du_vs) &&
           lhs.slosh_cost_horizon_steps == rhs.slosh_cost_horizon_steps &&
           same(lhs.slosh_cost_tail_discount, rhs.slosh_cost_tail_discount);
}

}  // namespace spmpc_local_planner
