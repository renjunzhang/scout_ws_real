#include "spmpc_local_planner/core/variant_config.h"

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
               variant_name == "B_slosh_linear" || variant_name == "B_slosh_anti" ||
               variant_name == "B_smooth" || variant_name == "B_ours" ||
               variant_name == "B_ours_anti") {
        cfg.name = variant_name;
    } else {
        cfg.name = "B0";
    }
    return cfg;
}

}  // namespace spmpc_local_planner
