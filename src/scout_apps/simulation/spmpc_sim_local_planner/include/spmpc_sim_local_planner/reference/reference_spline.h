#pragma once

#include "spmpc_sim_local_planner/reference/reference_path.h"

namespace spmpc_sim_local_planner {

struct ReferenceSample {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;    // 路径切向航向
    double kappa = 0.0;  // 路径曲率
    double s = 0.0;
};

// 弧长参数化的连续参考，供连续 MPCC 的 contour / lag 与曲率项按 s 采样。
//
// 从 ReferencePath 构建，sample(s) 提供 [x, y, psi, kappa]。
// 当前以离散点 + 弧长有限差分给出 psi / kappa；continuous MPCC wrapper
// 会在每个 horizon 内另行拟合局部三次多项式并写入 acados 参数。
class ReferenceSpline {
public:
    void build(const ReferencePath& path);
    bool empty() const { return path_.empty(); }
    double length() const { return path_.length(); }
    ReferenceSample sample(double s) const;

private:
    ReferencePath path_;
};

}  // namespace spmpc_sim_local_planner
