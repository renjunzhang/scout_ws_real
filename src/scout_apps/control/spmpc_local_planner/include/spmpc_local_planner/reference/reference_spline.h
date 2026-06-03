#pragma once

#include "spmpc_local_planner/reference/reference_path.h"

namespace spmpc_local_planner {

struct ReferenceSample {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;    // 路径切向航向
    double kappa = 0.0;  // 路径曲率
    double s = 0.0;
};

// 弧长参数化的连续参考，供连续 MPCC 的 contour / lag 与曲率项按 s 采样。
//
// Phase A：从 ReferencePath 构建，sample(s) 提供 [x, y, psi, kappa]。
// 当前以离散点 + 弧长有限差分给出 psi / kappa（骨架版本，primitive 后端不依赖它）。
// 待 acados 包装层（Phase B/C）明确其对解析导数 / 多项式系数的需求后，
// 再决定是否升级为 C2 三次样条。
class ReferenceSpline {
public:
    void build(const ReferencePath& path);
    bool empty() const { return path_.empty(); }
    double length() const { return path_.length(); }
    ReferenceSample sample(double s) const;

private:
    ReferencePath path_;
};

}  // namespace spmpc_local_planner
