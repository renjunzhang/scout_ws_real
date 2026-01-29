/**
 * @file cubic_spline.cpp
 * @brief 三次样条插值实现
 */

#include "scout_local_planner/cubic_spline.h"
#include <cmath>
#include <algorithm>

namespace scout_local_planner {

//==============================================================================
// CubicSpline 实现
//==============================================================================

bool CubicSpline::fit(const std::vector<double>& t, const std::vector<double>& y) {
    valid_ = false;
    
    if (t.size() != y.size() || t.size() < 2) {
        return false;
    }
    
    const int n = static_cast<int>(t.size());
    
    // 保存节点参数
    t_ = t;
    total_length_ = t.back() - t.front();
    
    // 特殊情况：只有两个点，使用线性插值
    if (n == 2) {
        a_.resize(1);
        b_.resize(1);
        c_.resize(1);
        d_.resize(1);
        
        a_[0] = y[0];
        b_[0] = (y[1] - y[0]) / (t[1] - t[0]);
        c_[0] = 0.0;
        d_[0] = 0.0;
        
        valid_ = true;
        return true;
    }
    
    // 计算步长 h
    std::vector<double> h(n - 1);
    for (int i = 0; i < n - 1; ++i) {
        h[i] = t[i + 1] - t[i];
        if (h[i] <= 0) {
            return false;  // 参数必须单调递增
        }
    }
    
    // 构建三对角矩阵系统求解二阶导数（自然边界条件）
    // A * M = b，其中 M[i] = y''(t_i)
    std::vector<double> alpha(n - 1);
    for (int i = 1; i < n - 1; ++i) {
        alpha[i] = 3.0 / h[i] * (y[i + 1] - y[i]) 
                 - 3.0 / h[i - 1] * (y[i] - y[i - 1]);
    }
    
    // 追赶法求解三对角系统
    std::vector<double> l(n), mu(n), z(n);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    
    for (int i = 1; i < n - 1; ++i) {
        l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    
    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    
    // 回代
    std::vector<double> M(n);  // 二阶导数
    M[n - 1] = 0.0;
    for (int j = n - 2; j >= 0; --j) {
        M[j] = z[j] - mu[j] * M[j + 1];
    }
    
    // 计算样条系数
    a_.resize(n - 1);
    b_.resize(n - 1);
    c_.resize(n - 1);
    d_.resize(n - 1);
    
    for (int i = 0; i < n - 1; ++i) {
        a_[i] = y[i];
        b_[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (M[i + 1] + 2.0 * M[i]) / 3.0;
        c_[i] = M[i];
        d_[i] = (M[i + 1] - M[i]) / (3.0 * h[i]);
    }
    
    valid_ = true;
    return true;
}

int CubicSpline::findSegment(double t) const {
    if (t_.empty()) return 0;
    
    // 边界处理
    if (t <= t_.front()) return 0;
    if (t >= t_.back()) return static_cast<int>(t_.size()) - 2;
    
    // 二分查找
    auto it = std::upper_bound(t_.begin(), t_.end(), t);
    return static_cast<int>(std::distance(t_.begin(), it)) - 1;
}

double CubicSpline::evaluate(double t) const {
    if (!valid_) return 0.0;
    
    int i = findSegment(t);
    i = std::max(0, std::min(i, static_cast<int>(a_.size()) - 1));
    
    double dt = t - t_[i];
    return a_[i] + b_[i] * dt + c_[i] * dt * dt + d_[i] * dt * dt * dt;
}

double CubicSpline::evaluateDerivative(double t) const {
    if (!valid_) return 0.0;
    
    int i = findSegment(t);
    i = std::max(0, std::min(i, static_cast<int>(b_.size()) - 1));
    
    double dt = t - t_[i];
    return b_[i] + 2.0 * c_[i] * dt + 3.0 * d_[i] * dt * dt;
}

double CubicSpline::evaluateSecondDerivative(double t) const {
    if (!valid_) return 0.0;
    
    int i = findSegment(t);
    i = std::max(0, std::min(i, static_cast<int>(c_.size()) - 1));
    
    double dt = t - t_[i];
    return 2.0 * c_[i] + 6.0 * d_[i] * dt;
}

//==============================================================================
// CubicSpline2D 实现
//==============================================================================

bool CubicSpline2D::fit(const std::vector<Eigen::Vector2d>& points) {
    if (points.size() < 2) {
        return false;
    }
    
    const int n = static_cast<int>(points.size());
    
    // 计算累积弧长作为参数
    std::vector<double> s(n);
    s[0] = 0.0;
    for (int i = 1; i < n; ++i) {
        s[i] = s[i - 1] + (points[i] - points[i - 1]).norm();
    }
    total_length_ = s.back();
    
    // 提取 x, y 坐标
    std::vector<double> x(n), y(n);
    for (int i = 0; i < n; ++i) {
        x[i] = points[i].x();
        y[i] = points[i].y();
    }
    
    // 分别拟合 x(s) 和 y(s)
    if (!spline_x_.fit(s, x) || !spline_y_.fit(s, y)) {
        return false;
    }
    
    return true;
}

Eigen::Vector2d CubicSpline2D::evaluate(double s) const {
    return Eigen::Vector2d(spline_x_.evaluate(s), spline_y_.evaluate(s));
}

double CubicSpline2D::evaluateTheta(double s) const {
    double dx = spline_x_.evaluateDerivative(s);
    double dy = spline_y_.evaluateDerivative(s);
    return std::atan2(dy, dx);
}

double CubicSpline2D::evaluateKappa(double s) const {
    // 曲率公式: κ = (x'y'' - y'x'') / (x'^2 + y'^2)^(3/2)
    double dx = spline_x_.evaluateDerivative(s);
    double dy = spline_y_.evaluateDerivative(s);
    double ddx = spline_x_.evaluateSecondDerivative(s);
    double ddy = spline_y_.evaluateSecondDerivative(s);
    
    double denom = std::pow(dx * dx + dy * dy, 1.5);
    if (denom < 1e-10) {
        return 0.0;  // 避免除零
    }
    
    return (dx * ddy - dy * ddx) / denom;
}

}  // namespace scout_local_planner
