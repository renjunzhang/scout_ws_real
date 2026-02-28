/**
 * @file cubic_spline.h
 * @brief 三次样条插值
 * 
 * 用于对离散路径点进行局部样条拟合，计算曲率和切向角
 */

#pragma once

#include <vector>
#include <Eigen/Dense>

namespace scout_omni_local_planner {

/**
 * @brief 三次样条插值类
 * 
 * 支持自然边界条件的三次样条插值
 * 可计算任意参数位置的位置、一阶导数、二阶导数
 */
class CubicSpline {
public:
    CubicSpline() = default;
    
    bool fit(const std::vector<double>& x, const std::vector<double>& y);
    bool fitParametric(const std::vector<Eigen::Vector2d>& points);
    
    double evaluate(double t) const;
    double evaluateDerivative(double t) const;
    double evaluateSecondDerivative(double t) const;
    
    double getTotalLength() const { return total_length_; }
    bool isValid() const { return valid_; }

private:
    int findSegment(double t) const;
    
    bool valid_ = false;
    std::vector<double> t_;
    std::vector<double> a_, b_, c_, d_;
    double total_length_ = 0.0;
};

/**
 * @brief 2D 参数化样条（用于路径）
 */
class CubicSpline2D {
public:
    CubicSpline2D() = default;
    
    bool fit(const std::vector<Eigen::Vector2d>& points);
    Eigen::Vector2d evaluate(double s) const;
    double evaluateTheta(double s) const;
    double evaluateKappa(double s) const;
    
    double getTotalLength() const { return total_length_; }
    bool isValid() const { return spline_x_.isValid() && spline_y_.isValid(); }

private:
    CubicSpline spline_x_;
    CubicSpline spline_y_;
    double total_length_ = 0.0;
};

}  // namespace scout_omni_local_planner
