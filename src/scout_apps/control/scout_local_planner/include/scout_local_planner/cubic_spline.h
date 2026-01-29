/**
 * @file cubic_spline.h
 * @brief 三次样条插值
 * 
 * 用于对离散路径点进行局部样条拟合，计算曲率和切向角
 */

#pragma once

#include <vector>
#include <Eigen/Dense>

namespace scout_local_planner {

/**
 * @brief 三次样条插值类
 * 
 * 支持自然边界条件的三次样条插值
 * 可计算任意参数位置的位置、一阶导数、二阶导数
 */
class CubicSpline {
public:
    CubicSpline() = default;
    
    /**
     * @brief 使用点序列初始化样条
     * @param x X坐标序列
     * @param y Y坐标序列
     * @return 是否成功（点数需 >= 2）
     */
    bool fit(const std::vector<double>& x, const std::vector<double>& y);
    
    /**
     * @brief 使用 2D 点序列初始化样条（参数化）
     * @param points 2D点序列
     * @return 是否成功
     */
    bool fitParametric(const std::vector<Eigen::Vector2d>& points);
    
    /**
     * @brief 计算指定参数位置的值
     * @param t 参数（弧长归一化）
     * @return 插值点的值
     */
    double evaluate(double t) const;
    
    /**
     * @brief 计算指定参数位置的一阶导数
     */
    double evaluateDerivative(double t) const;
    
    /**
     * @brief 计算指定参数位置的二阶导数
     */
    double evaluateSecondDerivative(double t) const;
    
    /**
     * @brief 获取总弧长
     */
    double getTotalLength() const { return total_length_; }
    
    /**
     * @brief 是否已初始化
     */
    bool isValid() const { return valid_; }

private:
    // 查找参数 t 所在的区间索引
    int findSegment(double t) const;
    
    bool valid_ = false;
    
    // 节点参数（累积弧长）
    std::vector<double> t_;
    
    // 样条系数（对于每个区间 [t_i, t_{i+1}]）
    // y(t) = a_i + b_i*(t-t_i) + c_i*(t-t_i)^2 + d_i*(t-t_i)^3
    std::vector<double> a_, b_, c_, d_;
    
    double total_length_ = 0.0;
};

/**
 * @brief 2D 参数化样条（用于路径）
 * 
 * 分别对 x(s) 和 y(s) 进行样条插值
 * 支持计算曲率和切向角
 */
class CubicSpline2D {
public:
    CubicSpline2D() = default;
    
    /**
     * @brief 使用 2D 点序列初始化
     * @param points 2D点序列
     * @return 是否成功
     */
    bool fit(const std::vector<Eigen::Vector2d>& points);
    
    /**
     * @brief 计算指定弧长位置的点
     * @param s 弧长参数
     * @return 插值点
     */
    Eigen::Vector2d evaluate(double s) const;
    
    /**
     * @brief 计算指定弧长位置的切向角
     * @param s 弧长参数
     * @return 切向角 θ (rad)
     */
    double evaluateTheta(double s) const;
    
    /**
     * @brief 计算指定弧长位置的曲率
     * @param s 弧长参数
     * @return 曲率 κ (1/m)
     */
    double evaluateKappa(double s) const;
    
    /**
     * @brief 获取总弧长
     */
    double getTotalLength() const { return total_length_; }
    
    /**
     * @brief 是否已初始化
     */
    bool isValid() const { return spline_x_.isValid() && spline_y_.isValid(); }

private:
    CubicSpline spline_x_;
    CubicSpline spline_y_;
    double total_length_ = 0.0;
};

}  // namespace scout_local_planner
