/**
 * @file dynamics_model.h
 * @brief 动力学模型基类（全向轮版本）
 */

#pragma once

#include "scout_omni_local_planner/types.h"
#include <Eigen/Dense>
#include <memory>

namespace scout_omni_local_planner {

class DynamicsModelBase {
public:
    virtual ~DynamicsModelBase() = default;
    virtual int stateDim() const = 0;
    virtual int controlDim() const = 0;
    
    virtual StateVector predict(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt) const = 0;
    
    virtual void linearize(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt,
        Eigen::MatrixXd& A,
        Eigen::MatrixXd& B,
        Eigen::VectorXd& c) const = 0;
    
    virtual std::string name() const = 0;
};

using DynamicsModelPtr = std::shared_ptr<DynamicsModelBase>;

}  // namespace scout_omni_local_planner
