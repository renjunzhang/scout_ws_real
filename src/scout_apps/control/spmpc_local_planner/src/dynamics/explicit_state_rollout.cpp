#include "spmpc_local_planner/dynamics/explicit_state_rollout.h"
#include <cmath>

namespace spmpc_local_planner {
bool stepExplicitState(const std::vector<double>& x, const std::array<double, 3>& u,
    const ActuatorModelParams& actuator, const SloshDynamics& liquid, double dt, std::vector<double>& next) {
    if ((x.size() != 24 && x.size() != 28) || !std::isfinite(dt) || std::abs(dt-actuator.dt) > 1e-8) return false;
    for (double v : x) if (!std::isfinite(v)) return false;
    for (double v : u) if (!std::isfinite(v)) return false;
    RobotState robot{x[0], x[1], x[2], x[3], x[5]};
    SloshState slosh;
    if (x.size() == 28) { slosh.eta_x=x[24]; slosh.eta_x_dot=x[25]; slosh.eta_y=x[26]; slosh.eta_y_dot=x[27]; }
    if (!propagateActualMotion(robot, slosh, {x[8], x[13]}, actuator, liquid, dt)) return false;
    auto result = x;
    result[0]=robot.x; result[1]=robot.y; result[2]=robot.yaw; result[3]=robot.v;
    result[4]=x[4]+dt*u[2]; result[5]=robot.omega;
    result[6]=x[6]+dt*u[0]; result[7]=x[7]+dt*u[1]; result[23]=u[0];
    for (int i=8; i<12; ++i) result[i]=x[i+1];
    result[12]=result[6];
    for (int i=13; i<22; ++i) result[i]=x[i+1];
    result[22]=result[7];
    if (x.size()==28) { result[24]=slosh.eta_x; result[25]=slosh.eta_x_dot; result[26]=slosh.eta_y; result[27]=slosh.eta_y_dot; }
    next = std::move(result);
    return true;
}
}  // namespace spmpc_local_planner
