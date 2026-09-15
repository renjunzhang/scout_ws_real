#pragma once
#include "spmpc_local_planner/reference/trajectory_plan.h"
#include "spmpc_local_planner/dynamics/explicit_state_rollout.h"
#include <cmath>
#include <stdexcept>

inline spmpc_local_planner::TrajectoryPlan movingPlanFixture() {
    using namespace spmpc_local_planner;
    TrajectoryPlan p;
    p.plan_id="test-motion"; p.frame_id="map"; p.region_id="room";
    p.dt=1./30; p.transport_duration=4.; p.deadline=4.; p.stop_window=1.;
    p.goal_position_tolerance=.05; p.goal_yaw_tolerance=.1;
    p.stop_speed_tolerance=.01; p.stop_omega_tolerance=.02;
    ActuatorModelParams actuator;
    SloshDynamics liquid;
    if (!liquid.configure(SloshModelParams{})) throw std::runtime_error("fixture liquid config");
    p.actuator_parameters={{actuator.linear_tau_sec, actuator.angular_tau_sec, actuator.linear_gain, actuator.angular_gain}};
    p.liquid_parameters=liquid.coefficients().values(); p.height_coeff=liquid.heightCoeff();
    p.motion_limits={{-.002,.8,1.2,.6,1.2,1.}};
    std::vector<double> x(28,0);
    for (int k=0; k<=150; ++k) {
        std::array<double,3> u{};
        std::vector<double> next;
        if (k<150) {
            const double desired = k+1<90 ? .12*std::pow(std::sin(M_PI*(k+1)/90.),2) : 0.;
            u[0]=(desired-x[6])/p.dt;
            if (!stepExplicitState(x,u,actuator,liquid,p.dt,next)) throw std::runtime_error("fixture rollout");
            u[2]=k<120 ? (next[0]-x[0])/p.dt : 0.;
            next[4]=x[4]+p.dt*u[2];
        }
        p.samples.push_back({k*p.dt,x,u,k>=120?"TAIL":(k>=45?"BRAKE":"MOVE")});
        x=std::move(next);
    }
    p.goal_pose={{p.samples[120].state[0],0,0}};
    p.route={{0,0},{p.samples.back().state[4],0}};
    p.region.enabled=true; p.region.id=p.region_id; p.region.frame_id=p.frame_id;
    p.region.cells.push_back({"room",0,p.route.back().x,{{-1,-1},{2,-1},{2,1},{-1,1}}});
    p.validate();
    return p;
}
