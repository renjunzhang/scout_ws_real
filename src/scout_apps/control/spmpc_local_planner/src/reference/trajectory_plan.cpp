#include "spmpc_local_planner/reference/trajectory_plan.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace spmpc_local_planner {
namespace {
using boost::property_tree::ptree;
void require(bool condition, const std::string& why) {
    if (!condition) throw std::invalid_argument("trajectory: " + why);
}
bool finite(double x) { return std::isfinite(x); }
bool near(double x, double y, double tolerance = 1e-6) { return std::abs(x-y) <= tolerance; }
template<size_t N> std::array<double, N> array(const ptree& root, const std::string& key) {
    const auto& values = root.get_child(key);
    require(values.size() == N, "wrong array size: " + key);
    std::array<double, N> result{};
    size_t i = 0;
    for (const auto& value : values) {
        require(value.first.empty(), "expected JSON array: " + key);
        result[i++] = value.second.get_value<double>();
    }
    return result;
}
}

TrajectoryPlan TrajectoryPlan::load(const std::string& path) {
    try {
        ptree root;
        boost::property_tree::read_json(path, root);
        TrajectoryPlan p;
        p.schema_version = root.get<int>("schema_version");
        p.liquid_model_version = root.get<int>("liquid_model_version");
        p.cost_model_version = root.get<int>("cost_model_version");
        p.plan_id = root.get<std::string>("plan_id");
        p.frame_id = root.get<std::string>("frame_id");
        p.region_id = root.get<std::string>("region_id");
        p.dt = root.get<double>("dt");
        p.transport_duration = root.get<double>("transport_duration");
        p.deadline = root.get<double>("deadline");
        p.stop_window = root.get<double>("stop_window");
        p.height_coeff = root.get<double>("height_coeff");
        p.actuator_parameters = array<4>(root, "actuator_parameters");
        p.liquid_parameters = array<4>(root, "liquid_parameters");
        const char* limits[] = {"actual_v_min", "v_max", "omega_max", "a_max", "alpha_max", "jerk_max"};
        for (size_t i = 0; i < 6; ++i)
            p.motion_limits[i] = root.get<double>(std::string("motion_limits.") + limits[i]);
        p.goal_pose = array<3>(root, "goal_pose");
        p.goal_position_tolerance = root.get<double>("goal_position_tolerance");
        p.goal_yaw_tolerance = root.get<double>("goal_yaw_tolerance");
        p.stop_speed_tolerance = root.get<double>("stop_speed_tolerance");
        p.stop_omega_tolerance = root.get<double>("stop_omega_tolerance");
        const auto& region=root.get_child("region");
        p.region.enabled=region.get<bool>("enabled",true);
        p.region.id=region.get<std::string>("id");
        p.region.frame_id=region.get<std::string>("frame_id");
        p.region.footprint_radius=region.get<double>("footprint_radius");
        p.region.margin=region.get<double>("margin");
        for (const auto& item:region.get_child("cells")) {
            MotionRegionCell cell;
            cell.id=item.second.get<std::string>("id");
            cell.s_begin=item.second.get<double>("s_begin");
            cell.s_end=item.second.get<double>("s_end");
            for (const auto& vertex:item.second.get_child("vertices")) {
                ptree point; point.add_child("xy",vertex.second);
                const auto xy=array<2>(point,"xy");
                cell.vertices.push_back({xy[0],xy[1]});
            }
            p.region.cells.push_back(cell);
        }
        for (const auto& item : root.get_child("route")) {
            require(item.first.empty(), "route must be an array");
            ptree point; point.add_child("xy", item.second);
            const auto xy = array<2>(point, "xy");
            p.route.push_back({xy[0], xy[1]});
        }
        for (const auto& item : root.get_child("samples")) {
            require(item.first.empty(), "samples must be an array");
            TrajectoryPlanSample sample;
            sample.t = item.second.get<double>("t");
            const auto x = array<28>(item.second, "state");
            sample.state.assign(x.begin(), x.end());
            sample.control = array<3>(item.second, "control");
            sample.phase = item.second.get<std::string>("phase");
            p.samples.push_back(sample);
        }
        p.validate();
        return p;
    } catch (const std::exception& e) {
        throw std::invalid_argument("cannot load trajectory " + path + ": " + e.what());
    }
}

void TrajectoryPlan::validate() const {
    require(schema_version == 1 && liquid_model_version == 1 && cost_model_version == 3,
            "unsupported schema/model version");
    require(!plan_id.empty() && !frame_id.empty() && !region_id.empty(), "missing identity");
    for (double x : {dt, transport_duration, deadline, stop_window, height_coeff})
        require(finite(x), "nonfinite scalar");
    require(dt > 0 && dt <= 1.0/30.0+1e-10 && transport_duration > 0 &&
            deadline >= transport_duration-1e-8 && stop_window > 0 && height_coeff > 0, "invalid timing/scales");
    for (double x : actuator_parameters) require(finite(x) && x > 0, "invalid actuator parameters");
    for (double x : liquid_parameters) require(finite(x), "nonfinite liquid parameters");
    require(liquid_parameters[0] >= 0 && liquid_parameters[1] > 0 &&
            liquid_parameters[2] > 0 && liquid_parameters[3] > 0, "invalid liquid parameters");
    for (size_t i = 0; i < motion_limits.size(); ++i)
        require(finite(motion_limits[i]) && (i ? motion_limits[i] > 0 : motion_limits[i] <= 0), "invalid motion limits");
    for (double x : goal_pose) require(finite(x), "nonfinite goal");
    for (double x : {goal_position_tolerance, goal_yaw_tolerance, stop_speed_tolerance, stop_omega_tolerance})
        require(finite(x) && x > 0, "invalid goal/stop tolerance");
    std::string region_error;
    require(region.enabled && region.id==region_id && region.frame_id==frame_id &&
            MotionRegion::validate(region,&region_error), "invalid region: "+region_error);
    require(route.size() >= 2, "route too short");
    double length = 0;
    for (size_t i = 0; i < route.size(); ++i) {
        require(finite(route[i].x) && finite(route[i].y), "nonfinite route");
        if (i) {
            const double ds = std::hypot(route[i].x-route[i-1].x, route[i].y-route[i-1].y);
            require(ds > 1e-9, "duplicate route points");
            length += ds;
        }
    }
    require(std::hypot(route.back().x-goal_pose[0], route.back().y-goal_pose[1]) <= goal_position_tolerance,
            "route and actual goal differ");
    require(samples.size() >= 2, "missing samples");
    bool seen_brake = false, seen_tail = false;
    for (size_t k = 0; k < samples.size(); ++k) {
        const auto& row = samples[k]; const auto& x = row.state; const auto& u = row.control;
        require(finite(row.t) && near(row.t, k*dt, 1e-7) && x.size() == 28, "sample grid/layout mismatch");
        for (double value : x) require(finite(value), "nonfinite state");
        for (double value : u) require(finite(value), "nonfinite control");
        require(row.phase == "MOVE" || row.phase == "WAIT" || row.phase == "BRAKE" || row.phase == "TAIL", "unknown phase");
        require(!seen_tail || row.phase == "TAIL", "tail cannot resume transport");
        require(!seen_brake || row.phase == "BRAKE" || row.phase == "TAIL", "braking cannot resume MOVE/WAIT");
        if (row.phase == "BRAKE") seen_brake = true;
        if (row.phase == "TAIL" && !seen_tail) {
            require(near(row.t, transport_duration, 1e-7), "TAIL start differs from transport_duration");
            seen_tail = true;
        }
        require(x[4] >= -1e-6 && x[4] <= length+1e-6, "progress outside original route");
        require(x[3] >= motion_limits[0]-1e-6 && x[3] <= motion_limits[1]+1e-6 &&
                std::abs(x[5]) <= motion_limits[2]+1e-6, "actual motion bound");
        require(x[6] >= -1e-6 && x[6] <= motion_limits[1]+1e-6 && std::abs(x[7]) <= motion_limits[2]+1e-6,
                "command bound");
        require(std::abs(u[0]) <= motion_limits[3]+1e-6 && std::abs(u[1]) <= motion_limits[4]+1e-6 &&
                u[2] >= -1e-6 && u[2] <= motion_limits[1]+1e-6, "control bound");
        if (k+1 < samples.size()) {
            require(std::abs(u[0]-x[23]) <= motion_limits[5]*dt+1e-6, "command jerk bound");
            const auto& next = samples[k+1].state;
            require(next.size() == 28, "next state layout");
            require(near(next[4], x[4]+dt*u[2]) && near(next[6], x[6]+dt*u[0]) &&
                    near(next[7], x[7]+dt*u[1]) && near(next[23], u[0]), "command/progress continuity");
            for (int i = 8; i < 12; ++i) require(near(next[i], x[i+1]), "linear FIFO shift");
            for (int i = 13; i < 22; ++i) require(near(next[i], x[i+1]), "angular FIFO shift");
            require(near(next[12], next[6]) && near(next[22], next[7]), "FIFO input differs from next command");
        }
        if (seen_tail) {
            require(std::hypot(x[0]-goal_pose[0], x[1]-goal_pose[1]) <= goal_position_tolerance+1e-6 &&
                    std::abs(std::atan2(std::sin(x[2]-goal_pose[2]), std::cos(x[2]-goal_pose[2]))) <= goal_yaw_tolerance+1e-6,
                    "tail misses goal pose");
            require(std::abs(x[3]) <= stop_speed_tolerance+1e-6 && std::abs(x[5]) <= stop_omega_tolerance+1e-6,
                    "tail actual motion not stopped");
            for (int i = 6; i <= 23; ++i) require(std::abs(x[i]) <= 1e-6, "tail commands/FIFO not clear");
            for (double value : u) require(std::abs(value) <= 1e-6, "tail control not zero");
        }
    }
    require(seen_tail && near(samples.back().t, deadline+stop_window, 1e-7), "wrong fixed evaluation window");
    require(near(samples.back().state[4], length), "final progress does not reach original route end");
    for (double value : samples.back().control) require(std::abs(value) <= 1e-8, "terminal control must be zero");
}

bool validateRoute(const std::vector<RegionVertex>& route, const std::string& frame,
                   double tolerance, const TrajectoryPlan& plan) {
    if (!finite(tolerance) || tolerance < 0 || frame != plan.frame_id || route.size() != plan.route.size()) return false;
    for (size_t i = 0; i < route.size(); ++i)
        if (!finite(route[i].x) || !finite(route[i].y) ||
            std::hypot(route[i].x-plan.route[i].x, route[i].y-plan.route[i].y) > tolerance) return false;
    return true;
}
}  // namespace spmpc_local_planner
