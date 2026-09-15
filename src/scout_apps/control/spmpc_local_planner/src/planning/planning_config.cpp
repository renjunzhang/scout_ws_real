#include "spmpc_local_planner/planning/planning_config.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace spmpc_local_planner {

const char* trajectoryReferenceModeName(TrajectoryReferenceMode mode) {
    switch (mode) {
    case TrajectoryReferenceMode::Cruise: return "cruise";
    case TrajectoryReferenceMode::Progress: return "progress";
    case TrajectoryReferenceMode::FixedTime: return "fixed_time";
    }
    return "invalid";
}

TrajectoryReferenceMode parseTrajectoryReferenceMode(const std::string& value) {
    if (value == "cruise") return TrajectoryReferenceMode::Cruise;
    if (value == "progress") return TrajectoryReferenceMode::Progress;
    if (value == "fixed_time") return TrajectoryReferenceMode::FixedTime;
    throw std::invalid_argument("unknown trajectory reference mode: " + value);
}

bool validatePlanningConfig(const PlanningConfig& c, std::string* reason) {
    auto fail = [reason](const std::string& value) { if (reason) *reason = value; return false; };
    const auto& g = c.geometry;
    if (c.experiment_profile_id.empty()) return fail("empty experiment_profile_id");
    if (!std::isfinite(c.task_deadline_sec) || c.task_deadline_sec < 0 ||
        !std::isfinite(c.evaluation_window_sec) || c.evaluation_window_sec <= 0)
        return fail("invalid task deadline/evaluation window");
    for (double value : {g.curvature_weight, g.curvature_rate_weight, g.goal_weight})
        if (!std::isfinite(value) || value < 0) return fail("invalid geometry weight");
    for (double value : {g.curvature_scale, g.curvature_rate_scale, g.speed_regularization, g.goal_position_scale, g.goal_yaw_scale,
                         c.trajectory.progress_window, c.trajectory.max_speed_error,
                         c.trajectory.route_tolerance})
        if (!std::isfinite(value) || value <= 0) return fail("invalid positive planning scale");
    if (!std::isfinite(c.trajectory.deadline_tolerance) || c.trajectory.deadline_tolerance < 0)
        return fail("invalid deadline tolerance");
    if (c.trajectory.mode != TrajectoryReferenceMode::Cruise &&
        c.trajectory.mode != TrajectoryReferenceMode::Progress &&
        c.trajectory.mode != TrajectoryReferenceMode::FixedTime) return fail("invalid reference mode");
    if (c.trajectory.mode != TrajectoryReferenceMode::Cruise && c.trajectory.plan_file.empty())
        return fail("trajectory reference requested without plan_file");
    if (g.enabled && !c.region.enabled) return fail("autonomous geometry requires an explicit motion region");
    if (c.liquid_free_baseline && c.trajectory.mode != TrajectoryReferenceMode::Cruise)
        return fail("raw-path baseline cannot consume a trajectory plan");
    if (c.liquid_free_baseline && (g.enabled || g.curvature_weight!=0 || g.curvature_rate_weight!=0 || g.goal_weight!=0))
        return fail("raw-path baseline cannot enable new geometry objectives");
    if (!MotionRegion::validate(c.region, reason)) return false;
    if (reason) reason->clear();
    return true;
}

std::string planningConfigJson(const PlanningConfig& c) {
    using boost::property_tree::ptree;
    ptree root;
    root.put("schema_version", 1);
    root.put("experiment_profile_id", c.experiment_profile_id);
    root.put("liquid_free_baseline", c.liquid_free_baseline);
    root.put("task_deadline_sec", c.task_deadline_sec);
    root.put("evaluation_window_sec", c.evaluation_window_sec);
    root.put("geometry.enabled", c.geometry.enabled);
    root.put("geometry.curvature_weight", c.geometry.curvature_weight);
    root.put("geometry.curvature_rate_weight", c.geometry.curvature_rate_weight);
    root.put("geometry.curvature_scale", c.geometry.curvature_scale);
    root.put("geometry.curvature_rate_scale", c.geometry.curvature_rate_scale);
    root.put("geometry.speed_regularization", c.geometry.speed_regularization);
    root.put("geometry.goal_weight", c.geometry.goal_weight);
    root.put("geometry.goal_position_scale", c.geometry.goal_position_scale);
    root.put("geometry.goal_yaw_scale", c.geometry.goal_yaw_scale);
    root.put("geometry.reference_curvature_speed_limit", c.geometry.reference_curvature_speed_limit);
    root.put("reference.mode", trajectoryReferenceModeName(c.trajectory.mode));
    root.put("reference.plan_file", c.trajectory.plan_file);
    root.put("reference.progress_window", c.trajectory.progress_window);
    root.put("reference.max_speed_error", c.trajectory.max_speed_error);
    root.put("reference.route_tolerance", c.trajectory.route_tolerance);
    root.put("reference.deadline_tolerance", c.trajectory.deadline_tolerance);
    root.put("region.enabled", c.region.enabled);
    root.put("region.id", c.region.id);
    root.put("region.frame_id", c.region.frame_id);
    root.put("region.footprint_radius", c.region.footprint_radius);
    root.put("region.margin", c.region.margin);
    ptree cells;
    for (const auto& cell : c.region.cells) {
        ptree row, vertices;
        row.put("id", cell.id); row.put("s_begin", cell.s_begin); row.put("s_end", cell.s_end);
        for (const auto& v : cell.vertices) {
            ptree point; point.put("x", v.x); point.put("y", v.y);
            vertices.push_back({"", point});
        }
        row.add_child("vertices", vertices); cells.push_back({"", row});
    }
    root.add_child("region.cells", cells);
    std::ostringstream out;
    boost::property_tree::write_json(out, root, false);
    return out.str();
}

}  // namespace spmpc_local_planner
