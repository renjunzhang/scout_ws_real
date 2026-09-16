#include "spmpc_local_planner/ros/planning_config_ros.h"

#include <XmlRpcValue.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace spmpc_local_planner {
namespace {

double readNumber(const XmlRpc::XmlRpcValue& value, const std::string& path) {
  const auto type = value.getType();
  if (type != XmlRpc::XmlRpcValue::TypeInt && type != XmlRpc::XmlRpcValue::TypeDouble) {
    throw std::invalid_argument(path + " must be numeric");
  }
  const double result = type == XmlRpc::XmlRpcValue::TypeInt
      ? static_cast<int>(value) : static_cast<double>(value);
  if (!std::isfinite(result)) throw std::invalid_argument(path + " is not finite");
  return result;
}

bool readBoolean(const XmlRpc::XmlRpcValue& value, const std::string& path) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    throw std::invalid_argument(path + " must be bool");
  }
  return static_cast<bool>(value);
}

std::string readString(const XmlRpc::XmlRpcValue& value, const std::string& path) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::invalid_argument(path + " must be string");
  }
  return static_cast<std::string>(value);
}

// Optional parameters retain their C++ defaults when absent. getParam is the
// only lookup; callers never perform a hasParam/getParam pair.
bool getOptional(const ros::NodeHandle& node, const std::string& key,
                 XmlRpc::XmlRpcValue& value) {
  return node.getParam(key, value);
}

RegionVertex parseVertex(const XmlRpc::XmlRpcValue& value, const std::string& path) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 2) {
    throw std::invalid_argument(path + " must be [x, y]");
  }
  return {readNumber(value[0], path + "[0]"), readNumber(value[1], path + "[1]")};
}

MotionRegionCell parseRegionCell(const XmlRpc::XmlRpcValue& value, int index) {
  const std::string path = "planning/region/cells[" + std::to_string(index) + "]";
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !value.hasMember("id") || !value.hasMember("s_begin") ||
      !value.hasMember("s_end") || !value.hasMember("vertices")) {
    throw std::invalid_argument(path + " must contain id, s_begin, s_end, and vertices");
  }

  MotionRegionCell cell;
  cell.id = readString(value["id"], path + ".id");
  cell.s_begin = readNumber(value["s_begin"], path + ".s_begin");
  cell.s_end = readNumber(value["s_end"], path + ".s_end");
  const XmlRpc::XmlRpcValue& vertices = value["vertices"];
  if (vertices.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument(path + ".vertices must be an array");
  }
  cell.vertices.reserve(vertices.size());
  for (int i = 0; i < vertices.size(); ++i) {
    const std::string vertex_path = path + ".vertices[" + std::to_string(i) + "]";
    cell.vertices.push_back(parseVertex(vertices[i], vertex_path));
  }
  return cell;
}

void readNumberIfPresent(const ros::NodeHandle& node, const std::string& key, double& target) {
  XmlRpc::XmlRpcValue value;
  if (getOptional(node, key, value)) target = readNumber(value, key);
}

void readBooleanIfPresent(const ros::NodeHandle& node, const std::string& key, bool& target) {
  XmlRpc::XmlRpcValue value;
  if (getOptional(node, key, value)) target = readBoolean(value, key);
}

void readStringIfPresent(const ros::NodeHandle& node, const std::string& key, std::string& target) {
  XmlRpc::XmlRpcValue value;
  if (getOptional(node, key, value)) target = readString(value, key);
}

}  // namespace

PlanningConfig loadPlanningConfig(const ros::NodeHandle& node) {
  PlanningConfig config;
  readStringIfPresent(node, "planning/experiment_profile_id", config.experiment_profile_id);
  readBooleanIfPresent(node, "planning/liquid_free_baseline", config.liquid_free_baseline);
  readNumberIfPresent(node, "planning/task_deadline_sec", config.task_deadline_sec);
  readNumberIfPresent(node, "planning/evaluation_window_sec", config.evaluation_window_sec);
  readNumberIfPresent(node, "planning/projection/lookahead", config.projection_lookahead);

  readBooleanIfPresent(node, "planning/geometry/enabled", config.geometry.enabled);
  readNumberIfPresent(node, "planning/geometry/curvature_weight", config.geometry.curvature_weight);
  readNumberIfPresent(node, "planning/geometry/curvature_rate_weight", config.geometry.curvature_rate_weight);
  readNumberIfPresent(node, "planning/geometry/curvature_scale", config.geometry.curvature_scale);
  readNumberIfPresent(node, "planning/geometry/curvature_rate_scale", config.geometry.curvature_rate_scale);
  readNumberIfPresent(node, "planning/geometry/speed_regularization", config.geometry.speed_regularization);
  readNumberIfPresent(node, "planning/geometry/goal_weight", config.geometry.goal_weight);
  readNumberIfPresent(node, "planning/geometry/goal_position_scale", config.geometry.goal_position_scale);
  readNumberIfPresent(node, "planning/geometry/goal_yaw_scale", config.geometry.goal_yaw_scale);
  readBooleanIfPresent(node, "planning/geometry/reference_curvature_speed_limit",
                       config.geometry.reference_curvature_speed_limit);

  XmlRpc::XmlRpcValue value;
  if (getOptional(node, "planning/reference/mode", value)) {
    config.trajectory.mode = parseTrajectoryReferenceMode(readString(value, "planning/reference/mode"));
  }
  readStringIfPresent(node, "planning/reference/plan_file", config.trajectory.plan_file);
  readNumberIfPresent(node, "planning/reference/progress_window", config.trajectory.progress_window);
  readNumberIfPresent(node, "planning/reference/max_speed_error", config.trajectory.max_speed_error);
  readNumberIfPresent(node, "planning/reference/route_tolerance", config.trajectory.route_tolerance);
  readNumberIfPresent(node, "planning/reference/deadline_tolerance", config.trajectory.deadline_tolerance);

  readBooleanIfPresent(node, "planning/region/enabled", config.region.enabled);
  readStringIfPresent(node, "planning/region/id", config.region.id);
  readStringIfPresent(node, "planning/region/frame_id", config.region.frame_id);
  readNumberIfPresent(node, "planning/region/footprint_radius", config.region.footprint_radius);
  readNumberIfPresent(node, "planning/region/margin", config.region.margin);
  if (getOptional(node, "planning/region/cells", value)) {
    if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::invalid_argument("planning/region/cells must be an array");
    }
    config.region.cells.clear();
    config.region.cells.reserve(value.size());
    for (int i = 0; i < value.size(); ++i) {
      config.region.cells.push_back(parseRegionCell(value[i], i));
    }
  }

  std::string reason;
  if (!validatePlanningConfig(config, &reason)) {
    throw std::invalid_argument("invalid planning config: " + reason);
  }
  return config;
}

}  // namespace spmpc_local_planner
