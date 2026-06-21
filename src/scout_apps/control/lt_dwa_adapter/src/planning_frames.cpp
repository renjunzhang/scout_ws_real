#include "lt_dwa_adapter/planning_frames.h"

#include <cmath>

namespace lt_dwa_adapter
{
namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}
}  // namespace

PlanningTransform2D identityTransform(const std::string& source_frame, const std::string& target_frame)
{
  PlanningTransform2D transform;
  transform.valid = true;
  transform.source_frame = source_frame;
  transform.target_frame = target_frame;
  return transform;
}

Pose2D transformPose(const Pose2D& pose, const PlanningTransform2D& transform)
{
  Pose2D out = pose;
  if (!transform.valid)
    return out;
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  out.x = transform.x + c * pose.x - s * pose.y;
  out.y = transform.y + s * pose.x + c * pose.y;
  out.yaw = normalizeAngle(pose.yaw + transform.yaw);
  return out;
}

RobotState transformState(const RobotState& state, const PlanningTransform2D& transform)
{
  RobotState out = state;
  if (!transform.valid)
    return out;
  const double c = std::cos(transform.yaw);
  const double s = std::sin(transform.yaw);
  out.x = transform.x + c * state.x - s * state.y;
  out.y = transform.y + s * state.x + c * state.y;
  out.yaw = normalizeAngle(state.yaw + transform.yaw);
  return out;
}
}  // namespace lt_dwa_adapter
