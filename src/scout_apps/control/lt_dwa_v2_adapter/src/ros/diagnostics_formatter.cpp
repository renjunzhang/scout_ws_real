#include "lt_dwa_v2_adapter/ros/diagnostics_formatter.h"

#include <iomanip>
#include <sstream>

namespace lt_dwa_v2_adapter
{
std::string formatDiagnostics(const RosDiagnosticsContext& context, const PlanResult& result)
{
  const auto& diagnostics = result.diagnostics;
  const auto& collision = diagnostics.initial_collision_details;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "status=" << result.status;
  ss << " plan_frame=" << (context.plan_frame.empty() ? std::string("<none>") : context.plan_frame);
  ss << " map_frame=" << (context.map_frame.empty() ? std::string("<none>") : context.map_frame);
  ss << " plan_map_transform_ok=" << (diagnostics.plan_map_transform_ok ? 1 : 0);
  ss << " path_size=" << context.path_size;
  ss << " tracker_progress_s=" << (context.have_last_progress ? context.tracker_progress_s : 0.0);
  ss << " state_plan_x=" << context.state.x;
  ss << " state_plan_y=" << context.state.y;
  ss << " state_plan_yaw=" << context.state.yaw;
  ss << " state_v=" << context.state.v;
  ss << " state_w=" << context.state.omega;
  ss << " raw_odom_frame=" << (context.raw_odom_frame.empty() ? std::string("<empty>") : context.raw_odom_frame);
  ss << " raw_odom_child=" << (context.raw_odom_child_frame.empty() ? std::string("<empty>") : context.raw_odom_child_frame);
  ss << " raw_odom_x=" << context.raw_odom_x;
  ss << " raw_odom_y=" << context.raw_odom_y;
  ss << " raw_odom_yaw=" << context.raw_odom_yaw;

  if (diagnostics.has_initial_match)
  {
    ss << " match_idx=" << diagnostics.initial_match_index;
    ss << " match_progress_s=" << diagnostics.initial_progress_s;
    ss << " match_dist=" << diagnostics.initial_match_distance;
    ss << " signed_lateral_err=" << diagnostics.initial_signed_lateral_error;
    ss << " match_heading_err=" << diagnostics.initial_match_heading_error;
  }
  ss << " target_idx=" << diagnostics.lookahead_target_index;
  ss << " target_progress_s=" << diagnostics.lookahead_target_progress_s;
  ss << " target_x=" << diagnostics.lookahead_target_x;
  ss << " target_y=" << diagnostics.lookahead_target_y;
  ss << " tracking_diverged=" << (diagnostics.tracking_diverged ? 1 : 0);
  ss << " max_tracking_deviation_m=" << diagnostics.max_tracking_deviation_m;
  ss << " initial_collision=" << (diagnostics.initial_collision ? 1 : 0);
  ss << " collision_samples=" << collision.checked_samples;
  ss << " collision_unknown=" << collision.unknown_samples;
  ss << " collision_out_of_map=" << collision.out_of_map_samples;
  ss << " collision_lethal=" << collision.lethal_samples;
  ss << " collision_center_occ=" << collision.center_occupancy;
  ss << " collision_max_occ=" << collision.max_occupancy;
  if (collision.has_first_lethal_sample)
  {
    ss << " first_lethal_x=" << collision.first_lethal_x;
    ss << " first_lethal_y=" << collision.first_lethal_y;
  }
  return ss.str();
}
}  // namespace lt_dwa_v2_adapter
