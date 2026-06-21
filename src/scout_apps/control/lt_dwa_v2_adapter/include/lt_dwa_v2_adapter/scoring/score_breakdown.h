#pragma once

namespace lt_dwa_v2_adapter
{
struct ScoreBreakdown
{
  double obstacle = 0.0;
  double path_lateral = 0.0;
  double heading = 0.0;
  double progress = 0.0;
  double terminal = 0.0;
  double smooth = 0.0;
  double speed = 0.0;
};
}  // namespace lt_dwa_v2_adapter
