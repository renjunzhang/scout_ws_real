#ifndef SPMPC_LOCAL_PLANNER_REFERENCE_MOTION_REGION_H
#define SPMPC_LOCAL_PLANNER_REFERENCE_MOTION_REGION_H

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace spmpc_local_planner {

constexpr std::size_t kMaxRegionFaces = 8;
// Numerical tolerance for progress coordinates only. Spatial halfspaces and
// footprint inflation are unchanged by this tolerance.
constexpr double kProgressTolerance = 1e-8;

struct RegionVertex { double x = 0.0; double y = 0.0; };
struct RegionHalfspace { double nx = 0.0; double ny = 0.0; double offset = 1e12; };
struct MotionRegionCell {
  std::string id;
  double s_begin = 0.0;
  double s_end = 0.0;
  std::vector<RegionVertex> vertices;
};
struct MotionRegionConfig {
  bool enabled = false;
  std::string id;
  std::string frame_id = "map";
  double footprint_radius = 0.45;
  double margin = 0.02;
  std::vector<MotionRegionCell> cells;
};
struct RegionStageData {
  bool enabled = false;
  std::string cell_id;
  double s_begin = 0.0;
  double s_end = 1e12;
  std::array<RegionHalfspace, kMaxRegionFaces> faces{};
};

class MotionRegion {
 public:
  explicit MotionRegion(MotionRegionConfig config);
  static bool validate(const MotionRegionConfig& config, std::string* reason = nullptr);
  const MotionRegionConfig& config() const { return config_; }
  RegionStageData stage(double route_s, double sweep_distance) const;
  RegionStageData stageForCell(std::size_t cell_index, double sweep_distance) const;
  double clearance(double x, double y, double route_s) const;

 private:
  MotionRegionConfig config_;
};

}  // namespace spmpc_local_planner

#endif
