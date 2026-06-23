#include "lt_dwa_official_wrapper/worker_request.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace lt_dwa_official_wrapper {
namespace {

constexpr const char* kMagic = "LT_DWA_WORKER_REQUEST_V1";

ros::Time TimeFromDouble(double seconds) {
  if (seconds <= 0.0 || !std::isfinite(seconds)) {
    return ros::Time();
  }
  return ros::Time(seconds);
}

Pose2d ReadPose(std::istringstream& iss) {
  Pose2d pose;
  double stamp_sec = 0.0;
  iss >> pose.frame_id >> stamp_sec >> pose.x >> pose.y >> pose.yaw;
  pose.stamp = TimeFromDouble(stamp_sec);
  return pose;
}

void AppendPose(std::ostringstream& oss, const std::string& key, const Pose2d& pose) {
  oss << key << ' ' << pose.frame_id << ' ' << pose.stamp.toSec() << ' '
      << pose.x << ' ' << pose.y << ' ' << pose.yaw << '\n';
}

bool ReadNextMeaningfulLine(std::istream& input, std::string* line) {
  while (std::getline(input, *line)) {
    if (line->empty()) {
      continue;
    }
    if ((*line)[0] == '#') {
      continue;
    }
    return true;
  }
  return false;
}

WorkerRequestParseResult Fail(const std::string& reason) {
  WorkerRequestParseResult result;
  result.ok = false;
  result.reason = reason;
  return result;
}

}  // namespace

std::string SerializeWorkerRequest(const PlannerInput& input, const ros::Time& now) {
  std::ostringstream oss;
  oss << std::setprecision(17);
  oss << kMagic << '\n';
  oss << "now " << now.toSec() << '\n';
  oss << "planning_frame " << input.planning_frame << '\n';
  oss << "stamp " << input.stamp.toSec() << '\n';
  AppendPose(oss, "robot_pose", input.robot_pose);
  oss << "robot_twist " << input.robot_twist.v << ' ' << input.robot_twist.w << '\n';
  AppendPose(oss, "target_pose", input.target_pose);

  oss << "path_count " << input.reference_path.size() << '\n';
  for (const auto& pose : input.reference_path) {
    AppendPose(oss, "path", pose);
  }

  const double origin_yaw = 2.0 * std::atan2(input.occupancy_grid.info.origin.orientation.z,
                                             input.occupancy_grid.info.origin.orientation.w);
  oss << "map " << input.occupancy_grid.header.frame_id << ' '
      << input.occupancy_grid.header.stamp.toSec() << ' '
      << input.occupancy_grid.info.width << ' '
      << input.occupancy_grid.info.height << ' '
      << input.occupancy_grid.info.resolution << ' '
      << input.occupancy_grid.info.origin.position.x << ' '
      << input.occupancy_grid.info.origin.position.y << ' '
      << origin_yaw << '\n';
  oss << "map_data " << input.occupancy_grid.data.size();
  for (const auto value : input.occupancy_grid.data) {
    oss << ' ' << static_cast<int>(value);
  }
  oss << '\n';

  oss << "obstacle_count " << input.dynamic_obstacles.size() << '\n';
  for (const auto& obstacle : input.dynamic_obstacles) {
    oss << "obstacle " << obstacle.id << ' ' << obstacle.frame_id << ' '
        << obstacle.stamp.toSec() << ' ' << obstacle.x << ' ' << obstacle.y << ' '
        << obstacle.vx << ' ' << obstacle.vy << ' ' << obstacle.radius << '\n';
  }
  return oss.str();
}

WorkerRequestParseResult ParseWorkerRequestText(const std::string& text) {
  std::istringstream input(text);
  std::string line;
  if (!ReadNextMeaningfulLine(input, &line)) {
    return Fail("empty request");
  }
  if (line != kMagic) {
    return Fail("bad request magic");
  }

  WorkerRequestParseResult result;
  result.ok = false;

  std::size_t expected_path_count = 0;
  std::size_t expected_obstacle_count = 0;
  bool saw_now = false;
  bool saw_planning_frame = false;
  bool saw_stamp = false;
  bool saw_robot_pose = false;
  bool saw_robot_twist = false;
  bool saw_target_pose = false;
  bool saw_path_count = false;
  bool saw_map = false;
  bool saw_map_data = false;
  bool saw_obstacle_count = false;

  while (ReadNextMeaningfulLine(input, &line)) {
    std::istringstream iss(line);
    std::string key;
    iss >> key;
    if (key == "now") {
      double seconds = 0.0;
      if (!(iss >> seconds)) {
        return Fail("invalid now line");
      }
      result.now = TimeFromDouble(seconds);
      saw_now = true;
    } else if (key == "planning_frame") {
      if (!(iss >> result.input.planning_frame)) {
        return Fail("invalid planning_frame line");
      }
      saw_planning_frame = true;
    } else if (key == "stamp") {
      double seconds = 0.0;
      if (!(iss >> seconds)) {
        return Fail("invalid stamp line");
      }
      result.input.stamp = TimeFromDouble(seconds);
      saw_stamp = true;
    } else if (key == "robot_pose") {
      result.input.robot_pose = ReadPose(iss);
      if (!iss) {
        return Fail("invalid robot_pose line");
      }
      saw_robot_pose = true;
    } else if (key == "robot_twist") {
      if (!(iss >> result.input.robot_twist.v >> result.input.robot_twist.w)) {
        return Fail("invalid robot_twist line");
      }
      saw_robot_twist = true;
    } else if (key == "target_pose") {
      result.input.target_pose = ReadPose(iss);
      if (!iss) {
        return Fail("invalid target_pose line");
      }
      saw_target_pose = true;
    } else if (key == "path_count") {
      if (!(iss >> expected_path_count)) {
        return Fail("invalid path_count line");
      }
      result.input.reference_path.clear();
      saw_path_count = true;
    } else if (key == "path") {
      Pose2d pose = ReadPose(iss);
      if (!iss) {
        return Fail("invalid path line");
      }
      result.input.reference_path.push_back(pose);
    } else if (key == "map") {
      std::string frame;
      double stamp_sec = 0.0;
      double origin_x = 0.0;
      double origin_y = 0.0;
      double origin_yaw = 0.0;
      unsigned int width = 0;
      unsigned int height = 0;
      double resolution = 0.0;
      if (!(iss >> frame >> stamp_sec >> width >> height >> resolution >> origin_x >> origin_y >> origin_yaw)) {
        return Fail("invalid map line");
      }
      result.input.occupancy_grid.header.frame_id = frame;
      result.input.occupancy_grid.header.stamp = TimeFromDouble(stamp_sec);
      result.input.occupancy_grid.info.width = width;
      result.input.occupancy_grid.info.height = height;
      result.input.occupancy_grid.info.resolution = resolution;
      result.input.occupancy_grid.info.origin.position.x = origin_x;
      result.input.occupancy_grid.info.origin.position.y = origin_y;
      result.input.occupancy_grid.info.origin.orientation.z = std::sin(0.5 * origin_yaw);
      result.input.occupancy_grid.info.origin.orientation.w = std::cos(0.5 * origin_yaw);
      saw_map = true;
    } else if (key == "map_data") {
      std::size_t count = 0;
      if (!(iss >> count)) {
        return Fail("invalid map_data count");
      }
      result.input.occupancy_grid.data.clear();
      result.input.occupancy_grid.data.reserve(count);
      for (std::size_t i = 0; i < count; ++i) {
        int value = 0;
        if (!(iss >> value)) {
          return Fail("map_data line has fewer values than declared");
        }
        result.input.occupancy_grid.data.push_back(static_cast<int8_t>(value));
      }
      saw_map_data = true;
    } else if (key == "obstacle_count") {
      if (!(iss >> expected_obstacle_count)) {
        return Fail("invalid obstacle_count line");
      }
      result.input.dynamic_obstacles.clear();
      saw_obstacle_count = true;
    } else if (key == "obstacle") {
      ObstacleTrack obstacle;
      double stamp_sec = 0.0;
      if (!(iss >> obstacle.id >> obstacle.frame_id >> stamp_sec >> obstacle.x >> obstacle.y >>
            obstacle.vx >> obstacle.vy >> obstacle.radius)) {
        return Fail("invalid obstacle line");
      }
      obstacle.stamp = TimeFromDouble(stamp_sec);
      result.input.dynamic_obstacles.push_back(obstacle);
    } else {
      return Fail("unknown request key: " + key);
    }
  }

  if (!saw_now || !saw_planning_frame || !saw_stamp || !saw_robot_pose ||
      !saw_robot_twist || !saw_target_pose || !saw_path_count || !saw_map ||
      !saw_map_data || !saw_obstacle_count) {
    return Fail("request missing required fields");
  }
  if (result.input.reference_path.size() != expected_path_count) {
    return Fail("path_count does not match path entries");
  }
  if (result.input.dynamic_obstacles.size() != expected_obstacle_count) {
    return Fail("obstacle_count does not match obstacle entries");
  }

  result.ok = true;
  result.reason = "ok";
  return result;
}

WorkerRequestParseResult LoadWorkerRequestFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return Fail("failed to open request file");
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return ParseWorkerRequestText(buffer.str());
}

}  // namespace lt_dwa_official_wrapper
