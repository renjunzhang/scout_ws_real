#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct SpeedProfileSample {
    double progress_m = 0.0;
    double speed_mps = 0.0;
};

struct SpeedProfileLoadResult {
    bool success = false;
    std::string status = "NOT_LOADED";
    std::string detail;
    std::size_t accepted_rows = 0;
    std::size_t skipped_rows = 0;
};

class SpeedProfile {
public:
    SpeedProfileLoadResult loadCsv(const std::string& path);
    void clear();

    bool lookup(double progress_m, double& speed_mps) const;
    bool empty() const { return samples_.empty(); }
    std::size_t size() const { return samples_.size(); }
    const std::string& sourcePath() const { return source_path_; }
    const std::vector<SpeedProfileSample>& samples() const { return samples_; }

private:
    std::string source_path_;
    std::vector<SpeedProfileSample> samples_;
};

}  // namespace spmpc_local_planner
