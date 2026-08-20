#pragma once

#include "spmpc_local_planner/phase_rejoin/types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct NominalArtifactLoadResult {
    bool success = false;
    std::string status = "NOT_RUN";
    std::string detail;
};

class NominalSequenceArtifact {
public:
    NominalArtifactLoadResult loadCsv(const std::string& path);
    void clear();

    bool valid() const { return valid_; }
    bool empty() const { return samples_.empty(); }
    std::size_t size() const { return samples_.size(); }
    const std::string& path() const { return path_; }
    const NominalArtifactMetadata& metadata() const { return metadata_; }
    const std::vector<PhaseNominalSample>& samples() const { return samples_; }

    bool hasIndex(std::size_t index) const { return index < samples_.size(); }
    const PhaseNominalSample* sample(std::size_t index) const;

private:
    bool valid_ = false;
    std::string path_;
    NominalArtifactMetadata metadata_;
    std::vector<PhaseNominalSample> samples_;
};

}  // namespace spmpc_local_planner
