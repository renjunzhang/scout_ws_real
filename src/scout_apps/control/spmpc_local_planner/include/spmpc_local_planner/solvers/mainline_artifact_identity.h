#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace spmpc_local_planner {
namespace mainline {

class ArtifactIdentityError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct ArtifactIdentityExpectation {
  std::string model_id;
  std::string model_contract_semantic_sha256;
  std::string artifact_sha256;
  std::string model_contract_filename;
  std::string model_contract_raw_sha256;
  std::string solver_library_relative_path;
  std::uint64_t solver_library_size_bytes{0};
  std::string solver_library_raw_sha256;
};

struct VerifiedArtifactIdentity {
  std::string artifact_directory;
  std::string model_id;
  std::string model_contract_semantic_sha256;
  std::string artifact_sha256;
};

// Verifies the two runtime files bound by the generated C++ header. The root
// and both leaves must be real directories/files rather than symbolic links.
VerifiedArtifactIdentity verifyArtifactDirectory(
    const std::string& artifact_directory,
    const ArtifactIdentityExpectation& expected);

}  // namespace mainline
}  // namespace spmpc_local_planner
