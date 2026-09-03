#include "spmpc_local_planner/solvers/mainline_artifact_identity.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr char kManifest[] = "{\"fixture\":true}\n";
constexpr char kLibrary[] = "solver-bytes\n";
constexpr char kManifestSha256[] =
    "218589323cbe80b7ed077e3ee36f1663e7cb5f8f4e4ad02c938ad8a5c2c5a6b9";
constexpr char kLibrarySha256[] =
    "b21a879b11c0f7fac154fbb7c158c0f97753de4e53bf7e5bb71d9059e807572c";

void writeFile(const std::string& path, const std::string& contents) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream || !(stream << contents)) {
    throw std::runtime_error("cannot write test artifact file");
  }
  stream.close();
  if (!stream) {
    throw std::runtime_error("cannot close test artifact file");
  }
}

class TemporaryArtifact {
 public:
  TemporaryArtifact() {
    char pattern[] = "/tmp/spmpc-artifact-identity-XXXXXX";
    char* created = ::mkdtemp(pattern);
    if (created == nullptr) {
      throw std::runtime_error("cannot create test artifact directory");
    }
    root_ = created;
    writeFile(manifestPath(), kManifest);
    writeFile(libraryPath(), kLibrary);
  }

  ~TemporaryArtifact() {
    ::unlink(manifestPath().c_str());
    ::unlink(libraryPath().c_str());
    ::unlink((root_ + "/real_contract.json").c_str());
    ::rmdir(root_.c_str());
  }

  const std::string& root() const { return root_; }
  std::string manifestPath() const { return root_ + "/model_contract.json"; }
  std::string libraryPath() const { return root_ + "/libsolver.so"; }

 private:
  std::string root_;
};

ArtifactIdentityExpectation expectation() {
  ArtifactIdentityExpectation expected;
  expected.model_id = "fixture_model";
  expected.model_contract_semantic_sha256 = std::string(64U, '1');
  expected.artifact_sha256 = std::string(64U, '2');
  expected.model_contract_filename = "model_contract.json";
  expected.model_contract_raw_sha256 = kManifestSha256;
  expected.solver_library_relative_path = "libsolver.so";
  expected.solver_library_size_bytes = sizeof(kLibrary) - 1U;
  expected.solver_library_raw_sha256 = kLibrarySha256;
  return expected;
}

TEST(MainlineArtifactIdentity, AcceptsExactRegularFiles) {
  TemporaryArtifact artifact;
  const VerifiedArtifactIdentity verified =
      verifyArtifactDirectory(artifact.root(), expectation());
  EXPECT_EQ(artifact.root(), verified.artifact_directory);
  EXPECT_EQ("fixture_model", verified.model_id);
  EXPECT_EQ(std::string(64U, '2'), verified.artifact_sha256);
}

TEST(MainlineArtifactIdentity, RejectsModifiedManifest) {
  TemporaryArtifact artifact;
  writeFile(artifact.manifestPath(), "tampered\n");
  EXPECT_THROW(verifyArtifactDirectory(artifact.root(), expectation()),
               ArtifactIdentityError);
}

TEST(MainlineArtifactIdentity, RejectsWrongLibraryBytesAndSize) {
  TemporaryArtifact artifact;
  writeFile(artifact.libraryPath(), "solver-byteS\n");
  EXPECT_THROW(verifyArtifactDirectory(artifact.root(), expectation()),
               ArtifactIdentityError);

  writeFile(artifact.libraryPath(), kLibrary);
  ArtifactIdentityExpectation wrong_size = expectation();
  ++wrong_size.solver_library_size_bytes;
  EXPECT_THROW(verifyArtifactDirectory(artifact.root(), wrong_size),
               ArtifactIdentityError);
}

TEST(MainlineArtifactIdentity, RejectsMissingFiles) {
  TemporaryArtifact artifact;
  ASSERT_EQ(0, ::unlink(artifact.libraryPath().c_str()));
  EXPECT_THROW(verifyArtifactDirectory(artifact.root(), expectation()),
               ArtifactIdentityError);
}

TEST(MainlineArtifactIdentity, RejectsSymbolicLinkLeaf) {
  TemporaryArtifact artifact;
  const std::string real_manifest = artifact.root() + "/real_contract.json";
  writeFile(real_manifest, kManifest);
  ASSERT_EQ(0, ::unlink(artifact.manifestPath().c_str()));
  ASSERT_EQ(0, ::symlink("real_contract.json", artifact.manifestPath().c_str()));
  EXPECT_THROW(verifyArtifactDirectory(artifact.root(), expectation()),
               ArtifactIdentityError);
}

TEST(MainlineArtifactIdentity, RejectsNonCanonicalRootAndLeaf) {
  TemporaryArtifact artifact;
  EXPECT_THROW(verifyArtifactDirectory(artifact.root() + "/../" +
                                           artifact.root().substr(5U),
                                       expectation()),
               ArtifactIdentityError);
  ArtifactIdentityExpectation traversal = expectation();
  traversal.solver_library_relative_path = "../libsolver.so";
  EXPECT_THROW(verifyArtifactDirectory(artifact.root(), traversal),
               ArtifactIdentityError);
}

TEST(MainlineArtifactIdentity, RejectsMalformedCompiledIdentity) {
  TemporaryArtifact artifact;
  ArtifactIdentityExpectation malformed = expectation();
  malformed.artifact_sha256 = std::string(64U, 'A');
  EXPECT_THROW(verifyArtifactDirectory(artifact.root(), malformed),
               ArtifactIdentityError);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
