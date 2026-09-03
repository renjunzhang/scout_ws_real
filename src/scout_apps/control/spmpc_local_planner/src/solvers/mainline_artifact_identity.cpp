#include "spmpc_local_planner/solvers/mainline_artifact_identity.h"

#include <fcntl.h>
#include <openssl/evp.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr std::uint64_t kMaximumManifestSizeBytes = 64ULL * 1024ULL * 1024ULL;

class FileDescriptor {
 public:
  explicit FileDescriptor(int value) : value_(value) {}
  ~FileDescriptor() {
    if (value_ >= 0) {
      ::close(value_);
    }
  }
  FileDescriptor(const FileDescriptor&) = delete;
  FileDescriptor& operator=(const FileDescriptor&) = delete;
  int get() const { return value_; }

 private:
  int value_;
};

std::string systemError(const std::string& action, int error_number) {
  return action + ": " + std::strerror(error_number);
}

bool isLowerHexSha256(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  for (const char character : value) {
    if (!((character >= '0' && character <= '9') ||
          (character >= 'a' && character <= 'f'))) {
      return false;
    }
  }
  return true;
}

void requireSafeLeaf(const std::string& value, const char* label) {
  if (value.empty() || value == "." || value == ".." ||
      value.find('/') != std::string::npos) {
    throw ArtifactIdentityError(std::string(label) +
                                " must be one canonical relative leaf");
  }
}

void requireExpectation(const ArtifactIdentityExpectation& expected) {
  if (expected.model_id.empty()) {
    throw ArtifactIdentityError("compiled model_id must not be empty");
  }
  for (const auto& digest : {
           std::make_pair(&expected.model_contract_semantic_sha256,
                          "compiled model-contract semantic SHA-256"),
           std::make_pair(&expected.artifact_sha256,
                          "compiled artifact SHA-256"),
           std::make_pair(&expected.model_contract_raw_sha256,
                          "compiled model-contract raw SHA-256"),
           std::make_pair(&expected.solver_library_raw_sha256,
                          "compiled solver-library raw SHA-256")}) {
    if (!isLowerHexSha256(*digest.first)) {
      throw ArtifactIdentityError(std::string(digest.second) + " is malformed");
    }
  }
  requireSafeLeaf(expected.model_contract_filename, "model-contract filename");
  requireSafeLeaf(expected.solver_library_relative_path,
                  "solver-library relative path");
  if (expected.solver_library_size_bytes == 0U) {
    throw ArtifactIdentityError("compiled solver-library size must be positive");
  }
}

void requireCanonicalDirectory(const std::string& path) {
  if (path.size() < 2U || path.front() != '/' || path.back() == '/') {
    throw ArtifactIdentityError(
        "artifact directory must be a named canonical absolute path");
  }
  std::string current;
  std::size_t begin = 1U;
  while (begin < path.size()) {
    const std::size_t end = path.find('/', begin);
    const std::string component =
        path.substr(begin, end == std::string::npos ? end : end - begin);
    if (component.empty() || component == "." || component == "..") {
      throw ArtifactIdentityError("artifact directory path is not canonical");
    }
    current += "/" + component;
    struct stat metadata {};
    if (::lstat(current.c_str(), &metadata) != 0) {
      const int error_number = errno;
      throw ArtifactIdentityError(
          systemError("cannot inspect " + current, error_number));
    }
    if (S_ISLNK(metadata.st_mode)) {
      throw ArtifactIdentityError("artifact directory contains a symbolic link: " +
                                  current);
    }
    if (!S_ISDIR(metadata.st_mode)) {
      throw ArtifactIdentityError("artifact directory component is not a directory: " +
                                  current);
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
}

bool sameFileIdentity(const struct stat& before, const struct stat& after) {
  return before.st_dev == after.st_dev && before.st_ino == after.st_ino &&
         before.st_mode == after.st_mode && before.st_size == after.st_size &&
         before.st_mtim.tv_sec == after.st_mtim.tv_sec &&
         before.st_mtim.tv_nsec == after.st_mtim.tv_nsec &&
         before.st_ctim.tv_sec == after.st_ctim.tv_sec &&
         before.st_ctim.tv_nsec == after.st_ctim.tv_nsec;
}

std::string hashRegularFileAt(int directory_fd, const std::string& leaf,
                              std::uint64_t maximum_size_bytes,
                              std::uint64_t exact_size_bytes) {
  const int raw_fd = ::openat(directory_fd, leaf.c_str(),
                              O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (raw_fd < 0) {
    const int error_number = errno;
    throw ArtifactIdentityError(
        systemError("cannot open artifact file " + leaf, error_number));
  }
  FileDescriptor file(raw_fd);
  struct stat before {};
  if (::fstat(file.get(), &before) != 0) {
    const int error_number = errno;
    throw ArtifactIdentityError(
        systemError("cannot inspect artifact file " + leaf, error_number));
  }
  if (!S_ISREG(before.st_mode) || before.st_size < 0) {
    throw ArtifactIdentityError("artifact file is not a regular file: " + leaf);
  }
  const std::uint64_t size = static_cast<std::uint64_t>(before.st_size);
  if (size > maximum_size_bytes ||
      (exact_size_bytes != 0U && size != exact_size_bytes)) {
    throw ArtifactIdentityError("artifact file size differs from contract: " + leaf);
  }

  using DigestContext = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  DigestContext context(EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw ArtifactIdentityError("cannot initialize SHA-256");
  }
  unsigned char buffer[64U * 1024U];
  while (true) {
    const ssize_t count = ::read(file.get(), buffer, sizeof(buffer));
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0) {
      const int error_number = errno;
      throw ArtifactIdentityError(
          systemError("cannot read artifact file " + leaf, error_number));
    }
    if (count == 0) {
      break;
    }
    if (EVP_DigestUpdate(context.get(), buffer,
                         static_cast<std::size_t>(count)) != 1) {
      throw ArtifactIdentityError("cannot update SHA-256 for " + leaf);
    }
  }
  struct stat after {};
  if (::fstat(file.get(), &after) != 0 || !sameFileIdentity(before, after)) {
    throw ArtifactIdentityError("artifact file changed while hashing: " + leaf);
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context.get(), digest, &digest_size) != 1 ||
      digest_size != 32U) {
    throw ArtifactIdentityError("cannot finalize SHA-256 for " + leaf);
  }
  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0');
  for (unsigned int index = 0U; index < digest_size; ++index) {
    encoded << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return encoded.str();
}

}  // namespace

VerifiedArtifactIdentity verifyArtifactDirectory(
    const std::string& artifact_directory,
    const ArtifactIdentityExpectation& expected) {
  requireExpectation(expected);
  requireCanonicalDirectory(artifact_directory);
  const int raw_directory_fd =
      ::open(artifact_directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC |
                                             O_NOFOLLOW);
  if (raw_directory_fd < 0) {
    const int error_number = errno;
    throw ArtifactIdentityError(
        systemError("cannot open artifact directory " + artifact_directory,
                    error_number));
  }
  FileDescriptor directory(raw_directory_fd);
  struct stat metadata {};
  if (::fstat(directory.get(), &metadata) != 0 || !S_ISDIR(metadata.st_mode)) {
    throw ArtifactIdentityError("artifact root is not a directory");
  }

  const std::string contract_sha256 = hashRegularFileAt(
      directory.get(), expected.model_contract_filename,
      kMaximumManifestSizeBytes, 0U);
  if (contract_sha256 != expected.model_contract_raw_sha256) {
    throw ArtifactIdentityError(
        "model-contract bytes differ from the compiled identity");
  }
  const std::string library_sha256 = hashRegularFileAt(
      directory.get(), expected.solver_library_relative_path,
      expected.solver_library_size_bytes, expected.solver_library_size_bytes);
  if (library_sha256 != expected.solver_library_raw_sha256) {
    throw ArtifactIdentityError(
        "solver-library bytes differ from the compiled identity");
  }
  return {artifact_directory, expected.model_id,
          expected.model_contract_semantic_sha256, expected.artifact_sha256};
}

}  // namespace mainline
}  // namespace spmpc_local_planner
