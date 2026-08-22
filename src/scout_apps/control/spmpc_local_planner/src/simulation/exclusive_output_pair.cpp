#include "spmpc_local_planner/simulation/exclusive_output_pair.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits.h>
#include <set>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace spmpc_local_planner {
namespace simulation {
namespace {

bool resolveTarget(const std::string& input,
                   std::string& parent,
                   std::string& target,
                   std::string& error) {
    if (input.empty() || input.back() == '/') {
        error = "invalid output path: " + input;
        return false;
    }
    const std::size_t separator = input.find_last_of('/');
    const std::string raw_parent = separator == std::string::npos
        ? "." : (separator == 0 ? "/" : input.substr(0, separator));
    const std::string name = separator == std::string::npos
        ? input : input.substr(separator + 1);
    if (name.empty() || name == "." || name == "..") {
        error = "invalid output name: " + input;
        return false;
    }
    char resolved[PATH_MAX];
    if (::realpath(raw_parent.c_str(), resolved) == nullptr) {
        error = "cannot resolve output parent " + raw_parent + ": " +
            std::strerror(errno);
        return false;
    }
    parent = resolved;
    target = parent + (parent == "/" ? "" : "/") + name;
    struct stat status;
    if (::lstat(target.c_str(), &status) == 0) {
        error = "output already exists; refusing to overwrite: " + target;
        return false;
    }
    if (errno != ENOENT) {
        error = "cannot inspect output " + target + ": " +
            std::strerror(errno);
        return false;
    }
    return true;
}

bool writeAll(int descriptor,
              const std::string& path,
              const std::string& contents,
              std::string& error) {
    std::size_t offset = 0;
    while (offset < contents.size()) {
        const ssize_t count = ::write(
            descriptor, contents.data() + offset, contents.size() - offset);
        if (count < 0) {
            if (errno == EINTR) continue;
            error = "cannot write staged output " + path + ": " +
                std::strerror(errno);
            return false;
        }
        if (count == 0) {
            error = "zero-length staged output write: " + path;
            return false;
        }
        offset += static_cast<std::size_t>(count);
    }
    if (::fsync(descriptor) != 0) {
        error = "cannot fsync staged output " + path + ": " +
            std::strerror(errno);
        return false;
    }
    return true;
}

bool stageOne(const std::string& parent,
              const std::string& final_path,
              const std::string& contents,
              std::string& staged_path,
              std::string& error) {
    const std::size_t separator = final_path.find_last_of('/');
    const std::string name = final_path.substr(separator + 1);
    std::string pattern = parent + (parent == "/" ? "" : "/") +
        "." + name + ".tmp.XXXXXX";
    std::vector<char> writable(pattern.begin(), pattern.end());
    writable.push_back('\0');
    const int descriptor = ::mkstemp(writable.data());
    if (descriptor < 0) {
        error = "cannot create staged output for " + final_path + ": " +
            std::strerror(errno);
        return false;
    }
    staged_path = writable.data();
    bool success = ::fchmod(descriptor, 0644) == 0 &&
        writeAll(descriptor, staged_path, contents, error);
    if (!success && error.empty()) {
        error = "cannot set staged output permissions " + staged_path +
            ": " + std::strerror(errno);
    }
    if (::close(descriptor) != 0 && success) {
        success = false;
        error = "cannot close staged output " + staged_path + ": " +
            std::strerror(errno);
    }
    if (!success) {
        ::unlink(staged_path.c_str());
        staged_path.clear();
    }
    return success;
}

bool fsyncDirectory(const std::string& path, std::string& error) {
    const int descriptor = ::open(path.c_str(), O_RDONLY | O_DIRECTORY);
    if (descriptor < 0) {
        error = "cannot open output directory " + path + ": " +
            std::strerror(errno);
        return false;
    }
    const bool success = ::fsync(descriptor) == 0;
    const int saved_errno = errno;
    ::close(descriptor);
    if (!success) {
        error = "cannot fsync output directory " + path + ": " +
            std::strerror(saved_errno);
    }
    return success;
}

}  // namespace

ExclusiveOutputPair::~ExclusiveOutputPair() {
    if (!committed_) rollback();
}

bool ExclusiveOutputPair::stage(
    const std::string& first_path,
    const std::string& first_contents,
    const std::string& second_path,
    const std::string& second_contents,
    std::string& error) {
    error.clear();
    if (staged_ || committed_) {
        error = "output pair transaction already used";
        return false;
    }
    if (!resolveTarget(
            first_path, first_parent_, first_path_, error) ||
        !resolveTarget(
            second_path, second_parent_, second_path_, error)) {
        rollback();
        return false;
    }
    if (first_path_ == second_path_) {
        error = "output pair paths alias: " + first_path_;
        rollback();
        return false;
    }
    if (!stageOne(
            first_parent_, first_path_, first_contents,
            first_staged_path_, error) ||
        !stageOne(
            second_parent_, second_path_, second_contents,
            second_staged_path_, error)) {
        rollback();
        return false;
    }
    staged_ = true;
    return true;
}

bool ExclusiveOutputPair::commit(std::string& error) {
    error.clear();
    if (!staged_ || committed_) {
        error = "output pair transaction is not staged";
        return false;
    }
    if (::link(first_staged_path_.c_str(), first_path_.c_str()) != 0) {
        error = "cannot publish output " + first_path_ + ": " +
            std::strerror(errno);
        rollback();
        return false;
    }
    first_published_ = true;
    if (::link(second_staged_path_.c_str(), second_path_.c_str()) != 0) {
        error = "cannot publish output " + second_path_ + ": " +
            std::strerror(errno);
        rollback();
        return false;
    }
    second_published_ = true;
    ::unlink(first_staged_path_.c_str());
    first_staged_path_.clear();
    ::unlink(second_staged_path_.c_str());
    second_staged_path_.clear();

    std::set<std::string> parents = {first_parent_, second_parent_};
    for (const std::string& parent : parents) {
        if (!fsyncDirectory(parent, error)) {
            rollback();
            return false;
        }
    }
    committed_ = true;
    staged_ = false;
    return true;
}

void ExclusiveOutputPair::rollback() {
    if (second_published_ && !second_path_.empty()) {
        ::unlink(second_path_.c_str());
    }
    if (first_published_ && !first_path_.empty()) {
        ::unlink(first_path_.c_str());
    }
    if (!second_staged_path_.empty()) {
        ::unlink(second_staged_path_.c_str());
    }
    if (!first_staged_path_.empty()) {
        ::unlink(first_staged_path_.c_str());
    }
    first_published_ = false;
    second_published_ = false;
    staged_ = false;
    first_staged_path_.clear();
    second_staged_path_.clear();
}

}  // namespace simulation
}  // namespace spmpc_local_planner
