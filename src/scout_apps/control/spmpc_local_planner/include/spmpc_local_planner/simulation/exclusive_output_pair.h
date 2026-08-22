#pragma once

#include <string>

namespace spmpc_local_planner {
namespace simulation {

// Stages two immutable outputs in their destination directories, then
// publishes them as one logical transaction.  The final paths are never
// overwritten.  If either publish fails, every final path created by this
// transaction is removed and pre-existing paths are left untouched.
class ExclusiveOutputPair {
public:
    ExclusiveOutputPair() = default;
    ~ExclusiveOutputPair();

    ExclusiveOutputPair(const ExclusiveOutputPair&) = delete;
    ExclusiveOutputPair& operator=(const ExclusiveOutputPair&) = delete;

    bool stage(const std::string& first_path,
               const std::string& first_contents,
               const std::string& second_path,
               const std::string& second_contents,
               std::string& error);

    bool commit(std::string& error);

private:
    void rollback();

    std::string first_path_;
    std::string second_path_;
    std::string first_staged_path_;
    std::string second_staged_path_;
    std::string first_parent_;
    std::string second_parent_;
    bool staged_ = false;
    bool first_published_ = false;
    bool second_published_ = false;
    bool committed_ = false;
};

}  // namespace simulation
}  // namespace spmpc_local_planner
