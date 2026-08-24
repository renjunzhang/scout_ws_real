#pragma once

// Test-support loader that reconstructs the exact solver inputs from a
// development `summary.json` first_solver_failure_diagnostic snapshot.
//
// Lives in test/ only so production never depends on a JSON parser.  It is
// dependency-free: a minimal, auditable extractor for the fixed snapshot
// schema avoids adding a JSON library to the build.
//
// Consumed schema (see the closed-loop trial writer):
//   summary.json
//     .first_solver_failure_diagnostic
//       .initial_state_22d                -> manifest-width physical x0
//       .stage_parameters                 -> (N+1)*parameter_width flat doubles
//       .parameter_width                  -> manifest parameter width
//       .failed_raw_solution_states        -> 242 doubles (diagnostic only)
//       .failed_raw_solution_controls      ->  30 doubles (diagnostic only)
//       .solver_residuals.stationarity / .equality / ...
//       .solver_backend_contract.solver_id / .solver_config_hash

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace test_support {

// A cursor into a parsed JSON document.  Cursors share ownership of one
// immutable store, so all cursors remain valid while the store lives.
class SnapshotJson {
public:
    SnapshotJson();  // null cursor

    static bool parse(const std::string& text, SnapshotJson& out,
                     std::string& error);

    bool isNull() const;
    bool isObject() const;
    bool isArray() const;
    bool isNumber() const;
    bool isString() const;
    bool isBool() const;

    SnapshotJson find(const std::string& key) const;
    SnapshotJson at(std::size_t index) const;
    std::size_t size() const;

    bool numberArray(std::vector<double>& out) const;
    bool stringArray(std::vector<std::string>& out) const;

    bool number(double& out) const;
    bool integer(std::int64_t& out) const;
    bool boolean(bool& out) const;
    bool stringValue(std::string& out) const;

private:
    struct Store;
    std::shared_ptr<const Store> store_;
    std::size_t node_ = kNullNode;  // kNullNode = null sentinel
    static constexpr std::size_t kNullNode =
        static_cast<std::size_t>(-1);
    SnapshotJson(std::shared_ptr<const Store> store, std::size_t node);
};

}  // namespace test_support
}  // namespace spmpc_local_planner
