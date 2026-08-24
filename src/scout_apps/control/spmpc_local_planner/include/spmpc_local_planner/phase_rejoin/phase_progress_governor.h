#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace spmpc_local_planner {

class PhaseProgressGovernor;

// The action is deliberately small: the governor only owns contiguous phase
// progress.  A rejected decision never authorizes a phase jump.
enum class PhaseProgressAction {
    Advance = 0,
    Hold = 1,
    Reject = 2,
    Complete = 3,

    // Upper-case aliases keep the wire/diagnostic spelling convenient for
    // callers while the mixed-case names follow the project's C++ style.
    ADVANCE = Advance,
    HOLD = Hold,
    REJECT = Reject,
    COMPLETE = Complete,
    kAdvance = Advance,
    kHold = Hold,
    kReject = Reject,
    kComplete = Complete,
};

const char* phaseProgressActionName(PhaseProgressAction action);

// A decision is a snapshot.  It contains the state revision on which it was
// made, so a decision evaluated before another committed decision cannot be
// replayed accidentally.  `origin` also prevents a decision from a different
// governor instance from being accepted when their revisions happen to match.
struct PhaseProgressDecision {
    PhaseProgressAction action = PhaseProgressAction::Reject;
    std::string name = "REJECT";
    // Alias retained in the result for diagnostics that use an explicit
    // action_name field.  It is always equal to name for governor decisions.
    std::string action_name = "REJECT";
    std::size_t next_index = 0;
    std::string status = "NOT_INITIALIZED";
    bool valid = false;
    std::string reason = "NOT_INITIALIZED";

    // Snapshot metadata is implementation-facing, but public fields keep the
    // decision an inexpensive value type and make tests able to inspect it.
    std::size_t source_index = 0;
    std::size_t source_hold_count = 0;
    // Absolute phase-clock snapshot used to bound lag while retaining the
    // contiguous cursor contract.  This is deliberately carried by the
    // decision so commit can re-validate the exact clock observation.
    std::size_t clock_index = 0;
    std::size_t lag_steps = 0;
    std::uint64_t revision = 0;
    const PhaseProgressGovernor* origin = nullptr;
};

// Minimal transactional progress policy for a phase artifact.
//
// Evaluation is pure.  It proposes at most one contiguous index step; only a
// matching commit changes the cursor or consecutive-hold counter.  Once the
// consecutive-hold or absolute-clock lag budget is exhausted this class
// rejects.  The caller above this module must then switch to Tail-Commit; the
// governor must never force a phase jump merely because a budget is exceeded.
class PhaseProgressGovernor {
public:
    using Action = PhaseProgressAction;
    using Decision = PhaseProgressDecision;

    // Negative budgets are invalid.  A valid configuration may allow zero
    // consecutive holds, in which case an otherwise non-advancing cycle is
    // rejected immediately.
    bool configure(int max_consecutive_holds);

    // The current index is inclusive and must identify a sample in a nonempty
    // artifact.  The last sample is terminal: evaluate() returns COMPLETE and
    // never proposes artifact_size as an out-of-range next index.
    bool initialize(std::size_t current_index, std::size_t artifact_size);

    // Priority is ADVANCE, then HOLD (while within both budgets), then
    // REJECT.  The no-argument form is retained for callers that do not have
    // an external clock; Coordinator callers must pass the absolute clock.
    // COMPLETE is returned for the artifact's final sample and takes priority
    // over the admission booleans because no successor exists.
    Decision evaluate(bool advance_joint_admitted,
                      bool hold_joint_admitted) const;
    Decision evaluate(bool advance_joint_admitted,
                      bool hold_joint_admitted,
                      std::size_t clock_index) const;

    // Commits exactly one decision snapshot.  Returns false for a rejected,
    // malformed, foreign, or stale decision and leaves state unchanged.
    bool commit(const Decision& decision);

    // Clears the initialized artifact/cursor and invalidates all decisions,
    // while retaining a valid hold-budget configuration for re-initialization.
    void reset();

    static const char* actionName(PhaseProgressAction action) {
        return phaseProgressActionName(action);
    }

    bool configured() const { return configured_; }
    bool initialized() const { return initialized_; }
    int maxConsecutiveHolds() const { return max_consecutive_holds_; }
    int max_consecutive_holds() const { return max_consecutive_holds_; }
    std::size_t currentIndex() const { return current_index_; }
    std::size_t current_index() const { return current_index_; }
    std::size_t artifactSize() const { return artifact_size_; }
    std::size_t artifact_size() const { return artifact_size_; }
    std::size_t consecutiveHolds() const { return consecutive_holds_; }
    std::size_t consecutive_holds() const { return consecutive_holds_; }

private:
    Decision makeDecision(PhaseProgressAction action,
                          bool valid,
                          std::size_t next_index,
                          const char* status,
                          const char* reason,
                          std::size_t clock_index) const;

    bool configured_ = false;
    bool initialized_ = false;
    int max_consecutive_holds_ = 0;
    std::size_t current_index_ = 0;
    std::size_t artifact_size_ = 0;
    std::size_t consecutive_holds_ = 0;
    std::uint64_t revision_ = 0;
};

}  // namespace spmpc_local_planner
