#include "spmpc_local_planner/phase_rejoin/phase_progress_governor.h"

#include <limits>

namespace spmpc_local_planner {

const char* phaseProgressActionName(PhaseProgressAction action) {
    switch (action) {
    case PhaseProgressAction::Advance: return "ADVANCE";
    case PhaseProgressAction::Hold: return "HOLD";
    case PhaseProgressAction::Reject: return "REJECT";
    case PhaseProgressAction::Complete: return "COMPLETE";
    }
    return "REJECT";
}

bool PhaseProgressGovernor::configure(int max_consecutive_holds) {
    if (max_consecutive_holds < 0) {
        configured_ = false;
        reset();
        return false;
    }

    max_consecutive_holds_ = max_consecutive_holds;
    configured_ = true;
    reset();
    return true;
}

bool PhaseProgressGovernor::initialize(std::size_t current_index,
                                       std::size_t artifact_size) {
    // Empty artifacts have no terminal sample, and accepting an index equal
    // to artifact_size would make a later +1 overflow the artifact contract.
    if (!configured_ || artifact_size == 0 || current_index >= artifact_size) {
        initialized_ = false;
        current_index_ = 0;
        artifact_size_ = 0;
        consecutive_holds_ = 0;
        ++revision_;
        return false;
    }

    current_index_ = current_index;
    artifact_size_ = artifact_size;
    consecutive_holds_ = 0;
    initialized_ = true;
    ++revision_;
    return true;
}

PhaseProgressDecision PhaseProgressGovernor::makeDecision(
    PhaseProgressAction action,
    bool valid,
    std::size_t next_index,
    const char* status,
    const char* reason,
    std::size_t clock_index) const {
    PhaseProgressDecision decision;
    decision.action = action;
    decision.name = phaseProgressActionName(action);
    decision.action_name = decision.name;
    decision.next_index = next_index;
    decision.status = status;
    decision.valid = valid;
    decision.reason = reason;
    decision.source_index = current_index_;
    decision.source_hold_count = consecutive_holds_;
    decision.clock_index = clock_index;
    decision.lag_steps = clock_index > current_index_
        ? clock_index - current_index_
        : 0;
    decision.revision = revision_;
    decision.origin = this;
    return decision;
}

PhaseProgressDecision PhaseProgressGovernor::evaluate(
    bool advance_joint_admitted,
    bool hold_joint_admitted) const {
    return evaluate(advance_joint_admitted, hold_joint_admitted,
                    current_index_);
}

PhaseProgressDecision PhaseProgressGovernor::evaluate(
    bool advance_joint_admitted,
    bool hold_joint_admitted,
    std::size_t clock_index) const {
    if (!configured_) {
        return makeDecision(PhaseProgressAction::Reject, false, 0,
                            "NOT_CONFIGURED", "NOT_CONFIGURED",
                            clock_index);
    }
    if (!initialized_) {
        return makeDecision(PhaseProgressAction::Reject, false, 0,
                            "NOT_INITIALIZED", "NOT_INITIALIZED",
                            clock_index);
    }

    // Check terminal ownership before considering a successor.  In
    // particular, current_index_ + 1 is never represented as next_index at
    // the artifact boundary.
    if (current_index_ + 1 >= artifact_size_) {
        return makeDecision(PhaseProgressAction::Complete, true,
                            current_index_, "COMPLETE", "ARTIFACT_END",
                            clock_index);
    }

    if (advance_joint_admitted) {
        // The terminal check above makes this increment safe and proves the
        // contiguous +1 invariant for every ADVANCE decision.
        return makeDecision(PhaseProgressAction::Advance, true,
                            current_index_ + 1, "ADVANCE", "JOINT_ADVANCE",
                            clock_index);
    }

    if (hold_joint_admitted) {
        const std::size_t lag_steps = clock_index > current_index_
            ? clock_index - current_index_
            : 0;
        // A HOLD at lag == budget would make the next clock sample exceed
        // the budget.  ADVANCE was handled first, so a temporarily overdue
        // cursor can always recover through a contiguous advance.
        if (lag_steps >= static_cast<std::size_t>(max_consecutive_holds_)) {
            return makeDecision(
                PhaseProgressAction::Reject, false, current_index_,
                "REJECT", "PHASE_LAG_LIMIT_EXCEEDED", clock_index);
        }
        if (consecutive_holds_ >= static_cast<std::size_t>(
                max_consecutive_holds_)) {
            return makeDecision(PhaseProgressAction::Reject, false,
                                current_index_, "REJECT",
                                "HOLD_LIMIT_EXCEEDED", clock_index);
        }
        return makeDecision(PhaseProgressAction::Hold, true, current_index_,
                            "HOLD", "JOINT_HOLD", clock_index);
    }
    return makeDecision(PhaseProgressAction::Reject, false, current_index_,
                        "REJECT", "NO_JOINT_ADMISSION", clock_index);
}

bool PhaseProgressGovernor::commit(const Decision& decision) {
    if (!configured_ || !initialized_ || !decision.valid ||
        decision.origin != this || decision.revision != revision_ ||
        decision.source_index != current_index_ ||
        decision.source_hold_count != consecutive_holds_) {
        return false;
    }

    // Re-validate the decision's public fields at the transaction boundary.
    // This keeps a copied or externally modified snapshot from bypassing the
    // contiguous-index and hold-budget contracts.
    const PhaseProgressDecision expected = evaluate(
        decision.action == PhaseProgressAction::Advance,
        decision.action == PhaseProgressAction::Hold,
        decision.clock_index);
    if (!expected.valid || expected.action != decision.action ||
        expected.next_index != decision.next_index ||
        expected.name != decision.name || expected.status != decision.status ||
        expected.reason != decision.reason ||
        expected.clock_index != decision.clock_index ||
        expected.lag_steps != decision.lag_steps) {
        return false;
    }

    switch (decision.action) {
    case PhaseProgressAction::Advance:
        // evaluate() already checked the boundary, but retain a defensive
        // check here because commit is the only state mutation point.
        if (current_index_ + 1 >= artifact_size_ ||
            decision.next_index != current_index_ + 1) {
            return false;
        }
        current_index_ = decision.next_index;
        consecutive_holds_ = 0;
        break;
    case PhaseProgressAction::Hold:
        if (decision.next_index != current_index_ ||
            consecutive_holds_ >= static_cast<std::size_t>(
                max_consecutive_holds_)) {
            return false;
        }
        ++consecutive_holds_;
        break;
    case PhaseProgressAction::Complete:
        if (current_index_ + 1 < artifact_size_ ||
            decision.next_index != current_index_) {
            return false;
        }
        // Completion owns no successor, so index/hold state remains unchanged.
        break;
    case PhaseProgressAction::Reject:
        return false;
    }

    // Even COMPLETE advances the transaction revision, preventing replay of
    // an old decision after a successful commit.  Cursor and hold counters
    // are changed only in the action branches above.
    ++revision_;
    return true;
}

void PhaseProgressGovernor::reset() {
    initialized_ = false;
    current_index_ = 0;
    artifact_size_ = 0;
    consecutive_holds_ = 0;
    ++revision_;
}

}  // namespace spmpc_local_planner
