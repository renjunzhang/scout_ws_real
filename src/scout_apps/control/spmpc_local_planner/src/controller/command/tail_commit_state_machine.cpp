#include "spmpc_local_planner/controller/command/tail_commit_state_machine.h"

namespace spmpc_local_planner {

const char* tailCommitStateName(TailCommitState state) {
    switch (state) {
    case TailCommitState::Disabled: return "DISABLED";
    case TailCommitState::Residual: return "RESIDUAL";
    case TailCommitState::Committed: return "COMMITTED";
    case TailCommitState::Released: return "RELEASED";
    case TailCommitState::Aborted: return "ABORTED";
    }
    return "ABORTED";
}

bool TailCommitStateMachine::configure(bool enabled) {
    enabled_ = enabled;
    artifact_size_ = 0;
    reset();
    return true;
}

bool TailCommitStateMachine::setArtifactSize(std::size_t artifact_size) {
    if (artifact_size == 0) return false;
    artifact_size_ = artifact_size;
    reset();
    return true;
}

void TailCommitStateMachine::reset() {
    state_ = enabled_ ? TailCommitState::Residual
                      : TailCommitState::Disabled;
    cursor_ = 0;
    anchor_index_ = TailPublicationEvidence::kNoIndex;
    reason_ = enabled_ ? "RESET_RESIDUAL" : "DISABLED";
}

TailCommitResult TailCommitStateMachine::snapshot(
    bool accepted,
    bool transitioned) const {
    TailCommitResult result;
    result.accepted = accepted;
    result.transitioned = transitioned;
    result.state = state_;
    result.cursor = cursor_;
    result.anchor_index = anchor_index_;
    result.reason = reason_;
    return result;
}

TailCommitResult TailCommitStateMachine::abort(const char* reason) {
    state_ = TailCommitState::Aborted;
    reason_ = reason;
    return snapshot(false, true);
}

TailCommitResult TailCommitStateMachine::onPublication(
    const TailPublicationEvidence& evidence) {
    if (state_ == TailCommitState::Disabled ||
        state_ == TailCommitState::Released ||
        state_ == TailCommitState::Aborted) {
        return snapshot(false, false);
    }
    if (evidence.safety_override) {
        return abort("ABORTED_SAFETY_OVERRIDE");
    }
    if (!evidence.delivered || !evidence.receipt_consistent ||
        !evidence.command_unmodified) {
        return abort("ABORTED_PUBLICATION_FAULT");
    }
    if (!evidence.contract_valid || !evidence.tail_command ||
        artifact_size_ == 0 ||
        evidence.artifact_index >= artifact_size_) {
        return abort("ABORTED_CONTRACT_FAULT");
    }

    const bool was_committed = state_ == TailCommitState::Committed;
    if (state_ == TailCommitState::Residual) {
        anchor_index_ = evidence.artifact_index;
    } else if (evidence.artifact_index != cursor_) {
        return abort("ABORTED_NONCONTIGUOUS_TAIL");
    }

    cursor_ = evidence.artifact_index + 1;
    if (cursor_ == artifact_size_) {
        state_ = TailCommitState::Released;
        reason_ = "TAIL_RELEASED";
    } else {
        state_ = TailCommitState::Committed;
        reason_ = anchor_index_ == evidence.artifact_index
            ? "TAIL_COMMITTED"
            : "TAIL_ADVANCED";
    }
    // Advancing an already committed cursor is accepted without another
    // lifecycle transition; entering committed or released is a transition.
    const bool transitioned = !was_committed ||
        state_ == TailCommitState::Released;
    return snapshot(true, transitioned);
}

}  // namespace spmpc_local_planner
