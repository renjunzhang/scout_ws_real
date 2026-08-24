#pragma once

#include <cstddef>
#include <limits>
#include <string>

namespace spmpc_local_planner {

enum class TailCommitState {
    Disabled = 0,
    Residual = 1,
    Committed = 2,
    Released = 3,
    Aborted = 4,
};

const char* tailCommitStateName(TailCommitState state);

// Evidence for exactly one finalized tail command.  The state machine stores
// indices only; commands are recomputed from the current aligned state and are
// never cached for open-loop replay.
struct TailPublicationEvidence {
    static constexpr std::size_t kNoIndex =
        std::numeric_limits<std::size_t>::max();

    std::size_t artifact_index = kNoIndex;
    bool tail_command = false;
    bool delivered = false;
    bool receipt_consistent = false;
    bool command_unmodified = false;
    bool safety_override = false;
    bool contract_valid = true;
};

struct TailCommitResult {
    bool accepted = false;
    bool transitioned = false;
    TailCommitState state = TailCommitState::Disabled;
    std::size_t cursor = 0;
    std::size_t anchor_index = TailPublicationEvidence::kNoIndex;
    std::string reason = "DISABLED";
};

// Persistent latch for a pre-validated feedback tail.  Higher-priority safety
// and publication faults abort the latch; it never returns to residual mode
// without an explicit lifecycle reset.
class TailCommitStateMachine {
public:
    bool configure(bool enabled);
    bool setArtifactSize(std::size_t artifact_size);
    void reset();

    TailCommitResult onPublication(
        const TailPublicationEvidence& evidence);

    TailCommitState state() const { return state_; }
    const char* stateName() const { return tailCommitStateName(state_); }
    std::size_t cursor() const { return cursor_; }
    std::size_t anchorIndex() const { return anchor_index_; }
    bool hasAnchor() const {
        return anchor_index_ != TailPublicationEvidence::kNoIndex;
    }
    const std::string& reason() const { return reason_; }
    bool enabled() const { return enabled_; }
    bool artifactConfigured() const { return artifact_size_ > 0; }

private:
    TailCommitResult snapshot(bool accepted, bool transitioned) const;
    TailCommitResult abort(const char* reason);

    bool enabled_ = false;
    std::size_t artifact_size_ = 0;
    TailCommitState state_ = TailCommitState::Disabled;
    std::size_t cursor_ = 0;
    std::size_t anchor_index_ = TailPublicationEvidence::kNoIndex;
    std::string reason_ = "DISABLED";
};

}  // namespace spmpc_local_planner
