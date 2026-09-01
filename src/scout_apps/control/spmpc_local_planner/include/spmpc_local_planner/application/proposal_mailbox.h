#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "spmpc_local_planner/domain/mainline_types.h"

namespace spmpc_local_planner {
namespace mainline {

struct Sha256Digest {
  std::array<std::uint8_t, 32> bytes{};
};

inline bool operator==(const Sha256Digest& lhs, const Sha256Digest& rhs) {
  return lhs.bytes == rhs.bytes;
}

inline bool operator!=(const Sha256Digest& lhs, const Sha256Digest& rhs) {
  return !(lhs == rhs);
}

struct ProposalIdentity {
  ProposalIdentity() : release_steady(0), release_model(0) {}

  std::uint64_t cycle_id{0};
  SteadyTimeNs release_steady;
  ModelTimeNs release_model;
  std::uint64_t reset_epoch{0};
  std::uint64_t history_generation{0};
  Sha256Digest config_hash;
  Sha256Digest model_hash;
  Sha256Digest artifact_hash;
  Sha256Digest path_hash;
};

inline SteadyTimeNs checkedProposalCutoff(SteadyTimeNs release_steady,
                                          std::int64_t handoff_margin_ns) {
  if (handoff_margin_ns < 0) {
    throw std::invalid_argument("proposal handoff margin must be nonnegative");
  }
  const std::int64_t minimum = std::numeric_limits<std::int64_t>::min();
  if (release_steady.value < minimum + handoff_margin_ns) {
    throw std::overflow_error("proposal cutoff exceeds steady clock range");
  }
  return SteadyTimeNs(release_steady.value - handoff_margin_ns);
}

inline bool operator==(const ProposalIdentity& lhs,
                       const ProposalIdentity& rhs) {
  return lhs.cycle_id == rhs.cycle_id &&
         lhs.release_steady.value == rhs.release_steady.value &&
         lhs.release_model.value == rhs.release_model.value &&
         lhs.reset_epoch == rhs.reset_epoch &&
         lhs.history_generation == rhs.history_generation &&
         lhs.config_hash == rhs.config_hash && lhs.model_hash == rhs.model_hash &&
         lhs.artifact_hash == rhs.artifact_hash &&
         lhs.path_hash == rhs.path_hash;
}

inline bool operator!=(const ProposalIdentity& lhs,
                       const ProposalIdentity& rhs) {
  return !(lhs == rhs);
}

template <typename Payload>
struct PendingRelease {
  PendingRelease() : ready_steady(0) {}

  ProposalIdentity identity;
  SteadyTimeNs ready_steady;
  Payload payload{};
};

enum class MailboxArmResult : std::uint8_t {
  kArmed = 0,
  kWrongEpoch,
  kWrongCycle,
  kSlotBusy,
  kInvalidCutoff,
  kCycleOverflow,
};

enum class ProposalPublishResult : std::uint8_t {
  kAccepted = 0,
  kNotArmed,
  kWrongCycle,
  kWrongIdentity,
  kLate,
  kDuplicate,
  kSlotBusy,
  kClosed,
};

enum class ProposalTakeResult : std::uint8_t {
  kReady = 0,
  kMissing,
  kWrongIdentity,
  kLate,
  kAlreadyClosed,
  kNotArmed,
};

// Fixed-capacity, single-producer/single-consumer proposal mailbox. Payload
// visibility is published only by a tagged atomic state; the cycle tag prevents
// an old writer from winning an ABA race after a physical slot is reused.
// closeAndTakeExact() never allocates, locks, or waits for solver progress.
template <typename Payload, std::size_t Capacity = 2>
class ProposalMailbox {
 public:
  static_assert(Capacity >= 2, "proposal mailbox needs at least two slots");
  static_assert(std::is_trivially_copyable<Payload>::value,
                "proposal payload must be trivially copyable");
  static_assert(std::is_default_constructible<Payload>::value,
                "proposal payload must be default constructible");
  static_assert(std::is_trivially_copyable<PendingRelease<Payload>>::value,
                "complete pending release must be trivially copyable");
  static_assert(
      std::is_trivially_copy_assignable<PendingRelease<Payload>>::value,
      "release-side pending copy must use trivial assignment");

  explicit ProposalMailbox(std::uint64_t reset_epoch,
                           std::uint64_t first_cycle_id = 0)
      : reset_epoch_(reset_epoch), next_arm_cycle_id_(first_cycle_id) {
    for (const Slot& slot : slots_) {
      if (!slot.tagged_state.is_lock_free()) {
        throw std::runtime_error("proposal mailbox atomics are not lock-free");
      }
    }
  }

  ProposalMailbox(const ProposalMailbox&) = delete;
  ProposalMailbox& operator=(const ProposalMailbox&) = delete;

  MailboxArmResult arm(const ProposalIdentity& expected,
                       SteadyTimeNs proposal_cutoff) {
    if (expected.reset_epoch != reset_epoch_) {
      return MailboxArmResult::kWrongEpoch;
    }
    if (expected.cycle_id != next_arm_cycle_id_) {
      return MailboxArmResult::kWrongCycle;
    }
    if (expected.cycle_id > kMaximumTaggedCycle) {
      return MailboxArmResult::kCycleOverflow;
    }
    if (proposal_cutoff.value > expected.release_steady.value) {
      return MailboxArmResult::kInvalidCutoff;
    }

    Slot& slot = slots_[indexFor(expected.cycle_id)];
    const std::uint64_t observed =
        slot.tagged_state.load(std::memory_order_acquire);
    if (!isReusable(stateOf(observed))) {
      return MailboxArmResult::kSlotBusy;
    }

    slot.expected = expected;
    slot.proposal_cutoff = proposal_cutoff;
    slot.rejection = Rejection::kNone;
    slot.tagged_state.store(tag(expected.cycle_id, SlotState::kOpen),
                            std::memory_order_release);
    ++next_arm_cycle_id_;
    return MailboxArmResult::kArmed;
  }

  ProposalPublishResult tryPublish(const PendingRelease<Payload>& proposal) {
    if (proposal.identity.cycle_id > kMaximumTaggedCycle) {
      return ProposalPublishResult::kWrongCycle;
    }
    Slot& slot = slots_[indexFor(proposal.identity.cycle_id)];
    std::uint64_t observed =
        slot.tagged_state.load(std::memory_order_acquire);
    for (;;) {
      const SlotState state = stateOf(observed);
      if (state == SlotState::kEmpty) {
        return ProposalPublishResult::kNotArmed;
      }
      if (cycleOf(observed) != proposal.identity.cycle_id) {
        return ProposalPublishResult::kWrongCycle;
      }
      if (state != SlotState::kOpen) {
        return publishFailure(state);
      }
      const std::uint64_t writing =
          tag(proposal.identity.cycle_id, SlotState::kWriting);
      if (slot.tagged_state.compare_exchange_strong(
              observed, writing, std::memory_order_acq_rel,
              std::memory_order_acquire)) {
        break;
      }
    }

    if (proposal.identity != slot.expected) {
      rejectWrite(slot, proposal.identity.cycle_id, Rejection::kWrongIdentity);
      return ProposalPublishResult::kWrongIdentity;
    }
    if (proposal.ready_steady.value > slot.proposal_cutoff.value) {
      rejectWrite(slot, proposal.identity.cycle_id, Rejection::kLate);
      return ProposalPublishResult::kLate;
    }

    slot.proposal = proposal;
    std::uint64_t writing =
        tag(proposal.identity.cycle_id, SlotState::kWriting);
    if (slot.tagged_state.compare_exchange_strong(
            writing, tag(proposal.identity.cycle_id, SlotState::kReady),
            std::memory_order_release, std::memory_order_acquire)) {
      return ProposalPublishResult::kAccepted;
    }
    // The only legal competing transition from Writing is the consumer's
    // close request. Do not publish Ready after that linearization point.
    slot.tagged_state.store(
        tag(proposal.identity.cycle_id, SlotState::kAbandoned),
        std::memory_order_release);
    return ProposalPublishResult::kClosed;
  }

  ProposalTakeResult closeAndTakeExact(
      const ProposalIdentity& expected, PendingRelease<Payload>& output) {
    if (expected.cycle_id > kMaximumTaggedCycle) {
      return ProposalTakeResult::kNotArmed;
    }
    Slot& slot = slots_[indexFor(expected.cycle_id)];
    std::uint64_t observed =
        slot.tagged_state.load(std::memory_order_acquire);
    for (;;) {
      const SlotState state = stateOf(observed);
      if (state == SlotState::kEmpty) {
        return ProposalTakeResult::kNotArmed;
      }
      if (cycleOf(observed) != expected.cycle_id) {
        return ProposalTakeResult::kNotArmed;
      }
      if (isTerminal(state)) {
        return ProposalTakeResult::kAlreadyClosed;
      }
      if (slot.expected != expected) {
        switch (state) {
          case SlotState::kOpen:
            if (transition(slot, observed, expected.cycle_id,
                           SlotState::kClosed)) {
              return ProposalTakeResult::kWrongIdentity;
            }
            break;
          case SlotState::kWriting:
            if (transition(slot, observed, expected.cycle_id,
                           SlotState::kCloseRequested)) {
              return ProposalTakeResult::kWrongIdentity;
            }
            break;
          case SlotState::kReady:
          case SlotState::kRejected:
            if (transition(slot, observed, expected.cycle_id,
                           SlotState::kClaimed)) {
              return ProposalTakeResult::kWrongIdentity;
            }
            break;
          case SlotState::kEmpty:
          case SlotState::kCloseRequested:
          case SlotState::kClosed:
          case SlotState::kClaiming:
          case SlotState::kClaimed:
          case SlotState::kAbandoned:
            return ProposalTakeResult::kAlreadyClosed;
        }
        continue;
      }

      switch (state) {
        case SlotState::kOpen:
          if (transition(slot, observed, expected.cycle_id,
                         SlotState::kClosed)) {
            return ProposalTakeResult::kMissing;
          }
          break;
        case SlotState::kWriting:
          if (transition(slot, observed, expected.cycle_id,
                         SlotState::kCloseRequested)) {
            return ProposalTakeResult::kMissing;
          }
          break;
        case SlotState::kReady:
          if (transition(slot, observed, expected.cycle_id,
                         SlotState::kClaiming)) {
            output = slot.proposal;
            slot.tagged_state.store(
                tag(expected.cycle_id, SlotState::kClaimed),
                std::memory_order_release);
            return ProposalTakeResult::kReady;
          }
          break;
        case SlotState::kRejected:
          if (transition(slot, observed, expected.cycle_id,
                         SlotState::kClaiming)) {
            const ProposalTakeResult rejection_result =
                slot.rejection == Rejection::kLate
                    ? ProposalTakeResult::kLate
                    : ProposalTakeResult::kWrongIdentity;
            slot.tagged_state.store(
                tag(expected.cycle_id, SlotState::kClaimed),
                std::memory_order_release);
            return rejection_result;
          }
          break;
        case SlotState::kEmpty:
        case SlotState::kCloseRequested:
        case SlotState::kClosed:
        case SlotState::kClaiming:
        case SlotState::kClaimed:
        case SlotState::kAbandoned:
          return ProposalTakeResult::kAlreadyClosed;
      }
    }
  }

  std::uint64_t nextArmCycleId() const noexcept { return next_arm_cycle_id_; }
  std::uint64_t resetEpoch() const noexcept { return reset_epoch_; }

 private:
  enum class SlotState : std::uint8_t {
    kEmpty = 0,
    kOpen,
    kWriting,
    kReady,
    kRejected,
    kCloseRequested,
    kClosed,
    kClaiming,
    kClaimed,
    kAbandoned,
  };

  enum class Rejection : std::uint8_t { kNone = 0, kWrongIdentity, kLate };

  static constexpr std::uint64_t kStateBits = 4;
  static constexpr std::uint64_t kStateMask = (1ULL << kStateBits) - 1ULL;
  static constexpr std::uint64_t kMaximumTaggedCycle =
      std::numeric_limits<std::uint64_t>::max() >> kStateBits;

  struct Slot {
    Slot() : tagged_state(tag(0, SlotState::kEmpty)), proposal_cutoff(0) {}

    std::atomic<std::uint64_t> tagged_state;
    ProposalIdentity expected;
    SteadyTimeNs proposal_cutoff;
    PendingRelease<Payload> proposal;
    Rejection rejection{Rejection::kNone};
  };

  static constexpr std::size_t indexFor(std::uint64_t cycle_id) {
    return static_cast<std::size_t>(cycle_id % Capacity);
  }

  static constexpr std::uint64_t tag(std::uint64_t cycle_id, SlotState state) {
    return (cycle_id << kStateBits) | static_cast<std::uint64_t>(state);
  }

  static constexpr std::uint64_t cycleOf(std::uint64_t tagged_state) {
    return tagged_state >> kStateBits;
  }

  static constexpr SlotState stateOf(std::uint64_t tagged_state) {
    return static_cast<SlotState>(tagged_state & kStateMask);
  }

  static bool isReusable(SlotState state) {
    return state == SlotState::kEmpty || state == SlotState::kClosed ||
           state == SlotState::kClaimed || state == SlotState::kAbandoned;
  }

  static bool isTerminal(SlotState state) {
    return state == SlotState::kCloseRequested || state == SlotState::kClosed ||
           state == SlotState::kClaiming || state == SlotState::kClaimed ||
           state == SlotState::kAbandoned;
  }

  static bool transition(Slot& slot, std::uint64_t& observed,
                         std::uint64_t cycle_id, SlotState desired) {
    return slot.tagged_state.compare_exchange_strong(
        observed, tag(cycle_id, desired), std::memory_order_acq_rel,
        std::memory_order_acquire);
  }

  static ProposalPublishResult publishFailure(SlotState state) {
    switch (state) {
      case SlotState::kReady:
      case SlotState::kRejected:
        return ProposalPublishResult::kDuplicate;
      case SlotState::kWriting:
        return ProposalPublishResult::kSlotBusy;
      case SlotState::kCloseRequested:
      case SlotState::kClosed:
      case SlotState::kClaiming:
      case SlotState::kClaimed:
      case SlotState::kAbandoned:
        return ProposalPublishResult::kClosed;
      case SlotState::kEmpty:
        return ProposalPublishResult::kNotArmed;
      case SlotState::kOpen:
        break;
    }
    return ProposalPublishResult::kSlotBusy;
  }

  static void rejectWrite(Slot& slot, std::uint64_t cycle_id,
                          Rejection rejection) {
    slot.rejection = rejection;
    std::uint64_t writing = tag(cycle_id, SlotState::kWriting);
    if (!slot.tagged_state.compare_exchange_strong(
            writing, tag(cycle_id, SlotState::kRejected),
            std::memory_order_release, std::memory_order_acquire)) {
      slot.tagged_state.store(tag(cycle_id, SlotState::kAbandoned),
                              std::memory_order_release);
    }
  }

  std::array<Slot, Capacity> slots_;
  const std::uint64_t reset_epoch_{0};
  std::uint64_t next_arm_cycle_id_{0};
};

}  // namespace mainline
}  // namespace spmpc_local_planner
