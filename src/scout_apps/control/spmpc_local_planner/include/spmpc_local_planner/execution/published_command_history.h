#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "spmpc_local_planner/domain/release_contract.h"
#include "spmpc_local_planner/execution/command_event.h"

namespace spmpc_local_planner {
namespace mainline {

enum class HistoryCommitResult : std::uint8_t {
  kCommitted = 0,
  kWouldPublish,
  kPublishFailed,
  kWrongEpoch,
  kWrongCycle,
  kWrongBoundary,
  kReceiptMismatch,
  kGenerationMismatch,
  kInvalidReason,
  kInvalidCommand,
  kInvalidAuthority,
  kPublishedTooEarly,
  kActualClockRegression,
  // Retained only to preserve numeric audit identity.  A late command that
  // was actually published is now committed with event.publish_late=true.
  kLateNominal,
  kTimeOverflow,
  kCycleOverflow,
  kGenerationOverflow,
  kSnapshotBusy,
  kFaultLatched,
};

enum class HistorySnapshotResult : std::uint8_t {
  kReady = 0,
  kEmpty,
  kConcurrentWrite,
  kReaderBusy,
  kSnapshotClockRegression,
};

enum class HistorySampleResult : std::uint8_t {
  kExact = 0,
  kHeldBetweenEvents,
  kFutureHold,
  kBeforeHistory,
  kEmpty,
};

enum class HistoryCoverageStatus : std::uint8_t {
  kComplete = 0,
  kEmpty,
  kInvalidRange,
  kMissingPredecessor,
  kGapTooLarge,
  kTimeOverflow,
};

struct HistoryCoverage {
  HistoryCoverageStatus status{HistoryCoverageStatus::kEmpty};
  std::uint64_t history_generation{0};
  std::uint64_t predecessor_release_generation{0};
  std::size_t covered_event_count{0};
  std::int64_t maximum_adjacent_gap_ns{0};
  std::int64_t future_hold_ns{0};
  std::int64_t maximum_required_gap_ns{0};
};

template <std::size_t Capacity>
class CommandHistorySnapshot {
 public:
  static_assert(Capacity >= 2, "command history needs at least two events");

  CommandHistorySnapshot() : snapshot_steady_(0), snapshot_model_(0) {}

  bool empty() const noexcept { return size_ == 0; }
  std::size_t size() const noexcept { return size_; }
  std::uint64_t generation() const noexcept { return generation_; }
  std::uint64_t resetEpoch() const noexcept { return reset_epoch_; }
  SteadyTimeNs snapshotSteady() const noexcept { return snapshot_steady_; }
  ModelTimeNs snapshotModel() const noexcept { return snapshot_model_; }

  const AuthoritativePublisherState& publisherState() const noexcept {
    return publisher_state_;
  }

  const PublishedCommandEvent& event(std::size_t index) const {
    const PublishedCommandEvent* selected = eventIfPresent(index);
    if (selected == nullptr) {
      throw std::out_of_range("command history snapshot index is out of range");
    }
    return *selected;
  }

  const PublishedCommandEvent* eventIfPresent(
      std::size_t index) const noexcept {
    return index < size_ ? &events_[index] : nullptr;
  }

  HistorySampleResult sampleAt(ModelTimeNs stamp,
                               PublishedCommandEvent& output) const {
    if (size_ == 0) {
      return HistorySampleResult::kEmpty;
    }
    if (stamp.value < events_[0].cycle.release_model.value) {
      return HistorySampleResult::kBeforeHistory;
    }

    std::size_t selected = 0;
    for (std::size_t index = 1; index < size_; ++index) {
      if (events_[index].cycle.release_model.value > stamp.value) {
        break;
      }
      selected = index;
    }
    output = events_[selected];
    if (stamp.value == output.cycle.release_model.value) {
      return HistorySampleResult::kExact;
    }
    if (selected + 1 == size_) {
      return HistorySampleResult::kFutureHold;
    }
    return HistorySampleResult::kHeldBetweenEvents;
  }

  HistoryCoverage coverage(ModelTimeNs left, ModelTimeNs right,
                            std::int64_t maximum_gap_ns) const {
    HistoryCoverage result;
    result.history_generation = generation_;
    if (right.value < left.value || maximum_gap_ns < 0) {
      result.status = HistoryCoverageStatus::kInvalidRange;
      return result;
    }
    if (size_ == 0) {
      return result;
    }

    std::size_t predecessor = size_;
    for (std::size_t index = 0; index < size_; ++index) {
      if (events_[index].cycle.release_model.value <= left.value) {
        predecessor = index;
      } else {
        break;
      }
    }
    if (predecessor == size_) {
      result.status = HistoryCoverageStatus::kMissingPredecessor;
      return result;
    }

    result.predecessor_release_generation =
        events_[predecessor].release_generation;
    result.covered_event_count = 1;
    std::int64_t previous = events_[predecessor].cycle.release_model.value;
    for (std::size_t index = predecessor + 1; index < size_; ++index) {
      const std::int64_t current =
          events_[index].cycle.release_model.value;
      if (current > right.value) {
        break;
      }
      std::int64_t gap = 0;
      if (!checkedNonnegativeDifference(current, previous, gap)) {
        result.status = HistoryCoverageStatus::kTimeOverflow;
        return result;
      }
      result.maximum_adjacent_gap_ns =
          std::max(result.maximum_adjacent_gap_ns, gap);
      previous = current;
      ++result.covered_event_count;
    }

    if (!checkedNonnegativeDifference(right.value, previous,
                                      result.future_hold_ns)) {
      result.status = HistoryCoverageStatus::kTimeOverflow;
      return result;
    }
    result.maximum_required_gap_ns =
        std::max(result.maximum_adjacent_gap_ns, result.future_hold_ns);
    result.status = result.maximum_required_gap_ns > maximum_gap_ns
                        ? HistoryCoverageStatus::kGapTooLarge
                        : HistoryCoverageStatus::kComplete;
    return result;
  }

 private:
  template <std::size_t>
  friend class PublishedCommandHistory;

  static bool checkedNonnegativeDifference(std::int64_t later,
                                           std::int64_t earlier,
                                           std::int64_t& output) {
    if (later < earlier ||
        (earlier < 0 &&
         later > std::numeric_limits<std::int64_t>::max() + earlier)) {
      return false;
    }
    output = later - earlier;
    return true;
  }

  std::array<PublishedCommandEvent, Capacity> events_{};
  std::size_t size_{0};
  std::uint64_t generation_{0};
  std::uint64_t reset_epoch_{0};
  SteadyTimeNs snapshot_steady_;
  ModelTimeNs snapshot_model_;
  AuthoritativePublisherState publisher_state_;
};

// Fixed-capacity live history with one release-thread writer and one concurrent
// snapshot reader.  Three preallocated immutable buffers form a bounded RCU:
// the writer always has a buffer that is neither active nor reader-pinned.  A
// commit copies bounded storage but never allocates, locks, or waits.
template <std::size_t Capacity>
class PublishedCommandHistory {
 public:
  static_assert(Capacity >= 2, "command history needs at least two events");
  static_assert(std::is_trivially_copyable<PublishedCommandEvent>::value,
                "published events must have bounded trivial copies");

  PublishedCommandHistory(ClockAnchor anchor, std::uint64_t reset_epoch,
                          std::int64_t maximum_publish_lateness_ns,
                          std::uint64_t first_cycle_id = 0)
      : anchor_(anchor),
        reset_epoch_(reset_epoch),
        maximum_publish_lateness_ns_(maximum_publish_lateness_ns),
        next_cycle_id_(first_cycle_id),
        last_actual_steady_(0),
        last_actual_model_(0) {
    if (maximum_publish_lateness_ns < 0) {
      throw std::invalid_argument(
          "maximum publish lateness must be nonnegative");
    }
    for (std::atomic<std::uint64_t>& reader_count : reader_counts_) {
      reader_count.store(0, std::memory_order_relaxed);
    }
    if (!published_state_.is_lock_free() || !fault_latched_.is_lock_free() ||
        !reader_gate_.is_lock_free()) {
      throw std::runtime_error("command history atomics are not lock-free");
    }
    for (const std::atomic<std::uint64_t>& reader_count : reader_counts_) {
      if (!reader_count.is_lock_free()) {
        throw std::runtime_error("command history atomics are not lock-free");
      }
    }
  }

  PublishedCommandHistory(const PublishedCommandHistory&) = delete;
  PublishedCommandHistory& operator=(const PublishedCommandHistory&) = delete;

  HistoryCommitResult commitEmitted(const EmittedCommandCommit& commit,
                                    PublishedCommandEvent* committed_event =
                                        nullptr) {
    if (faultLatched()) {
      return HistoryCommitResult::kFaultLatched;
    }

    switch (commit.receipt.status) {
      case PublicationStatus::kWouldPublish:
        return HistoryCommitResult::kWouldPublish;
      case PublicationStatus::kFailed:
        return reject(HistoryCommitResult::kPublishFailed);
      case PublicationStatus::kPublished:
        break;
      default:
        return reject(HistoryCommitResult::kPublishFailed);
    }

    if (commit.expected_history_generation != writer_generation_) {
      return reject(HistoryCommitResult::kGenerationMismatch);
    }
    if (commit.cycle.reset_epoch != reset_epoch_) {
      return reject(HistoryCommitResult::kWrongEpoch);
    }
    if (commit.cycle.cycle_id != next_cycle_id_) {
      return reject(HistoryCommitResult::kWrongCycle);
    }
    if (next_cycle_id_ == std::numeric_limits<std::uint64_t>::max()) {
      return reject(HistoryCommitResult::kCycleOverflow);
    }

    CycleRequest expected;
    try {
      expected.cycle_id = next_cycle_id_;
      expected.release_steady =
          ReleaseGridContract::boundary(anchor_.steady, next_cycle_id_);
      expected.release_model =
          mapSteadyToModel(anchor_, expected.release_steady);
      expected.reset_epoch = reset_epoch_;
    } catch (const std::overflow_error&) {
      return reject(HistoryCommitResult::kTimeOverflow);
    }
    if (!sameCycleBoundary(commit.cycle, expected)) {
      return reject(HistoryCommitResult::kWrongBoundary);
    }
    if (!isKnownEmissionReason(commit.reason)) {
      return reject(HistoryCommitResult::kInvalidReason);
    }
    if (!std::isfinite(commit.command.linear) ||
        !std::isfinite(commit.command.angular) ||
        !std::isfinite(commit.receipt.command.linear) ||
        !std::isfinite(commit.receipt.command.angular)) {
      return reject(HistoryCommitResult::kInvalidCommand);
    }
    if (requiresZeroCommand(commit.reason) &&
        (commit.command.linear != 0.0 || commit.command.angular != 0.0)) {
      return reject(HistoryCommitResult::kInvalidCommand);
    }
    if (!std::isfinite(commit.authoritative_acceleration.linear) ||
        !std::isfinite(commit.authoritative_acceleration.angular) ||
        (requiresZeroPublisherAcceleration(commit.reason) &&
         (commit.authoritative_acceleration.linear != 0.0 ||
          commit.authoritative_acceleration.angular != 0.0))) {
      return reject(HistoryCommitResult::kInvalidAuthority);
    }
    if (!sameCycleBoundary(commit.receipt.cycle, commit.cycle) ||
        !sameCommand(commit.receipt.command, commit.command)) {
      return reject(HistoryCommitResult::kReceiptMismatch);
    }
    if (commit.receipt.actual_steady.value <
            commit.cycle.release_steady.value ||
        commit.receipt.actual_model.value < commit.cycle.release_model.value) {
      return reject(HistoryCommitResult::kPublishedTooEarly);
    }
    if (has_last_event_ &&
        (commit.receipt.actual_steady.value <= last_actual_steady_.value ||
         commit.receipt.actual_model.value <= last_actual_model_.value)) {
      return reject(HistoryCommitResult::kActualClockRegression);
    }

    std::int64_t lateness_ns = 0;
    if (!checkedNonnegativeDifference(commit.receipt.actual_steady.value,
                                      commit.cycle.release_steady.value,
                                      lateness_ns)) {
      return reject(HistoryCommitResult::kTimeOverflow);
    }
    const bool publish_late =
        lateness_ns > maximum_publish_lateness_ns_;
    // Lateness is known only after the publisher call.  Once the exact finite
    // command has crossed that boundary it remains an authoritative fact;
    // publish_late invalidates the trial and drives the next release decision.
    if (writer_generation_ == kMaximumPublishedGeneration) {
      return reject(HistoryCommitResult::kGenerationOverflow);
    }

    PublishedCommandEvent event;
    event.cycle = commit.cycle;
    event.actual_steady = commit.receipt.actual_steady;
    event.actual_model = commit.receipt.actual_model;
    event.command = commit.command;
    event.publisher_state_after.previous_linear_command =
        commit.command.linear;
    event.publisher_state_after.previous_angular_command =
        commit.command.angular;
    event.publisher_state_after.previous_linear_acceleration =
        commit.authoritative_acceleration.linear;
    event.publisher_state_after.previous_angular_acceleration =
        commit.authoritative_acceleration.angular;
    event.reason = commit.reason;
    event.release_generation = writer_generation_ + 1;
    event.publish_late = publish_late;

    const std::uint64_t published =
        published_state_.load(std::memory_order_seq_cst);
    const std::size_t active = publishedIndex(published);
    std::size_t destination = kBufferCount;
    for (std::size_t index = 0; index < kBufferCount; ++index) {
      if (index != active &&
          reader_counts_[index].load(std::memory_order_seq_cst) == 0) {
        destination = index;
        break;
      }
    }
    if (destination == kBufferCount) {
      return reject(HistoryCommitResult::kSnapshotBusy);
    }
    buffers_[destination] = buffers_[active];
    buffers_[destination].append(event);
    published_state_.store(
        makePublishedState(event.release_generation, destination),
        std::memory_order_seq_cst);

    writer_generation_ = event.release_generation;
    ++next_cycle_id_;
    has_last_event_ = true;
    last_actual_steady_ = event.actual_steady;
    last_actual_model_ = event.actual_model;
    if (committed_event != nullptr) {
      *committed_event = event;
    }
    return HistoryCommitResult::kCommitted;
  }

  HistorySnapshotResult capture(
      SteadyTimeNs snapshot_steady, ModelTimeNs snapshot_model,
      CommandHistorySnapshot<Capacity>& output) const {
    if (reader_gate_.exchange(1, std::memory_order_acq_rel) != 0) {
      return HistorySnapshotResult::kReaderBusy;
    }

    for (std::size_t attempt = 0; attempt < kBufferCount; ++attempt) {
      const std::uint64_t published =
          published_state_.load(std::memory_order_seq_cst);
      const std::size_t active = publishedIndex(published);
      reader_counts_[active].fetch_add(1, std::memory_order_seq_cst);
      if (published_state_.load(std::memory_order_seq_cst) != published) {
        reader_counts_[active].fetch_sub(1, std::memory_order_seq_cst);
        continue;
      }

      const StorageBuffer& storage = buffers_[active];
      const std::uint64_t generation = publishedGeneration(published);
      if (generation == 0) {
        reader_counts_[active].fetch_sub(1, std::memory_order_seq_cst);
        reader_gate_.store(0, std::memory_order_release);
        return HistorySnapshotResult::kEmpty;
      }
      if (storage.generation != generation) {
        reader_counts_[active].fetch_sub(1, std::memory_order_seq_cst);
        continue;
      }

      CommandHistorySnapshot<Capacity> candidate;
      candidate.generation_ = generation;
      candidate.reset_epoch_ = reset_epoch_;
      candidate.snapshot_steady_ = snapshot_steady;
      candidate.snapshot_model_ = snapshot_model;
      candidate.size_ = storage.size;
      for (std::size_t index = 0; index < storage.size; ++index) {
        candidate.events_[index] =
            storage.events[StorageBuffer::wrappedIndex(storage.start, index)];
      }
      candidate.publisher_state_ =
          candidate.events_[candidate.size_ - 1].publisher_state_after;

      reader_counts_[active].fetch_sub(1, std::memory_order_seq_cst);
      reader_gate_.store(0, std::memory_order_release);

      const PublishedCommandEvent& latest =
          candidate.events_[candidate.size_ - 1];
      if (snapshot_steady.value < latest.actual_steady.value ||
          snapshot_model.value < latest.actual_model.value) {
        return HistorySnapshotResult::kSnapshotClockRegression;
      }
      output = candidate;
      return HistorySnapshotResult::kReady;
    }

    reader_gate_.store(0, std::memory_order_release);
    return HistorySnapshotResult::kConcurrentWrite;
  }

  std::uint64_t generation() const noexcept {
    return publishedGeneration(
        published_state_.load(std::memory_order_seq_cst));
  }

  std::uint64_t resetEpoch() const noexcept { return reset_epoch_; }

  bool faultLatched() const noexcept {
    return fault_latched_.load(std::memory_order_acquire) != 0;
  }

  // Single-writer diagnostic only.  Planning readers must use capture().
  std::uint64_t nextCycleIdForWriter() const noexcept {
    return next_cycle_id_;
  }

 private:
  static constexpr std::size_t kBufferCount = 3;
  static constexpr std::uint64_t kPublishedIndexBits = 2;
  static constexpr std::uint64_t kPublishedIndexMask =
      (1ULL << kPublishedIndexBits) - 1ULL;
  static constexpr std::uint64_t kMaximumPublishedGeneration =
      std::numeric_limits<std::uint64_t>::max() >> kPublishedIndexBits;

  struct StorageBuffer {
    std::array<PublishedCommandEvent, Capacity> events{};
    std::size_t start{0};
    std::size_t size{0};
    std::uint64_t generation{0};

    static constexpr std::size_t wrappedIndex(std::size_t base,
                                              std::size_t offset) {
      return offset >= Capacity - base ? offset - (Capacity - base)
                                       : base + offset;
    }

    void append(const PublishedCommandEvent& event) {
      std::size_t destination = 0;
      if (size < Capacity) {
        destination = wrappedIndex(start, size);
        ++size;
      } else {
        destination = start;
        start = wrappedIndex(start, 1);
      }
      events[destination] = event;
      generation = event.release_generation;
    }
  };

  static constexpr std::uint64_t makePublishedState(
      std::uint64_t generation, std::size_t index) {
    return (generation << kPublishedIndexBits) |
           static_cast<std::uint64_t>(index);
  }

  static constexpr std::size_t publishedIndex(std::uint64_t state) {
    return static_cast<std::size_t>(state & kPublishedIndexMask);
  }

  static constexpr std::uint64_t publishedGeneration(std::uint64_t state) {
    return state >> kPublishedIndexBits;
  }

  static bool sameCycleBoundary(const CycleRequest& lhs,
                                const CycleRequest& rhs) {
    return lhs.cycle_id == rhs.cycle_id &&
           lhs.release_steady.value == rhs.release_steady.value &&
           lhs.release_model.value == rhs.release_model.value &&
           lhs.reset_epoch == rhs.reset_epoch;
  }

  static bool sameCommand(const PlanarCommand& lhs,
                          const PlanarCommand& rhs) {
    return lhs.linear == rhs.linear && lhs.angular == rhs.angular;
  }

  static bool checkedNonnegativeDifference(std::int64_t later,
                                           std::int64_t earlier,
                                           std::int64_t& output) {
    if (later < earlier ||
        (earlier < 0 &&
         later > std::numeric_limits<std::int64_t>::max() + earlier)) {
      return false;
    }
    output = later - earlier;
    return true;
  }

  HistoryCommitResult reject(HistoryCommitResult result) {
    fault_latched_.store(1, std::memory_order_release);
    return result;
  }

  std::array<StorageBuffer, kBufferCount> buffers_{};
  const ClockAnchor anchor_;
  const std::uint64_t reset_epoch_{0};
  const std::int64_t maximum_publish_lateness_ns_{0};
  std::atomic<std::uint64_t> published_state_{0};
  std::atomic<std::uint64_t> fault_latched_{0};
  mutable std::array<std::atomic<std::uint64_t>, kBufferCount> reader_counts_;
  mutable std::atomic<std::uint64_t> reader_gate_{0};

  // Written and read only by the single release-owner thread.
  std::uint64_t writer_generation_{0};
  std::uint64_t next_cycle_id_{0};
  bool has_last_event_{false};
  SteadyTimeNs last_actual_steady_;
  ModelTimeNs last_actual_model_;
};

}  // namespace mainline
}  // namespace spmpc_local_planner
