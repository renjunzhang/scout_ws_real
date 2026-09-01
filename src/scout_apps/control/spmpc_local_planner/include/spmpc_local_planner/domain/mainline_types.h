#pragma once

#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace spmpc_local_planner {
namespace mainline {

// The two time domains deliberately use distinct types.  Converting between
// them without a frozen ClockAnchor is a contract violation.
struct SteadyTimeNs {
  explicit constexpr SteadyTimeNs(std::int64_t value_ns = 0) : value(value_ns) {}
  std::int64_t value;
};

struct ModelTimeNs {
  explicit constexpr ModelTimeNs(std::int64_t value_ns = 0) : value(value_ns) {}
  std::int64_t value;
};

static_assert(!std::is_convertible<SteadyTimeNs, ModelTimeNs>::value,
              "steady time must not implicitly convert to model time");
static_assert(!std::is_convertible<ModelTimeNs, SteadyTimeNs>::value,
              "model time must not implicitly convert to steady time");

struct ClockAnchor {
  SteadyTimeNs steady;
  ModelTimeNs model;
};

struct CycleRequest {
  CycleRequest() : release_steady(0), release_model(0) {}

  std::uint64_t cycle_id{0};
  SteadyTimeNs release_steady;
  ModelTimeNs release_model;
  std::uint64_t reset_epoch{0};
};

enum class ExperimentCondition : std::uint8_t {
  kB0 = 0,
  kBslosh = 1,
};

inline double liquidObjectiveScale(ExperimentCondition condition) {
  switch (condition) {
    case ExperimentCondition::kB0:
      return 0.0;
    case ExperimentCondition::kBslosh:
      return 1.0;
  }
  throw std::invalid_argument("unknown mainline experiment condition");
}

}  // namespace mainline
}  // namespace spmpc_local_planner
