#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "spmpc_local_planner/domain/content_identity.h"

namespace spmpc_local_planner {
namespace mainline {

struct ReferencePathIdentity {
  std::uint64_t path_id{0};
  Sha256Digest path_hash;
  std::uint64_t reset_epoch{0};
};

inline bool operator==(const ReferencePathIdentity& lhs,
                       const ReferencePathIdentity& rhs) noexcept {
  return lhs.path_id == rhs.path_id && lhs.path_hash == rhs.path_hash &&
         lhs.reset_epoch == rhs.reset_epoch;
}

inline bool operator!=(const ReferencePathIdentity& lhs,
                       const ReferencePathIdentity& rhs) noexcept {
  return !(lhs == rhs);
}

struct ReferencePathVertex {
  double x{0.0};
  double y{0.0};
  double cumulative_s{0.0};
};

// Immutable-by-contract input assembled before a planning cycle.  The
// projector validates it but never repairs, resamples, or re-hashes it.
template <std::size_t VertexCapacity>
struct ReferencePathSnapshot {
  static_assert(VertexCapacity >= 2,
                "reference path needs at least two vertex slots");

  ReferencePathIdentity identity;
  std::array<ReferencePathVertex, VertexCapacity> vertices{};
  std::size_t vertex_count{0};
  double s_path_end{0.0};
};

}  // namespace mainline
}  // namespace spmpc_local_planner
