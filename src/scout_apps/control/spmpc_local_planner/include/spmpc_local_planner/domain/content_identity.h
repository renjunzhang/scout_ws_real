#pragma once

#include <array>
#include <cstdint>

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

inline bool isZeroDigest(const Sha256Digest& digest) noexcept {
  for (const std::uint8_t value : digest.bytes) {
    if (value != 0) {
      return false;
    }
  }
  return true;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
