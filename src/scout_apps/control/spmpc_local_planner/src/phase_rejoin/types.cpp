#include "spmpc_local_planner/phase_rejoin/types.h"

#include <algorithm>
#include <cctype>

namespace spmpc_local_planner {
namespace {

std::string lowercase(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

}  // namespace

std::string phaseRejoinModeName(PhaseRejoinMode mode) {
    switch (mode) {
    case PhaseRejoinMode::Monitor:
        return "monitor";
    case PhaseRejoinMode::Enforce:
        return "enforce";
    case PhaseRejoinMode::Off:
    default:
        return "off";
    }
}

bool parsePhaseRejoinMode(const std::string& text, PhaseRejoinMode& mode) {
    const std::string value = lowercase(text);
    if (value == "off") {
        mode = PhaseRejoinMode::Off;
        return true;
    }
    if (value == "monitor" || value == "shadow") {
        mode = PhaseRejoinMode::Monitor;
        return true;
    }
    if (value == "enforce" || value == "active") {
        mode = PhaseRejoinMode::Enforce;
        return true;
    }
    return false;
}

std::string phaseRejoinEvidenceLevelName(PhaseRejoinEvidenceLevel level) {
    switch (level) {
    case PhaseRejoinEvidenceLevel::DevelopmentOnly:
        return "development_only";
    case PhaseRejoinEvidenceLevel::EmpiricalHeldOut:
        return "empirical_held_out";
    case PhaseRejoinEvidenceLevel::Unknown:
    default:
        return "unknown";
    }
}

bool parsePhaseRejoinEvidenceLevel(const std::string& text,
                                   PhaseRejoinEvidenceLevel& level) {
    const std::string value = lowercase(text);
    if (value == "development_only") {
        level = PhaseRejoinEvidenceLevel::DevelopmentOnly;
        return true;
    }
    if (value == "empirical_held_out") {
        level = PhaseRejoinEvidenceLevel::EmpiricalHeldOut;
        return true;
    }
    return false;
}

}  // namespace spmpc_local_planner
