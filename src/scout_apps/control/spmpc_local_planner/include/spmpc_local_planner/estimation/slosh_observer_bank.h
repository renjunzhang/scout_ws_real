#pragma once

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/estimation/motion_excitation.h"

#include <cstdint>

namespace spmpc_local_planner {

// One independently-integrated liquid observer channel.  A channel can retain
// its last numerical state for diagnostics while valid=false; consumers must
// never interpret an invalid snapshot as a current measurement.
struct SloshObserverSnapshot {
    bool configured = false;
    bool valid = false;
    SloshState state;
    MotionExcitation excitation;
    std::int64_t state_stamp_ns = 0;
    std::uint64_t update_count = 0;
    double modal_height_m = 0.0;
    double total_height_m = 0.0;
};

// Owns two physically identical but completely independent observer states.
// Source admission is intentionally outside this integration bank and is
// handled by SloshObserverSelector at the control-cycle boundary.
class SloshObserverBank {
public:
    bool configure(const SloshModelParams& params, double imu_observer_dt_sec);

    void resetOdom();
    void resetImu();

    bool stepOdom(const MotionExcitation& excitation);
    bool stepImu(const MotionExcitation& excitation);
    void invalidateImu(std::uint32_t reset_epoch);

    const SloshObserverSnapshot& odom() const { return odom_snapshot_; }
    const SloshObserverSnapshot& imu() const { return imu_snapshot_; }

    // Legacy odom accessor retained for exact pre-selector equivalence tests.
    // Runtime solver input must go through SloshObserverSelector.
    const SloshState& solverState() const { return odom_snapshot_.state; }

    bool odomConfigured() const { return odom_dynamics_.configured(); }
    bool imuConfigured() const { return imu_dynamics_.configured(); }
    double heightCoeff() const;
    double solverHeight(const SloshState& state, double omega_z) const;

private:
    static bool finiteExcitation(const MotionExcitation& excitation);
    static std::int64_t observerStamp(const MotionExcitation& excitation);
    static double modalHeight(const SloshDynamics& dynamics, const SloshState& state);
    static void refreshSnapshotHeight(const SloshDynamics& dynamics,
                                      SloshObserverSnapshot& snapshot);

    SloshDynamics odom_dynamics_;
    SloshDynamics imu_dynamics_;
    SloshModelParams base_params_;
    bool have_imu_epoch_ = false;
    std::uint32_t imu_epoch_ = 0;
    SloshObserverSnapshot odom_snapshot_;
    SloshObserverSnapshot imu_snapshot_;
};

}  // namespace spmpc_local_planner
