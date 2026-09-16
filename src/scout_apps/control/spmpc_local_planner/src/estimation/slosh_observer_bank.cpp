#include "spmpc_local_planner/estimation/slosh_observer_bank.h"

#include <cmath>

namespace spmpc_local_planner {

namespace {

// Every accepted observer interval must account for the elapsed state time.
// A reported short dt after a timestamp gap would otherwise silently skip the
// missing physical interval and leave the liquid state discontinuous.
constexpr double kContinuityDtToleranceSec = 1.0e-6;

}  // namespace

bool SloshObserverBank::configure(
    const SloshModelParams& params,
    double imu_observer_dt_sec) {
    base_params_ = params;

    const bool odom_ok = odom_dynamics_.configure(base_params_);

    SloshModelParams imu_params = base_params_;
    imu_params.dt = imu_observer_dt_sec;
    const bool imu_ok = std::isfinite(imu_observer_dt_sec) && imu_observer_dt_sec > 0.0 &&
                        imu_dynamics_.configure(imu_params);

    odom_snapshot_ = SloshObserverSnapshot();
    imu_snapshot_ = SloshObserverSnapshot();
    odom_snapshot_.configured = odom_ok;
    imu_snapshot_.configured = imu_ok;
    have_imu_epoch_ = false;
    imu_epoch_ = 0;
    odom_needs_initialization_ = false;
    imu_needs_initialization_ = false;
    return odom_ok;
}

void SloshObserverBank::resetOdom() {
    const bool configured = odom_dynamics_.configured();
    odom_snapshot_ = SloshObserverSnapshot();
    odom_snapshot_.configured = configured;
    odom_needs_initialization_ = false;
}

void SloshObserverBank::invalidateOdom() {
    odom_snapshot_.valid = false;
    if (odom_snapshot_.update_count > 0) {
        odom_needs_initialization_ = true;
    }
}

void SloshObserverBank::resetImu() {
    const bool configured = imu_dynamics_.configured();
    imu_snapshot_ = SloshObserverSnapshot();
    imu_snapshot_.configured = configured;
    have_imu_epoch_ = false;
    imu_epoch_ = 0;
    imu_needs_initialization_ = false;
}

bool SloshObserverBank::stepOdom(const MotionExcitation& excitation) {
    if (discontinuousInterval(odom_snapshot_, excitation)) {
        odom_needs_initialization_ = true;
        odom_snapshot_.valid = false;
        odom_snapshot_.excitation = excitation;
        return false;
    }
    if (!odom_dynamics_.configured() || !excitation.valid ||
        excitation.source != MotionExcitationSource::Odom ||
        !finiteExcitation(excitation) || observerStamp(excitation) <= 0 ||
        odom_needs_initialization_ ||
        (odom_snapshot_.update_count > 0 &&
         observerStamp(excitation) <= odom_snapshot_.state_stamp_ns)) {
        odom_snapshot_.configured = odom_dynamics_.configured();
        odom_snapshot_.valid = false;
        odom_snapshot_.excitation = excitation;
        return false;
    }

    SloshState next_state;
    if (!odom_dynamics_.stepWithDt(
            odom_snapshot_.state, excitation.atContainer(), excitation.sample_dt_sec, next_state)) {
        odom_snapshot_.valid = false;
        if (odom_snapshot_.update_count > 0) {
            odom_needs_initialization_ = true;
        }
        odom_snapshot_.excitation = excitation;
        return false;
    }
    odom_snapshot_.state = next_state;
    odom_snapshot_.configured = true;
    odom_snapshot_.valid = true;
    odom_needs_initialization_ = false;
    odom_snapshot_.excitation = excitation;
    odom_snapshot_.state_stamp_ns = observerStamp(excitation);
    ++odom_snapshot_.update_count;
    refreshSnapshotHeight(odom_dynamics_, odom_snapshot_);
    return true;
}

bool SloshObserverBank::stepImu(const MotionExcitation& excitation) {
    if (!have_imu_epoch_ || excitation.reset_epoch != imu_epoch_) {
        const bool configured = imu_dynamics_.configured();
        const bool had_state = imu_snapshot_.update_count > 0;
        imu_snapshot_ = SloshObserverSnapshot();
        imu_snapshot_.configured = configured;
        imu_needs_initialization_ = imu_needs_initialization_ || had_state;
        have_imu_epoch_ = true;
        imu_epoch_ = excitation.reset_epoch;
    }

    if (discontinuousInterval(imu_snapshot_, excitation)) {
        imu_needs_initialization_ = true;
        imu_snapshot_.valid = false;
        imu_snapshot_.excitation = excitation;
        return false;
    }

    if (!imu_dynamics_.configured() || !excitation.valid ||
        excitation.source != MotionExcitationSource::ProcessedImu ||
        !finiteExcitation(excitation) || observerStamp(excitation) <= 0 ||
        imu_needs_initialization_ ||
        (imu_snapshot_.update_count > 0 &&
         observerStamp(excitation) <= imu_snapshot_.state_stamp_ns)) {
        imu_snapshot_.configured = imu_dynamics_.configured();
        imu_snapshot_.valid = false;
        imu_snapshot_.excitation = excitation;
        return false;
    }

    // Accepted, aligned excitation interval; the same RHS/integrator as OCP.
    SloshState next_state;
    if (!imu_dynamics_.stepWithDt(
            imu_snapshot_.state,
            excitation.atContainer(),
            excitation.sample_dt_sec,
            next_state)) {
        imu_snapshot_.valid = false;
        if (imu_snapshot_.update_count > 0) {
            imu_needs_initialization_ = true;
        }
        imu_snapshot_.excitation = excitation;
        return false;
    }
    imu_snapshot_.state = next_state;
    imu_snapshot_.configured = true;
    imu_snapshot_.valid = true;
    imu_needs_initialization_ = false;
    imu_snapshot_.excitation = excitation;
    imu_snapshot_.state_stamp_ns = observerStamp(excitation);
    ++imu_snapshot_.update_count;
    refreshSnapshotHeight(imu_dynamics_, imu_snapshot_);
    return true;
}

bool SloshObserverBank::initializeOdom(
    const SloshState& state,
    const MotionExcitation& excitation) {
    if (!odom_dynamics_.configured() || !finiteState(state) || !excitation.valid ||
        excitation.source != MotionExcitationSource::Odom ||
        !finiteExcitation(excitation) || observerStamp(excitation) <= 0) {
        odom_snapshot_.configured = odom_dynamics_.configured();
        odom_snapshot_.valid = false;
        odom_snapshot_.excitation = excitation;
        return false;
    }
    odom_snapshot_.state = state;
    odom_snapshot_.configured = true;
    odom_snapshot_.valid = true;
    odom_snapshot_.excitation = excitation;
    odom_snapshot_.state_stamp_ns = observerStamp(excitation);
    odom_snapshot_.update_count = 1;
    odom_needs_initialization_ = false;
    refreshSnapshotHeight(odom_dynamics_, odom_snapshot_);
    return true;
}

bool SloshObserverBank::initializeImu(
    const SloshState& state,
    const MotionExcitation& excitation) {
    if (!imu_dynamics_.configured() || !finiteState(state) || !excitation.valid ||
        excitation.source != MotionExcitationSource::ProcessedImu ||
        !finiteExcitation(excitation) || observerStamp(excitation) <= 0) {
        imu_snapshot_.configured = imu_dynamics_.configured();
        imu_snapshot_.valid = false;
        imu_snapshot_.excitation = excitation;
        return false;
    }
    if (!have_imu_epoch_ || excitation.reset_epoch != imu_epoch_) {
        have_imu_epoch_ = true;
        imu_epoch_ = excitation.reset_epoch;
    }
    imu_snapshot_.state = state;
    imu_snapshot_.configured = true;
    imu_snapshot_.valid = true;
    imu_snapshot_.excitation = excitation;
    imu_snapshot_.state_stamp_ns = observerStamp(excitation);
    imu_snapshot_.update_count = 1;
    imu_needs_initialization_ = false;
    refreshSnapshotHeight(imu_dynamics_, imu_snapshot_);
    return true;
}

void SloshObserverBank::invalidateImu(std::uint32_t reset_epoch) {
    if (!have_imu_epoch_ || reset_epoch != imu_epoch_) {
        const bool configured = imu_dynamics_.configured();
        const bool had_state = imu_snapshot_.update_count > 0;
        imu_snapshot_ = SloshObserverSnapshot();
        imu_snapshot_.configured = configured;
        imu_needs_initialization_ = imu_needs_initialization_ || had_state;
        have_imu_epoch_ = true;
        imu_epoch_ = reset_epoch;
    }
    imu_snapshot_.valid = false;
    if (imu_snapshot_.update_count > 0) {
        imu_needs_initialization_ = true;
    }
}

double SloshObserverBank::heightCoeff() const {
    return odom_dynamics_.configured() ? odom_dynamics_.heightCoeff() : 0.0;
}

bool SloshObserverBank::finiteState(const SloshState& state) {
    return std::isfinite(state.eta_x) && std::isfinite(state.eta_x_dot) &&
           std::isfinite(state.eta_y) && std::isfinite(state.eta_y_dot);
}

bool SloshObserverBank::discontinuousInterval(
    const SloshObserverSnapshot& snapshot,
    const MotionExcitation& excitation) {
    if (snapshot.update_count == 0 || snapshot.state_stamp_ns <= 0) {
        return false;
    }
    const std::int64_t stamp = observerStamp(excitation);
    if (stamp <= snapshot.state_stamp_ns) {
        return false;
    }
    const double stamp_dt = static_cast<double>(stamp - snapshot.state_stamp_ns) * 1.0e-9;
    return !std::isfinite(stamp_dt) || !std::isfinite(excitation.sample_dt_sec) ||
        std::abs(stamp_dt - excitation.sample_dt_sec) > kContinuityDtToleranceSec;
}

double SloshObserverBank::solverHeight(const SloshState& state, double omega_z) const {
    return odom_dynamics_.configured() ? odom_dynamics_.height(state, omega_z) : 0.0;
}

bool SloshObserverBank::finiteExcitation(const MotionExcitation& excitation) {
    return std::isfinite(excitation.ax) && std::isfinite(excitation.ay) &&
           std::isfinite(excitation.omega_z) && std::isfinite(excitation.alpha_z) &&
           std::isfinite(excitation.sample_dt_sec) && excitation.sample_dt_sec > 1e-4;
}

std::int64_t SloshObserverBank::observerStamp(const MotionExcitation& excitation) {
    if (excitation.source == MotionExcitationSource::ProcessedImu &&
        excitation.accel_effective_stamp_ns > 0) {
        return excitation.accel_effective_stamp_ns;
    }
    return excitation.measurement_stamp_ns > 0
        ? excitation.measurement_stamp_ns
        : excitation.source_stamp_ns;
}

double SloshObserverBank::modalHeight(
    const SloshDynamics& dynamics,
    const SloshState& state) {
    return dynamics.configured() ? dynamics.heightCoeff() * dynamics.etaNorm(state) : 0.0;
}

void SloshObserverBank::refreshSnapshotHeight(
    const SloshDynamics& dynamics,
    SloshObserverSnapshot& snapshot) {
    snapshot.modal_height_m = modalHeight(dynamics, snapshot.state);
    snapshot.total_height_m = dynamics.configured()
        ? dynamics.height(snapshot.state, snapshot.excitation.omega_z)
        : 0.0;
}

}  // namespace spmpc_local_planner
