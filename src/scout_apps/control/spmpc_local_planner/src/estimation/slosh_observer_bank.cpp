#include "spmpc_local_planner/estimation/slosh_observer_bank.h"

#include <cmath>

namespace spmpc_local_planner {

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
    return odom_ok;
}

void SloshObserverBank::resetOdom() {
    const bool configured = odom_dynamics_.configured();
    odom_snapshot_ = SloshObserverSnapshot();
    odom_snapshot_.configured = configured;
}

void SloshObserverBank::resetImu() {
    const bool configured = imu_dynamics_.configured();
    imu_snapshot_ = SloshObserverSnapshot();
    imu_snapshot_.configured = configured;
    have_imu_epoch_ = false;
    imu_epoch_ = 0;
}

bool SloshObserverBank::stepOdom(const MotionExcitation& excitation) {
    if (!odom_dynamics_.configured() || !excitation.valid ||
        excitation.source != MotionExcitationSource::Odom ||
        !finiteExcitation(excitation) || observerStamp(excitation) <= 0 ||
        (odom_snapshot_.update_count > 0 &&
         observerStamp(excitation) <= odom_snapshot_.state_stamp_ns)) {
        odom_snapshot_.configured = odom_dynamics_.configured();
        odom_snapshot_.valid = false;
        odom_snapshot_.excitation = excitation;
        return false;
    }

    // Preserve the pre-refactor odom behavior: discretize at the message dt.
    if (std::abs(excitation.sample_dt_sec - odom_dynamics_.params().dt) > 1e-4) {
        SloshModelParams params = base_params_;
        params.dt = excitation.sample_dt_sec;
        if (!odom_dynamics_.configure(params)) {
            odom_snapshot_.configured = false;
            odom_snapshot_.valid = false;
            odom_snapshot_.excitation = excitation;
            return false;
        }
    }

    odom_snapshot_.state = odom_dynamics_.step(
        odom_snapshot_.state, excitation.ax, excitation.ay, excitation.omega_z);
    odom_snapshot_.configured = true;
    odom_snapshot_.valid = true;
    odom_snapshot_.excitation = excitation;
    odom_snapshot_.state_stamp_ns = observerStamp(excitation);
    ++odom_snapshot_.update_count;
    refreshSnapshotHeight(odom_dynamics_, odom_snapshot_);
    return true;
}

bool SloshObserverBank::stepImu(const MotionExcitation& excitation) {
    if (!have_imu_epoch_ || excitation.reset_epoch != imu_epoch_) {
        const bool configured = imu_dynamics_.configured();
        imu_snapshot_ = SloshObserverSnapshot();
        imu_snapshot_.configured = configured;
        have_imu_epoch_ = true;
        imu_epoch_ = excitation.reset_epoch;
    }

    if (!imu_dynamics_.configured() || !excitation.valid ||
        excitation.source != MotionExcitationSource::ProcessedImu ||
        !finiteExcitation(excitation) || observerStamp(excitation) <= 0 ||
        (imu_snapshot_.update_count > 0 &&
         observerStamp(excitation) <= imu_snapshot_.state_stamp_ns)) {
        imu_snapshot_.configured = imu_dynamics_.configured();
        imu_snapshot_.valid = false;
        imu_snapshot_.excitation = excitation;
        return false;
    }

    // Use the accepted sensor interval so the shadow state's physical time
    // cannot drift from its published timestamp.  This exact variable-dt path
    // is isolated from the fixed-step solver/odom dynamics.
    SloshState next_state;
    if (!imu_dynamics_.stepWithDt(
            imu_snapshot_.state,
            excitation.ax,
            excitation.ay,
            excitation.omega_z,
            excitation.sample_dt_sec,
            next_state)) {
        imu_snapshot_.valid = false;
        imu_snapshot_.excitation = excitation;
        return false;
    }
    imu_snapshot_.state = next_state;
    imu_snapshot_.configured = true;
    imu_snapshot_.valid = true;
    imu_snapshot_.excitation = excitation;
    imu_snapshot_.state_stamp_ns = observerStamp(excitation);
    ++imu_snapshot_.update_count;
    refreshSnapshotHeight(imu_dynamics_, imu_snapshot_);
    return true;
}

void SloshObserverBank::invalidateImu(std::uint32_t reset_epoch) {
    if (!have_imu_epoch_ || reset_epoch != imu_epoch_) {
        const bool configured = imu_dynamics_.configured();
        imu_snapshot_ = SloshObserverSnapshot();
        imu_snapshot_.configured = configured;
        have_imu_epoch_ = true;
        imu_epoch_ = reset_epoch;
    }
    imu_snapshot_.valid = false;
}

double SloshObserverBank::heightCoeff() const {
    return odom_dynamics_.configured() ? odom_dynamics_.heightCoeff() : 0.0;
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
