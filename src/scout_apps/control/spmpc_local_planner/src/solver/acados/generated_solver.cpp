#include "spmpc_local_planner/solver/acados/generated_solver.h"

#ifdef SPMPC_WITH_ACADOS

#include "acados_solver_spmpc_b0.h"
#ifdef SPMPC_WITH_ACADOS_SLOSH
#include "acados_solver_spmpc_slosh.h"
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
#include "acados_solver_spmpc_phase_rejoin.h"
#endif
#include "acados_c/ocp_nlp_interface.h"

#include <vector>

namespace spmpc_local_planner {
namespace {

struct B0CapsuleDeleter {
    void operator()(spmpc_b0_solver_capsule* capsule) const {
        if (capsule == nullptr) return;
        spmpc_b0_acados_free(capsule);
        spmpc_b0_acados_free_capsule(capsule);
    }
};

#ifdef SPMPC_WITH_ACADOS_SLOSH
struct SloshCapsuleDeleter {
    void operator()(spmpc_slosh_solver_capsule* capsule) const {
        if (capsule == nullptr) return;
        spmpc_slosh_acados_free(capsule);
        spmpc_slosh_acados_free_capsule(capsule);
    }
};
#endif

#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
struct PhaseRejoinCapsuleDeleter {
    void operator()(spmpc_phase_rejoin_solver_capsule* capsule) const {
        if (capsule == nullptr) return;
        spmpc_phase_rejoin_acados_free(capsule);
        spmpc_phase_rejoin_acados_free_capsule(capsule);
    }
};
#endif

}  // namespace

struct GeneratedAcadosSolver::Impl {
    Kind kind = Kind::B0;
    std::unique_ptr<spmpc_b0_solver_capsule, B0CapsuleDeleter> b0;
#ifdef SPMPC_WITH_ACADOS_SLOSH
    std::unique_ptr<spmpc_slosh_solver_capsule, SloshCapsuleDeleter> slosh;
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
    std::unique_ptr<spmpc_phase_rejoin_solver_capsule,
                    PhaseRejoinCapsuleDeleter> phase_rejoin;
#endif
    int nx = 0;
    int nu = 0;
    int np = 0;
    int horizon = 0;

    bool ready() const {
        if (kind == Kind::B0) return b0 != nullptr;
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == Kind::SLOSH) return slosh != nullptr;
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == Kind::PHASE_REJOIN) return phase_rejoin != nullptr;
#endif
        return false;
    }

    ocp_nlp_config* config() const {
        if (kind == Kind::B0) {
            return spmpc_b0_acados_get_nlp_config(b0.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == Kind::SLOSH) {
            return spmpc_slosh_acados_get_nlp_config(slosh.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == Kind::PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_config(
                phase_rejoin.get());
        }
#endif
        return nullptr;
    }

    ocp_nlp_dims* dims() const {
        if (kind == Kind::B0) {
            return spmpc_b0_acados_get_nlp_dims(b0.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == Kind::SLOSH) {
            return spmpc_slosh_acados_get_nlp_dims(slosh.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == Kind::PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_dims(
                phase_rejoin.get());
        }
#endif
        return nullptr;
    }

    ocp_nlp_in* input() const {
        if (kind == Kind::B0) {
            return spmpc_b0_acados_get_nlp_in(b0.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == Kind::SLOSH) {
            return spmpc_slosh_acados_get_nlp_in(slosh.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == Kind::PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_in(
                phase_rejoin.get());
        }
#endif
        return nullptr;
    }

    ocp_nlp_out* output() const {
        if (kind == Kind::B0) {
            return spmpc_b0_acados_get_nlp_out(b0.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == Kind::SLOSH) {
            return spmpc_slosh_acados_get_nlp_out(slosh.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == Kind::PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_out(
                phase_rejoin.get());
        }
#endif
        return nullptr;
    }

    ocp_nlp_solver* solver() const {
        if (kind == Kind::B0) {
            return spmpc_b0_acados_get_nlp_solver(b0.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == Kind::SLOSH) {
            return spmpc_slosh_acados_get_nlp_solver(slosh.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == Kind::PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_solver(
                phase_rejoin.get());
        }
#endif
        return nullptr;
    }
};

GeneratedAcadosSolver::GeneratedAcadosSolver() : impl_(new Impl()) {}
GeneratedAcadosSolver::~GeneratedAcadosSolver() = default;

void GeneratedAcadosSolver::reset() {
    impl_.reset(new Impl());
}

bool GeneratedAcadosSolver::create(Kind kind) {
    reset();
    impl_->kind = kind;
    if (kind == Kind::B0) {
        spmpc_b0_solver_capsule* capsule =
            spmpc_b0_acados_create_capsule();
        if (capsule == nullptr) return false;
        if (spmpc_b0_acados_create(capsule) != 0) {
            spmpc_b0_acados_free_capsule(capsule);
            return false;
        }
        impl_->b0.reset(capsule);
        impl_->nx = SPMPC_B0_NX;
        impl_->nu = SPMPC_B0_NU;
        impl_->np = SPMPC_B0_NP;
        impl_->horizon = SPMPC_B0_N;
        return true;
    }
    if (kind == Kind::SLOSH) {
#ifdef SPMPC_WITH_ACADOS_SLOSH
        spmpc_slosh_solver_capsule* capsule =
            spmpc_slosh_acados_create_capsule();
        if (capsule == nullptr) return false;
        if (spmpc_slosh_acados_create(capsule) != 0) {
            spmpc_slosh_acados_free_capsule(capsule);
            return false;
        }
        impl_->slosh.reset(capsule);
        impl_->nx = SPMPC_SLOSH_NX;
        impl_->nu = SPMPC_SLOSH_NU;
        impl_->np = SPMPC_SLOSH_NP;
        impl_->horizon = SPMPC_SLOSH_N;
        return true;
#else
        return false;
#endif
    }
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
    spmpc_phase_rejoin_solver_capsule* capsule =
        spmpc_phase_rejoin_acados_create_capsule();
    if (capsule == nullptr) return false;
    if (spmpc_phase_rejoin_acados_create(capsule) != 0) {
        spmpc_phase_rejoin_acados_free_capsule(capsule);
        return false;
    }
    impl_->phase_rejoin.reset(capsule);
    impl_->nx = SPMPC_PHASE_REJOIN_NX;
    impl_->nu = SPMPC_PHASE_REJOIN_NU;
    impl_->np = SPMPC_PHASE_REJOIN_NP;
    impl_->horizon = SPMPC_PHASE_REJOIN_N;
    return true;
#else
    return false;
#endif
}

bool GeneratedAcadosSolver::ready() const {
    return impl_->ready();
}

GeneratedAcadosSolver::Kind GeneratedAcadosSolver::kind() const {
    return impl_->kind;
}

int GeneratedAcadosSolver::stateWidth() const { return impl_->nx; }
int GeneratedAcadosSolver::controlWidth() const { return impl_->nu; }
int GeneratedAcadosSolver::parameterWidth() const { return impl_->np; }
int GeneratedAcadosSolver::horizonSteps() const { return impl_->horizon; }

bool GeneratedAcadosSolver::updateParameters(
    int stage, const double* parameters) {
    if (!ready() || parameters == nullptr || stage < 0 ||
        stage > impl_->horizon) {
        return false;
    }
    double* mutable_parameters = const_cast<double*>(parameters);
    if (impl_->kind == Kind::B0) {
        return spmpc_b0_acados_update_params(
            impl_->b0.get(), stage, mutable_parameters, impl_->np) == 0;
    }
#ifdef SPMPC_WITH_ACADOS_SLOSH
    if (impl_->kind == Kind::SLOSH) {
        return spmpc_slosh_acados_update_params(
            impl_->slosh.get(), stage, mutable_parameters, impl_->np) == 0;
    }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
    if (impl_->kind == Kind::PHASE_REJOIN) {
        return spmpc_phase_rejoin_acados_update_params(
            impl_->phase_rejoin.get(), stage,
            mutable_parameters, impl_->np) == 0;
    }
#endif
    return false;
}

bool GeneratedAcadosSolver::setState(int stage, const double* state) {
    if (!ready() || state == nullptr || stage < 0 || stage > impl_->horizon) {
        return false;
    }
    ocp_nlp_out_set(impl_->config(), impl_->dims(), impl_->output(),
                    impl_->input(), stage, "x", const_cast<double*>(state));
    return true;
}

bool GeneratedAcadosSolver::setControl(int stage, const double* control) {
    if (!ready() || control == nullptr || stage < 0 ||
        stage >= impl_->horizon) {
        return false;
    }
    ocp_nlp_out_set(impl_->config(), impl_->dims(), impl_->output(),
                    impl_->input(), stage, "u", const_cast<double*>(control));
    return true;
}

bool GeneratedAcadosSolver::getState(int stage, double* state) const {
    if (!ready() || state == nullptr || stage < 0 || stage > impl_->horizon) {
        return false;
    }
    ocp_nlp_out_get(
        impl_->config(), impl_->dims(), impl_->output(), stage, "x", state);
    return true;
}

bool GeneratedAcadosSolver::getControl(int stage, double* control) const {
    if (!ready() || control == nullptr || stage < 0 ||
        stage >= impl_->horizon) {
        return false;
    }
    ocp_nlp_out_get(
        impl_->config(), impl_->dims(), impl_->output(), stage, "u", control);
    return true;
}

bool GeneratedAcadosSolver::setStateBounds(
    int stage, const double* lower, const double* upper) {
    if (!ready() || lower == nullptr || upper == nullptr || stage < 0 ||
        stage > impl_->horizon) {
        return false;
    }
    ocp_nlp_constraints_model_set(
        impl_->config(), impl_->dims(), impl_->input(), impl_->output(),
        stage, "lbx", const_cast<double*>(lower));
    ocp_nlp_constraints_model_set(
        impl_->config(), impl_->dims(), impl_->input(), impl_->output(),
        stage, "ubx", const_cast<double*>(upper));
    return true;
}

bool GeneratedAcadosSolver::setControlBounds(
    int stage, const double* lower, const double* upper) {
    if (!ready() || lower == nullptr || upper == nullptr || stage < 0 ||
        stage >= impl_->horizon) {
        return false;
    }
    ocp_nlp_constraints_model_set(
        impl_->config(), impl_->dims(), impl_->input(), impl_->output(),
        stage, "lbu", const_cast<double*>(lower));
    ocp_nlp_constraints_model_set(
        impl_->config(), impl_->dims(), impl_->input(), impl_->output(),
        stage, "ubu", const_cast<double*>(upper));
    return true;
}

bool GeneratedAcadosSolver::copyIterateFrom(
    const GeneratedAcadosSolver& source) {
    if (!ready() || !source.ready() ||
        impl_->horizon != source.impl_->horizon ||
        impl_->nx != source.impl_->nx || impl_->nu != source.impl_->nu ||
        impl_->np != source.impl_->np) {
        return false;
    }
    const char* fields[] = {"x", "u", "z", "sl", "su", "pi", "lam"};
    for (const char* field : fields) {
        const int dimension = ocp_nlp_dims_get_total_from_attr(
            source.impl_->config(), source.impl_->dims(),
            source.impl_->output(), field);
        if (dimension < 0) return false;
        if (dimension == 0) continue;
        std::vector<double> values(static_cast<std::size_t>(dimension), 0.0);
        ocp_nlp_get_all(
            source.impl_->solver(), source.impl_->input(),
            source.impl_->output(), field, values.data());
        ocp_nlp_set_all(
            impl_->solver(), impl_->input(), impl_->output(),
            field, values.data());
    }
    return true;
}

int GeneratedAcadosSolver::solve() {
    if (!ready()) return -1;
    if (impl_->kind == Kind::B0) {
        return spmpc_b0_acados_solve(impl_->b0.get());
    }
#ifdef SPMPC_WITH_ACADOS_SLOSH
    if (impl_->kind == Kind::SLOSH) {
        return spmpc_slosh_acados_solve(impl_->slosh.get());
    }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
    if (impl_->kind == Kind::PHASE_REJOIN) {
        return spmpc_phase_rejoin_acados_solve(impl_->phase_rejoin.get());
    }
#endif
    return -1;
}

double GeneratedAcadosSolver::solveTimeSec() const {
    if (!ready()) return 0.0;
    double time = 0.0;
    ocp_nlp_get(impl_->solver(), "time_tot", &time);
    return time;
}

}  // namespace spmpc_local_planner

#else

namespace spmpc_local_planner {

struct GeneratedAcadosSolver::Impl {
    Kind kind = Kind::B0;
};

GeneratedAcadosSolver::GeneratedAcadosSolver() : impl_(new Impl()) {}
GeneratedAcadosSolver::~GeneratedAcadosSolver() = default;
bool GeneratedAcadosSolver::create(Kind kind) {
    impl_->kind = kind;
    return false;
}
void GeneratedAcadosSolver::reset() {}
bool GeneratedAcadosSolver::ready() const { return false; }
GeneratedAcadosSolver::Kind GeneratedAcadosSolver::kind() const {
    return impl_->kind;
}
int GeneratedAcadosSolver::stateWidth() const { return 0; }
int GeneratedAcadosSolver::controlWidth() const { return 0; }
int GeneratedAcadosSolver::parameterWidth() const { return 0; }
int GeneratedAcadosSolver::horizonSteps() const { return 0; }
bool GeneratedAcadosSolver::updateParameters(int, const double*) { return false; }
bool GeneratedAcadosSolver::setState(int, const double*) { return false; }
bool GeneratedAcadosSolver::setControl(int, const double*) { return false; }
bool GeneratedAcadosSolver::getState(int, double*) const { return false; }
bool GeneratedAcadosSolver::getControl(int, double*) const { return false; }
bool GeneratedAcadosSolver::setStateBounds(
    int, const double*, const double*) { return false; }
bool GeneratedAcadosSolver::setControlBounds(
    int, const double*, const double*) { return false; }
bool GeneratedAcadosSolver::copyIterateFrom(
    const GeneratedAcadosSolver&) { return false; }
int GeneratedAcadosSolver::solve() { return -1; }
double GeneratedAcadosSolver::solveTimeSec() const { return 0.0; }

}  // namespace spmpc_local_planner

#endif
