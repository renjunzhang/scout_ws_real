#pragma once

#include <memory>

namespace spmpc_local_planner {

// Typed, ROS-free owner for one generated acados capsule. The public contract
// intentionally exposes numerical operations rather than acados C handles.
class GeneratedAcadosSolver {
public:
    enum class Kind { B0, SLOSH, PHASE_REJOIN };

    GeneratedAcadosSolver();
    ~GeneratedAcadosSolver();
    GeneratedAcadosSolver(const GeneratedAcadosSolver&) = delete;
    GeneratedAcadosSolver& operator=(const GeneratedAcadosSolver&) = delete;

    bool create(Kind kind);
    void reset();
    bool ready() const;
    Kind kind() const;
    int stateWidth() const;
    int controlWidth() const;
    int parameterWidth() const;
    int horizonSteps() const;

    bool updateParameters(int stage, const double* parameters);
    bool setState(int stage, const double* state);
    bool setControl(int stage, const double* control);
    bool getState(int stage, double* state) const;
    bool getControl(int stage, double* control) const;
    bool setStateBounds(int stage, const double* lower, const double* upper);
    bool setControlBounds(int stage, const double* lower, const double* upper);
    bool copyIterateFrom(const GeneratedAcadosSolver& source);
    int solve();
    double solveTimeSec() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spmpc_local_planner
