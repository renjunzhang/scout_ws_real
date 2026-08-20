#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double eta_x;
    double eta_x_dot;
    double eta_y;
    double eta_y_dot;
} spmpc_modal_state_t;

typedef struct {
    double two_zeta_omega_n;
    double omega_n_sq;
    double kappa_x;
    double kappa_y;
} spmpc_modal_parameters_t;

typedef struct {
    uint64_t state_stamp_ns;
    uint64_t update_count;
    uint64_t reset_epoch;
    spmpc_modal_state_t state;
} spmpc_observer_anchor_t;

typedef struct {
    uint64_t state_stamp_ns;
    double sample_dt_sec;
    uint64_t update_count;
    uint64_t reset_epoch;
    double ax;
    double ay;
} spmpc_observer_input_t;

typedef struct {
    uint64_t state_stamp_ns;
    uint64_t update_count;
    uint64_t reset_epoch;
    spmpc_modal_state_t state;
    int has_input;
    double sample_dt_sec;
    double ax;
    double ay;
    int epoch_reset_applied;
} spmpc_observer_replay_point_t;

typedef struct {
    double a;
    double alpha;
    double duration_sec;
} spmpc_planned_control_t;

typedef struct {
    double time_sec;
    double v;
    double omega;
    spmpc_modal_state_t state;
    int control_index;
} spmpc_planned_replay_point_t;

int spmpc_modal_cubic_hermite(
    const spmpc_modal_state_t* left,
    const spmpc_modal_state_t* right,
    double tau_sec,
    double interval_sec,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity);

int spmpc_modal_sample_nodes(
    const spmpc_modal_state_t* nodes,
    size_t node_count,
    double node_dt_sec,
    double query_sec,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity);

int spmpc_modal_exact_zoh_step(
    const spmpc_modal_state_t* state,
    double ax,
    double ay,
    double dt_sec,
    const spmpc_modal_parameters_t* parameters,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity);

int spmpc_modal_replay_observer(
    const spmpc_observer_anchor_t* anchor,
    const spmpc_observer_input_t* samples,
    size_t sample_count,
    const spmpc_modal_parameters_t* parameters,
    double state_dt_tolerance_sec,
    int allow_epoch_reset,
    spmpc_observer_replay_point_t* output,
    size_t output_capacity,
    size_t* output_count,
    int* skipped_anchor_echo,
    size_t* epoch_reset_count,
    char* error,
    size_t error_capacity);

int spmpc_modal_sample_observer(
    const spmpc_observer_replay_point_t* points,
    size_t point_count,
    uint64_t query_stamp_ns,
    const spmpc_modal_parameters_t* parameters,
    spmpc_modal_state_t* output,
    char* error,
    size_t error_capacity);

int spmpc_modal_replay_planned(
    const spmpc_modal_state_t* initial_state,
    double initial_v,
    double initial_omega,
    const spmpc_planned_control_t* controls,
    size_t control_count,
    const spmpc_modal_parameters_t* parameters,
    double max_substep_sec,
    spmpc_planned_replay_point_t* output,
    size_t output_capacity,
    size_t* output_count,
    char* error,
    size_t error_capacity);

int spmpc_modal_sample_planned(
    const spmpc_planned_replay_point_t* points,
    size_t point_count,
    double query_sec,
    spmpc_planned_replay_point_t* output,
    char* error,
    size_t error_capacity);

#ifdef __cplusplus
}
#endif
