# Current SPMPC slosh model figures

Generated from the current SPMPC liquid model configuration:

- `src/scout_apps/control/spmpc_local_planner/config/containers/tube_default.yaml`
- `src/scout_apps/control/slosh_models/src/liquid_slosh_model.cpp`
- `src/scout_apps/control/spmpc_local_planner/scripts/acados/spmpc_acados_model.py`

## Parameters

| item | value |
|---|---:|
| container radius R | 0.018500 m = 18.5 mm |
| liquid height h | 0.058000 m = 58.0 mm |
| liquid density rho | 1000.0 kg/m^3 |
| mode index | 1 |
| modal root xi_11 | 1.8412 |
| damping ratio zeta | 0.0500 |
| natural frequency omega_n | 31.246035 rad/s |
| natural frequency f_n | 4.972961 Hz |
| natural period T_n | 0.201087 s |
| two_zeta_omega_n | 3.124604 1/s |
| omega_n_sq | 976.314708 1/s^2 |
| total liquid mass m_F | 0.062362185 kg |
| modal mass m_n | 0.009040337 kg |
| modal stiffness k_n | 8.826213654 N/m |
| modal damping c_n | 0.028247468 N*s/m |
| linear height coefficient c_h | 1.817939967 |
| static modal displacement gain | 0.001024260 m/(m/s^2) |
| static signed height gain | 1.862043 mm/(m/s^2) |
| slosh_height_ref | 0.005000 m |
| eta_ref = slosh_height_ref / c_h | 0.002750366 m |
| eta_dot_ref = omega_n * slosh_height_ref | 0.156230175 m/s |
| parabola coefficient R^2/(4g) | 0.000008721967 m/(rad/s)^2 |

## Linear transfer function used for Bode/Nyquist

For each axis, the current model is:

```text
eta_ddot_i + 2*zeta*omega_n*eta_dot_i + omega_n^2*eta_i = -kappa_i*a_i
kappa_x = kappa_y = 1
```

The Bode and Nyquist plots use the signed linear height component:

```text
H_h(s) = h_i(s) / a_i(s) = -c_h / (s^2 + 2*zeta*omega_n*s + omega_n^2)
```

The published height proxy also adds the nonlinear rotation parabola term:

```text
h_slosh = c_h * sqrt(eta_x^2 + eta_y^2) + R^2 * omega_z^2 / (4*g)
```

That nonlinear parabola term is shown in the block diagram, but is not included in the Bode/Nyquist transfer function.

## Files

- `00_combined_current_slosh_model_overview.svg/png`
- `01_block_diagram_current_slosh_model.svg/png`
- `02_time_response_unit_accel_step.svg/png`
- `03_bode_current_slosh_model.svg/png`
- `04_nyquist_current_slosh_model.svg/png`
