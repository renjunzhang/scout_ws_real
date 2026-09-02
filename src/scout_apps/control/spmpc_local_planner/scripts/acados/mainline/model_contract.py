"""Shared semantic identity for the actuator-aware mainline model.

The constants in this module name graph structure, not a generated solver
artifact or one trial's runtime parameter values.  Keeping them dependency
free lets numeric oracles, symbolic adapters, and future manifest generation
consume one authority without importing CasADi, acados, or ROS.
"""

MODEL_ID = "spmpc_actuator_slosh_discrete_v1"
DISCRETIZATION_SCHEMA = "zoh_fopdt_piecewise_midpoint_pose_rk4_slosh_v1"
COST_SCHEMA = "right_endpoint_fixed_liquid_weight_v1"

__all__ = ["COST_SCHEMA", "DISCRETIZATION_SCHEMA", "MODEL_ID"]
