#pragma once

// Compatibility facade for downstream code that historically consumed both
// request and result types from one header.  Production modules use the narrow
// authoritative headers directly.
#include "spmpc_local_planner/solver/api/solver_input.h"
#include "spmpc_local_planner/solver/api/solver_output.h"
