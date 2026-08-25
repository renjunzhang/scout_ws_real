#include "spmpc_local_planner/provenance/compiled_source_identity.h"

#ifndef SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD
#error "evidence dependency target is not bound to a Git source HEAD"
#endif

namespace spmpc_local_planner {
namespace provenance {

#if defined(SPMPC_SOURCE_IDENTITY_MODEL)
const char* modelCompiledSourceHead() {
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
}
#elif defined(SPMPC_SOURCE_IDENTITY_CONTROLLER)
const char* controllerCompiledSourceHead() {
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
}
#elif defined(SPMPC_SOURCE_IDENTITY_PHASE_REJOIN)
const char* phaseRejoinCompiledSourceHead() {
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
}
#elif defined(SPMPC_SOURCE_IDENTITY_SAFETY)
const char* safetyCompiledSourceHead() {
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
}
#elif defined(SPMPC_SOURCE_IDENTITY_REFERENCE)
const char* referenceCompiledSourceHead() {
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
}
#elif defined(SPMPC_SOURCE_IDENTITY_RUNTIME)
const char* runtimeCompiledSourceHead() {
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
}
#elif defined(SPMPC_SOURCE_IDENTITY_ESTIMATION)
const char* estimationCompiledSourceHead() {
    return SPMPC_BT_RESIDUAL_BUILD_SOURCE_HEAD;
}
#else
#error "evidence dependency target has no source-identity role"
#endif

}  // namespace provenance
}  // namespace spmpc_local_planner
