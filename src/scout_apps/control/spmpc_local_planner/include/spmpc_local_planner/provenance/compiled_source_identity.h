#pragma once

namespace spmpc_local_planner {
namespace provenance {

const char* modelCompiledSourceHead();
const char* controllerCompiledSourceHead();
const char* phaseRejoinCompiledSourceHead();
const char* safetyCompiledSourceHead();
const char* referenceCompiledSourceHead();
const char* runtimeCompiledSourceHead();
const char* estimationCompiledSourceHead();

}  // namespace provenance
}  // namespace spmpc_local_planner
