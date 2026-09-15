#!/usr/bin/env python3
"""Generate runtime cost components from the exact symbolic OCP expressions."""
import os
from pathlib import Path
import casadi as ca
from generate_spmpc_acados import load_config, default_parameter_values
from spmpc_acados_model import export_spmpc_b0_symbols, export_spmpc_slosh_symbols, PARAM_NAMES, PARAM_NAMES_SLOSH
from spmpc_acados_cost import cost_components, COST_COMPONENT_NAMES
from model_contract import COST_VERSION, require_codegen_version


def generate(destination, cfg=None):
    require_codegen_version(ca.__version__)
    cfg = load_config() if cfg is None else cfg
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    previous = Path.cwd()
    try:
        os.chdir(destination)
        gen = ca.CodeGenerator('ocp_cost_generated.c', {'with_header': True})
        for name, export in [('b0', export_spmpc_b0_symbols), ('slosh', export_spmpc_slosh_symbols)]:
            sym = export()
            for terminal in [False, True]:
                gen.add(ca.Function('spmpc_' + name + ('_terminal_terms' if terminal else '_stage_terms'),
                        [sym['x'], sym['u'], sym['p']], [ca.densify(cost_components(sym, cfg, terminal))]))
        gen.generate()
        for name in ['ocp_cost_generated.c', 'ocp_cost_generated.h']:
            f = Path(name)
            f.write_text('\n'.join(line.rstrip() for line in f.read_text().splitlines())+'\n')
        Path('ocp_cost_contract.h').write_text(
            '/* Generated; do not edit. */\n#pragma once\n' +
            f'#define SPMPC_COST_VERSION {COST_VERSION}\n' +
            f'#define SPMPC_COST_COMPONENTS {len(COST_COMPONENT_NAMES)}\n' +
            f'#define SPMPC_COST_N {cfg["N"]}\n')
        # C++ consumes this generated schema instead of a hand-maintained copy.
        default_values = default_parameter_values(cfg, True)
        lines = ['/* Generated from Python model/defaults; do not edit. */', '#pragma once',
                 '#include <array>', '#include <string>', '#include <vector>',
                 'namespace spmpc_local_planner { namespace ocp_parameters {', 'enum Param {']
        lines += [f'    {name.upper()} = {i},' for i, name in enumerate(PARAM_NAMES_SLOSH)]
        lines += [f'    PARAM_MAX = {len(PARAM_NAMES_SLOSH)}', '};',
                  f'constexpr int kB0ParameterCount = {len(PARAM_NAMES)};',
                  'inline std::array<double, PARAM_MAX> defaults() { return {{' +
                  ', '.join(format(float(v), '.17g') for v in default_values) + '}}; }',
                  'inline std::vector<std::string> names(int width) {',
                  '    static const std::vector<std::string> all = {' +
                  ', '.join('"'+n+'"' for n in PARAM_NAMES_SLOSH) + '};',
                  '    if (width != kB0ParameterCount && width != PARAM_MAX) return {};',
                  '    return {all.begin(), all.begin()+width};', '}', '}}']
        Path('ocp_parameter_contract.h').write_text('\n'.join(lines)+'\n')
    finally:
        os.chdir(previous)

if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[2] / 'src/core/generated')
