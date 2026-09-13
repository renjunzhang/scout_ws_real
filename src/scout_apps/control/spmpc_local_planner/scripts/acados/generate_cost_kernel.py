#!/usr/bin/env python3
"""Generate runtime cost components from the exact symbolic OCP expressions."""
import os
from pathlib import Path
import casadi as ca
from generate_spmpc_acados import load_config
from spmpc_acados_model import export_spmpc_b0_symbols, export_spmpc_slosh_symbols
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
    finally:
        os.chdir(previous)

if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[2] / 'src/core/generated')
