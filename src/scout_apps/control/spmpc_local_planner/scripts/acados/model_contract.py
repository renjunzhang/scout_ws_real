"""Version/integration/codegen contract shared by the model and generators."""

MODEL_VERSION = 1
RK4_SUBSTEPS = 4
MAX_RK4_STEP_SEC = 1.0 / (30 * RK4_SUBSTEPS)
CASADI_CODEGEN_VERSION = "3.7.2"


def require_codegen_version(version):
    if version != CASADI_CODEGEN_VERSION:
        raise RuntimeError(
            f"reproducible codegen requires CasADi {CASADI_CODEGEN_VERSION}; "
            "install scripts/acados/requirements.txt")
