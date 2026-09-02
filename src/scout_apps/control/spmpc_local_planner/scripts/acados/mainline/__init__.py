"""Mainline acados contract package.

Submodules are deliberately imported explicitly by their consumers.  Keeping
package initialization empty prevents an independent contract such as the
Stage 3-D development capacity from acquiring Stage 0/1 or legacy scaffold
dependencies merely because Python initialized its parent package.
"""

__all__ = []
