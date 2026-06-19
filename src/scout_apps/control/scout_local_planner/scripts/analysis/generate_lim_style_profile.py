#!/usr/bin/env python3
"""Compatibility wrapper for the Lim-style profile baseline generator.

The implementation was moved to scout_profile_baselines so supplementary
comparison baselines are isolated from the legacy controller package. This file
keeps the old rosrun entrypoint working during the transition.
"""

import os
import runpy
import sys
from pathlib import Path


TARGET_REL = Path("scout_profile_baselines/scripts/generate_lim_style_profile.py")


def _candidate_scripts():
    here = Path(__file__).resolve()
    # Source-tree layout: .../control/scout_local_planner/scripts/analysis/wrapper.py
    if len(here.parents) >= 4:
        yield here.parents[3] / TARGET_REL

    for root in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if root:
            yield Path(root) / TARGET_REL

    try:
        import rospkg  # type: ignore

        yield Path(rospkg.RosPack().get_path("scout_profile_baselines")) / "scripts" / TARGET_REL.name
    except Exception:
        return


def main():
    for script in _candidate_scripts():
        if script.is_file():
            sys.argv[0] = str(script)
            runpy.run_path(str(script), run_name="__main__")
            return
    raise RuntimeError(
        "Cannot find scout_profile_baselines/scripts/generate_lim_style_profile.py. "
        "Build/source the scout_profile_baselines package and use "
        "`rosrun scout_profile_baselines generate_lim_style_profile.py` for the new entrypoint."
    )


if __name__ == "__main__":
    main()
