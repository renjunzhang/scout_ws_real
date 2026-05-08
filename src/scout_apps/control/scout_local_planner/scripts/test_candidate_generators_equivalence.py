#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""候选生成层抽取的等价性测试（§6.1 函数级 numeric exact equality）。

本 refactor 是同段代码搬家（path_callback 内联循环 -> generate_georef_candidates）。
逐点 float 精确相等是天然性质，任一不等都说明 refactor 引入了不该有的差异。

依据：docs/Claude/修改方案-时间-简介/2026-05-09_OSCRS候选生成层抽取方案.md §6.1

用法：
    cd src/scout_apps/control/scout_local_planner/scripts
    python3 test_candidate_generators_equivalence.py
预期输出：OK
"""

from generate_anti_slosh_path_candidates import smooth_path
from anti_slosh_path_post_processor import sanitize_points
from candidate_generators import generate_georef_candidates


def main():
    base = [
        (0.0, 0.0), (0.5, 0.1), (1.0, 0.3), (1.6, 0.2),
        (2.2, -0.1), (2.8, -0.4), (3.4, -0.5), (4.0, -0.5),
    ]
    specs = [
        ("original", 0, 0.0, 0.0),
        ("mild",   18, 0.35, 0.08),
        ("medium", 40, 0.45, 0.12),
        ("mid",    24, 0.30, 0.07),
        ("strong", 56, 0.55, 0.18),
    ]
    min_seg = 0.08

    # 旧路径：复刻 path_callback 内联循环逻辑（refactor 前形态）。
    expected = []
    for name, iters, gain, drift_limit in specs:
        if name == "original":
            pts = base
        else:
            pts = smooth_path(base, int(iters), float(gain), float(drift_limit))
            pts = sanitize_points(pts, min_seg)
        expected.append((name, pts))

    actual = generate_georef_candidates(base, specs, min_seg, sanitize_points)

    assert len(actual) == len(expected), (
        f"candidate count mismatch: actual={len(actual)} expected={len(expected)}"
    )
    for (na, pa), (ne, pe) in zip(actual, expected):
        assert na == ne, f"name mismatch: {na} vs {ne}"
        assert len(pa) == len(pe), (
            f"{na}: point count mismatch: {len(pa)} vs {len(pe)}"
        )
        for i, ((xa, ya), (xe, ye)) in enumerate(zip(pa, pe)):
            assert xa == xe and ya == ye, (
                f"{na}[{i}]: numeric drift ({xa},{ya}) vs ({xe},{ye})"
            )

    print("OK")


if __name__ == "__main__":
    main()
