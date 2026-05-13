"""G 层 — GeoRef smoothing 候选生成器 (当前唯一 G 实例)。

包装 candidate_generators.generate_georef_candidates()。

注意：max_candidate_level 是 G 层 policy，但当前 L2.5 阶段故意不删除被
policy 跳过的候选，而是生成 row 并标记为 generation_skipped。这样
candidate_report / validate 脚本能看到 "level:medium>mild" 这类诊断原因。
"""

from reference_generation.candidate_generators import generate_georef_candidates
from oscrs.generators.tail_protect import protect_tail


def generate_georef_candidates_with_meta(base, candidate_specs, min_segment_length,
                                          sanitize_fn, candidate_levels, max_candidate_level,
                                          tail_protect_enable=False,
                                          tail_protect_distance=0.6,
                                          tail_protect_mode="replace_raw_tail"):
    """对 base 路径生成 GeoRef smoothing 候选集合（行为与旧实现等价）。

    Returns:
        (candidates, meta)
        candidates: list[(name, points)] 顺序和数量与旧 GeoRef 实现严格一致。
        meta: dict with keys candidate_levels, max_candidate_level — 供 F 层 level gate 使用。
    """
    candidates = generate_georef_candidates(
        base, candidate_specs, min_segment_length, sanitize_fn,
    )
    if tail_protect_enable:
        protected = []
        for name, points in candidates:
            if name == "original":
                protected.append((name, points))
                continue
            pts = protect_tail(points, base, tail_protect_distance, tail_protect_mode)
            protected.append((name, sanitize_fn(pts, min_segment_length)))
        candidates = protected
    max_level = candidate_levels[max_candidate_level]
    generation_policy = {
        name: {
            "skipped": candidate_levels.get(name, 0) > max_level,
            "reason": f"level:{name}>{max_candidate_level}"
            if candidate_levels.get(name, 0) > max_level else "generated",
        }
        for name, _ in candidates
    }
    meta = {
        "candidate_levels": candidate_levels,
        "max_candidate_level": max_candidate_level,
        "generation_policy": generation_policy,
        "tail_protect_enable": bool(tail_protect_enable),
        "tail_protect_distance": float(tail_protect_distance),
        "tail_protect_mode": tail_protect_mode,
    }
    return candidates, meta
