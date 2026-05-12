"""G 层 — GeoRef smoothing 候选生成器 (当前唯一 G 实例)。

包装 candidate_generators.generate_georef_candidates()，
附带 level 元数据和 cap 信息供 F 层使用。
候选点列、顺序、数量与旧实现逐点等价。
"""

from reference_generation.candidate_generators import generate_georef_candidates


def generate_georef_candidates_with_meta(base, candidate_specs, min_segment_length,
                                          sanitize_fn, candidate_levels, max_candidate_level):
    """对 base 路径生成 GeoRef smoothing 候选集合（行为与旧实现等价）。

    Returns:
        (candidates, meta)
        candidates: list[(name, points)] 顺序严格一致。
        meta: dict with keys candidate_levels, max_candidate_level — 供 F 层 level cap gate 使用。
    """
    candidates = generate_georef_candidates(
        base, candidate_specs, min_segment_length, sanitize_fn,
    )
    meta = {
        "candidate_levels": candidate_levels,
        "max_candidate_level": max_candidate_level,
    }
    return candidates, meta
