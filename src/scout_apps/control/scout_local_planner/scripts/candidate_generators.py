"""候选生成器 G 的运行时实例。

OSCRS 在方法层定义为 G + S 两层结构（见 RA-L 投稿方案 §3）：
    G : 候选生成器，当前实例 = GeoRef smoothing
    S : slosh-constrained 候选选择器（OSCRS proper，留在 post_processor）

本模块只承担 G。S 与 hard gate / score / fallback 仍在 post_processor 内。

设计依据：
    docs/Claude/修改方案-时间-简介/2026-05-09_OSCRS候选生成层抽取方案.md
"""

from generate_anti_slosh_path_candidates import smooth_path


def generate_georef_candidates(base, candidate_specs, min_segment_length, sanitize_fn):
    """对 base 路径生成 GeoRef smoothing 候选集合。

    Args:
        base: list[(x, y)] 已 resample 过的基础路径。
        candidate_specs: list[(name, iters, gain, drift_limit)]，
            与 post_processor.candidate_specs 同构。name == "original" 时不平滑。
        min_segment_length: float, sanitize 阈值。
        sanitize_fn: callable(points, min_seg) -> points。复用 post_processor.sanitize_points
            通过参数注入，避免 candidate_generators -> post_processor 反向 import
            （post_processor 是 ROS 节点 module，反向 import 会触发副作用）。

    Returns:
        list[(name, points)]，顺序与 candidate_specs 严格一致。
        OSCRS 同分时取最先出现者，依赖此性质。
    """
    out = []
    for name, iters, gain, drift_limit in candidate_specs:
        if name == "original":
            pts = base
        else:
            pts = smooth_path(base, int(iters), float(gain), float(drift_limit))
            pts = sanitize_fn(pts, min_segment_length)
        out.append((name, pts))
    return out
