"""OSCRS 内部类型定义。

当前阶段 (L2.5 模块化) 以 dataclass 作为接口文档和签名约束；
内部数据流暂保持 dict 形态，以保证 candidate_report 字段兼容。
后续阶段可从 dict 迁移到这些 dataclass。
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class FeasibilityResult:
    """F 层对单条候选的输出。"""
    accepted: bool
    reject_reason: str  # "accepted" 或 "|".join(reasons)
    # geometry gate 中间量
    length_ratio: float
    kappa_ratio: float
    dkappa_ratio: float
    endpoint_error_m: float
    predicted_ay_p95: float
    predicted_ay_ratio: float
    predicted_vmax: float
    base_predicted_ay_p95: float
    base_predicted_vmax: float
    target_kappa_penalty: float
    geometry_score: float
    # collision 子结果
    collision_status: str
    collision_idx: int
    collision_cost: int
    reject_stage: str = "ACCEPTED"
    tail_protect_applied: bool = False
    tail_gate_enabled: bool = False
    tail_deviation_m: float = 0.0
    tail_heading_error_deg: float = 0.0


@dataclass
class SloshMetrics:
    """R 层对单条候选的输出 — 物理晃动指标。"""
    slosh_h_p95: float
    slosh_h_max: float
    slosh_h_residual_max: float
    slosh_h_modal_p95: float
    slosh_h_parabola_p95: float
    slosh_eta_x_p95: float
    slosh_eta_y_p95: float
    slosh_eta_dot_rms: float
    slosh_energy_rms: float
    slosh_terminal_E: float


@dataclass
class OSCRSParams:
    """S 层 hard gate 与 score 的参数集。"""
    eta_lim: float  # m
    residual_ratio: float
    settle_duration: float
    use_legacy_score: bool
    score_batch_norm: bool
    score_w_h_p95: float
    score_w_energy_rms: float
    score_w_eta_dot_rms: float
    score_w_terminal_E: float
    score_w_geom: float


@dataclass
class CandidateEval:
    """单条候选的完整评估结果（G→F→R→S 全链路）。

    当前阶段作为文档类型；内部数据流仍使用 dict，
    字段名与 publish_candidate_report() 输出 key 严格一致。
    """
    index: int
    name: str
    accepted: bool
    score: float
    geometry_score: float
    slosh_score: float
    length_ratio: float
    kappa_ratio: float
    dkappa_ratio: float
    predicted_ay_p95: float
    predicted_ay_ratio: float
    predicted_ax_p95: float
    predicted_ax_peak: float
    predicted_ay_peak: float
    predicted_vmax: float
    base_predicted_ay_p95: float
    base_predicted_vmax: float
    target_kappa_penalty: float
    endpoint_error_m: float
    collision_status: str
    collision_idx: int
    collision_cost: int
    reject_stage: str
    reject_reason: str
    tail_protect_applied: bool = False
    tail_gate_enabled: bool = False
    tail_deviation_m: float = 0.0
    tail_heading_error_deg: float = 0.0
    oscrs_feasible: bool = False
    oscrs_height_pass: bool = False
    oscrs_residual_pass: bool = False
    oscrs_violation: float = 0.0
    oscrs_score: float = 0.0
    oscrs_eta_lim: float = 0.0
    oscrs_residual_limit: float = 0.0
    slosh_h_p95: float = 0.0
    slosh_h_max: float = 0.0
    slosh_h_residual_max: float = 0.0
    slosh_h_modal_p95: float = 0.0
    slosh_h_parabola_p95: float = 0.0
    slosh_eta_x_p95: float = 0.0
    slosh_eta_y_p95: float = 0.0
    slosh_eta_dot_rms: float = 0.0
    slosh_energy_rms: float = 0.0
    slosh_terminal_E: float = 0.0
    # path_metrics 字段
    length_m: float = 0.0
    max_drift_m: float = 0.0
    min_seg_m: float = 0.0
    kappa_p95: float = 0.0
    kappa_max: float = 0.0
    dkappa_p95: float = 0.0
    dkappa_max: float = 0.0

    @classmethod
    def from_dict(cls, d: dict, index: int = 0) -> "CandidateEval":
        return cls(
            index=d.get("index", index),
            name=d.get("name", ""),
            accepted=d.get("accepted", False),
            score=d.get("score", 0.0),
            geometry_score=d.get("geometry_score", 0.0),
            slosh_score=d.get("slosh_score", 0.0),
            length_ratio=d.get("length_ratio", 0.0),
            kappa_ratio=d.get("kappa_ratio", 0.0),
            dkappa_ratio=d.get("dkappa_ratio", 0.0),
            predicted_ay_p95=d.get("predicted_ay_p95", 0.0),
            predicted_ay_ratio=d.get("predicted_ay_ratio", 0.0),
            predicted_ax_p95=d.get("predicted_ax_p95", 0.0),
            predicted_ax_peak=d.get("predicted_ax_peak", 0.0),
            predicted_ay_peak=d.get("predicted_ay_peak", 0.0),
            predicted_vmax=d.get("predicted_vmax", 0.0),
            base_predicted_ay_p95=d.get("base_predicted_ay_p95", 0.0),
            base_predicted_vmax=d.get("base_predicted_vmax", 0.0),
            target_kappa_penalty=d.get("target_kappa_penalty", 0.0),
            endpoint_error_m=d.get("endpoint_error_m", 0.0),
            tail_protect_applied=d.get("tail_protect_applied", False),
            tail_gate_enabled=d.get("tail_gate_enabled", False),
            tail_deviation_m=d.get("tail_deviation_m", 0.0),
            tail_heading_error_deg=d.get("tail_heading_error_deg", 0.0),
            collision_status=d.get("collision_status", ""),
            collision_idx=d.get("collision_idx", -1),
            collision_cost=d.get("collision_cost", -1),
            reject_stage=d.get("reject_stage", "ACCEPTED"),
            reject_reason=d.get("reject_reason", ""),
            oscrs_feasible=d.get("oscrs_feasible", False),
            oscrs_height_pass=d.get("oscrs_height_pass", False),
            oscrs_residual_pass=d.get("oscrs_residual_pass", False),
            oscrs_violation=d.get("oscrs_violation", 0.0),
            oscrs_score=d.get("oscrs_score", 0.0),
            oscrs_eta_lim=d.get("oscrs_eta_lim", 0.0),
            oscrs_residual_limit=d.get("oscrs_residual_limit", 0.0),
            slosh_h_p95=d.get("slosh_h_p95", 0.0),
            slosh_h_max=d.get("slosh_h_max", 0.0),
            slosh_h_residual_max=d.get("slosh_h_residual_max", 0.0),
            slosh_h_modal_p95=d.get("slosh_h_modal_p95", 0.0),
            slosh_h_parabola_p95=d.get("slosh_h_parabola_p95", 0.0),
            slosh_eta_x_p95=d.get("slosh_eta_x_p95", 0.0),
            slosh_eta_y_p95=d.get("slosh_eta_y_p95", 0.0),
            slosh_eta_dot_rms=d.get("slosh_eta_dot_rms", 0.0),
            slosh_energy_rms=d.get("slosh_energy_rms", 0.0),
            slosh_terminal_E=d.get("slosh_terminal_E", 0.0),
            length_m=d.get("length_m", 0.0),
            max_drift_m=d.get("max_drift_m", 0.0),
            min_seg_m=d.get("min_seg_m", 0.0),
            kappa_p95=d.get("kappa_p95", 0.0),
            kappa_max=d.get("kappa_max", 0.0),
            dkappa_p95=d.get("dkappa_p95", 0.0),
            dkappa_max=d.get("dkappa_max", 0.0),
        )


@dataclass
class SelectionResult:
    """S 层选择输出。"""
    best_row: dict
    best_points: list
    geometry_best_row: dict
    geometry_best_points: list
    oscrs_best_row: dict | None
    oscrs_best_points: list | None
    # fb / takeover 语义码
    fb: int = -1       # -1 未 active; 0 OSCRS selected safe candidate; 1 only original safe; 2 几何可行 slosh 全失败; 3 无可用几何
    takeover: int = 0   # 1 if OSCRS active publishes a non-original safe candidate


@dataclass
class AntiSloshParams:
    """完整参数集，供各层函数注入。避免各层依赖 self/rospy。"""
    # G
    candidate_specs: list = field(default_factory=list)
    candidate_levels: dict = field(default_factory=dict)
    generation_policy: dict = field(default_factory=dict)
    max_candidate_level: str = "medium"
    # F
    min_segment_length: float = 0.02
    max_drift: float = 0.18
    max_length_ratio: float = 1.15
    min_length_ratio: float = 0.995
    min_kappa_ratio: float = 0.20
    target_kappa_ratio: float = 0.35
    max_endpoint_error: float = 0.05
    ay_ratio_limit: float = 1.0
    enable_collision_check: bool = False
    # R
    predict_v_max: float = 2.0
    predict_ay_max: float = 2.0
    predict_a_max: float = 1.0
    predict_v_init: float = 0.0
    slosh_omega_n: float = 31.25
    slosh_zeta: float = 0.05
    slosh_rollout_dt: float = 0.05
    slosh_v_floor: float = 0.05
    slosh_height_coeff: float = 1.0
    slosh_container_radius: float = 0.0185
    slosh_offset_x: float = 0.0
    slosh_offset_y: float = 0.0
    slosh_use_parabola: bool = True
    # Score weights (legacy slosh)
    w_slosh_h: float = 0.0
    w_slosh_energy: float = 1.0
    w_slosh_eta_dot: float = 0.5
    w_slosh_terminal: float = 0.2
    w_slosh_kappa: float = 1.0
    w_slosh_dkappa: float = 0.5
    w_slosh_ay: float = 0.0
    w_slosh_length: float = 0.3
    w_slosh_drift: float = 0.5
    # Score weights (geometry)
    w_kappa: float = 1.0
    w_dkappa: float = 0.5
    w_length: float = 0.3
    w_drift: float = 0.5
    w_shortening: float = 10.0
    w_over_smooth: float = 2.0
    # S
    oscrs_shadow_enable: bool = False
    oscrs_active_enable: bool = False
    oscrs_eta_lim: float = 0.025
    oscrs_residual_ratio: float = 0.2
    oscrs_settle_duration: float = 2.0
    oscrs_use_legacy_score: bool = False
    oscrs_score_batch_norm: bool = True
    oscrs_score_w_h_p95: float = 1.0
    oscrs_score_w_energy: float = 0.3
    oscrs_score_w_eta_dot: float = 0.3
    oscrs_score_w_terminal: float = 0.2
    oscrs_score_w_geom: float = 0.2
    # Fixed candidate
    fixed_candidate_name: str = ""
    slosh_score_enable: bool = False
    ds: float = 0.10
    tail_protect_enable: bool = False
    tail_gate_enable: bool = False
    tail_protect_distance: float = 0.6
    tail_protect_mode: str = "replace_raw_tail"
    tail_deviation_limit: float = 0.05
    terminal_tail_heading_limit_deg: float = 10.0
