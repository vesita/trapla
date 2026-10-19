//! 校验层：**独立**实现的硬约束校验器。
//!
//! 「独立」的含义（硬要求）：
//!
//! - 判定逻辑不调用 `Robot::position_check` / `Robot::satisfy_*`（那是被测者自证）；
//! - 足的覆盖格与四角逐字重写（[`foot_cover`] / [`foot_corners`]），只复用
//!   `trapla-core` 的纯几何原语（`Dot` / `Line` / `FootShape` / `Terrain::plane_at` / `fit_plane_up`），
//!   并有测试钉住它与被复用原语在数值上一致（`covers_match_core_primitive`）；
//! - 约束上下界来自 [`Limits`]（默认值取自 `trapla-core::config` 的唯一事实源，可被 JSON 覆盖）。
//!
//! ## 输入只有 `(x, y)`，`rz` 是重建出来的
//!
//! 点位序列不含朝向，本模块按 [`RzConvention`] 从**同一只脚的相邻位移**重建：
//!
//! - [`RzConvention::Raw`]（默认）：`|p_new − p_old| >= active_distance` 时 `rz = 步向`，否则保持原值
//!   —— 即 `Robot::next` 的语义（`robot.rs:150-159`）。两足初始 `rz = 0`。
//! - [`RzConvention::Sticky`]：`rz` 永不更新（恒为初始值）。这是**敏感性对照**用的另一种约定：
//!   两种约定下的结论若不同，说明结论依赖重建假设，必须一起报告。
//!
//! 步长按「同一只脚的位移」算 —— **不是** `metrics.json` 里的 `stride` 字段（那里存的是
//! 相邻落足点的跨距 `|p[i+1] − p[i]|`，几何上不是步长，实测最大 51.2 也不违反 40 的步长约束）。

use std::collections::BTreeMap;
use std::f64::consts::SQRT_2;

use serde::{Deserialize, Serialize};
use trapla_core::config::{FootShape, RobotConfig, SEPARATION_EPS};
use trapla_core::geom::{Dot, Idx, Line};
use trapla_core::metrics::wrap_angle;
use trapla_core::terrain::Terrain;

/// `Robot::new` 里初始双足间距的权重（`0.6 * min + 0.4 * max`，`robot.rs:43`）。
///
/// 这两个数字在 core 里是字面量、不是常量，所以这里重抄一份用于**重建初始足位**；
/// 若有变动，`initial_feet_match_robot_new` 这条测试会红。
const INITIAL_SEPARATION_WEIGHT_MIN: f64 = 0.6;
const INITIAL_SEPARATION_WEIGHT_MAX: f64 = 0.4;

/// `Robot::next_committed` 的朝向平滑系数（`robot.rs::HEADING_SMOOTHING`，core 未导出）。
const HEADING_SMOOTHING: f64 = 0.5;

/// 转角判据的量化容差（弧度）：坐标由 6 位小数 CSV 给出时，角度误差量级 ~1e-8，
/// 取 1e-6 足够宽，同时远小于 75°（1.309 rad）。
const TURN_QUANTUM_MARGIN: f64 = 1e-6;

/// 校验用的足形（避免 `FootShape` 依赖 serde）。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ShapeLimits {
    pub length: f64,
    pub width: f64,
    pub scan_step: f64,
}

impl From<FootShape> for ShapeLimits {
    fn from(shape: FootShape) -> Self {
        Self {
            length: shape.length,
            width: shape.width,
            scan_step: shape.scan_step,
        }
    }
}

impl From<ShapeLimits> for FootShape {
    fn from(shape: ShapeLimits) -> Self {
        FootShape {
            length: shape.length,
            width: shape.width,
            scan_step: shape.scan_step,
        }
    }
}

/// 校验用的一组上下界。默认值来自 `trapla-core::config`，可被 JSON 覆盖。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    pub max_stride: f64,
    pub min_separation: f64,
    pub max_separation: f64,
    pub max_turn: f64,
    pub max_normal_angle: f64,
    pub active_distance: f64,
    pub separation_eps: f64,
    pub shape: ShapeLimits,
}

impl Default for Limits {
    fn default() -> Self {
        Self::from(RobotConfig::default())
    }
}

impl From<RobotConfig> for Limits {
    fn from(config: RobotConfig) -> Self {
        Self {
            max_stride: config.max_stride,
            min_separation: config.min_separation,
            max_separation: config.max_separation,
            max_turn: config.max_turn,
            max_normal_angle: config.max_normal_angle,
            active_distance: config.active_distance,
            separation_eps: SEPARATION_EPS,
            shape: ShapeLimits::from(config.shape),
        }
    }
}

impl Limits {
    /// 默认阈值（与 `trapla-core::config` 同源：`MAX_STRIDE_CM` / `MIN|MAX_SEPARATION_CM` /
    /// `MAX_TURN_RAD` / `MAX_NORMAL_ANGLE_RAD` / `SEPARATION_EPS` / `MAX_STRIDE_CM * ACTIVE_DISTANCE_RATIO`）。
    pub fn canonical() -> Self {
        Self::from(RobotConfig::default())
    }

    /// 转回 core 的规划器配置（「换约束」实验要能同时作用于被测者）。
    pub fn to_config(self) -> RobotConfig {
        RobotConfig {
            shape: FootShape::from(self.shape),
            max_stride: self.max_stride,
            min_separation: self.min_separation,
            max_separation: self.max_separation,
            max_turn: self.max_turn,
            max_normal_angle: self.max_normal_angle,
            active_distance: self.active_distance,
        }
    }

    /// 应用 JSON 覆盖；返回「键: 旧值 -> 新值」的人类可读清单。
    ///
    /// 未知键**报错**而不是忽略：静默忽略会让人以为约束换了、实际没换。
    pub fn apply_overrides(&mut self, value: &serde_json::Value) -> Result<Vec<String>, String> {
        let object = value
            .as_object()
            .ok_or_else(|| "约束覆盖必须是一个 JSON 对象".to_string())?;
        let mut applied = Vec::with_capacity(object.len());
        for (key, raw) in object {
            let target: &mut f64 = match key.as_str() {
                "max_stride" => &mut self.max_stride,
                "min_separation" => &mut self.min_separation,
                "max_separation" => &mut self.max_separation,
                "max_turn" => &mut self.max_turn,
                "max_normal_angle" => &mut self.max_normal_angle,
                "active_distance" => &mut self.active_distance,
                "separation_eps" => &mut self.separation_eps,
                "shape_length" => &mut self.shape.length,
                "shape_width" => &mut self.shape.width,
                "shape_scan_step" => &mut self.shape.scan_step,
                other => return Err(format!("未知的约束覆盖键 `{other}`")),
            };
            let number = raw
                .as_f64()
                .ok_or_else(|| format!("覆盖键 `{key}` 的值不是数值：{raw}"))?;
            let old = *target;
            *target = number;
            applied.push(format!("{key}: {old} -> {number}"));
        }
        Ok(applied)
    }
}

/// `rz` 的重建约定。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RzConvention {
    /// `Robot::next` 语义：位移 >= `active_distance` 时 `rz = 步向`，否则保持。
    Raw,
    /// 敏感性对照：`rz` 恒为初始值，永不更新。
    Sticky,
    /// 提交语义 `Robot::next_committed`：`rz` 按 `HEADING_SMOOTHING` 向步向插值
    /// （`robot.rs::HEADING_SMOOTHING`，core 未导出，这里镜像同一数值；一致性由
    /// `planner::tests::reconstructed_rz_matches_robot_final_state` 对着机器人真实末态钉住）。
    Smoothed,
}

impl RzConvention {
    pub fn name(self) -> &'static str {
        match self {
            Self::Raw => "raw",
            Self::Sticky => "sticky",
            Self::Smoothed => "smoothed",
        }
    }
}

/// 越界类别。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Violation {
    /// 同脚位移 >= `max_stride`（严格小于才算满足）。
    Stride,
    /// 转角不在开区间 `(-max_turn, max_turn)` 内。
    Turn,
    /// 双足间距不在闭区间 `[min − eps, max + eps]` 内。
    Spacing,
    /// 足覆盖格有落在图外的。
    FootOutOfGrid,
    /// 足平面法向角 >= `max_normal_angle`。
    NormalAngle,
    /// 覆盖点拟合不出平面（`CoreError::DegeneratePlane`）。
    DegeneratePlane,
    /// 坐标不是有限数。
    NonFinite,
}

impl Violation {
    pub fn name(self) -> &'static str {
        match self {
            Self::Stride => "stride",
            Self::Turn => "turn",
            Self::Spacing => "spacing",
            Self::FootOutOfGrid => "foot_out_of_grid",
            Self::NormalAngle => "normal_angle",
            Self::DegeneratePlane => "degenerate_plane",
            Self::NonFinite => "non_finite",
        }
    }
}

/// 单步的检查结果。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StepCheck {
    /// 点位序列里的下标。
    pub index: usize,
    /// 0 = 偶数下标那一只脚，1 = 奇数下标那一只脚。
    pub foot: usize,
    /// 同脚位移（步长）。
    pub stride: f64,
    pub stride_ok: bool,
    /// 转角（弧度，已 wrap 到 (-π, π]）。
    pub turn_rad: f64,
    pub turn_ok: bool,
    /// 双足间距（已夹紧 `>= 0`）。
    pub spacing: f64,
    pub spacing_ok: bool,
    /// 足平面法向角（弧度）；无地图或不可站立时为 `None`。
    pub normal_angle_rad: Option<f64>,
    pub standable: Option<bool>,
    pub strict_violations: Vec<Violation>,
    /// 只有**超出输入量化界**的越界才算在这里（用于退出码与门禁）。
    pub hard_violations: Vec<Violation>,
}

/// 校验选项。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValidateOptions {
    pub limits: Limits,
    pub rz: RzConvention,
    /// 点位序列里偶数下标是不是左脚（`Robot::new` 的 `feet[0]`）。
    pub parity0_is_left: bool,
    /// 输入坐标的量化步长（CSV 6 位小数 ⇒ 5e-7）；`0` = 严格判据。
    pub coord_quantum: f64,
}

impl ValidateOptions {
    /// 默认：阈值取自 core、`rz` 用**提交语义**（`next_committed`）、偶数下标为左脚、量化 5e-7。
    ///
    /// 默认取 `smoothed` 而不是 `raw`：`raw`（`Robot::next`）只是候选检查用的语义，提交路径用的是
    /// `next_committed`；实测只有 `smoothed` 能逐位复现机器人的真实末态（`raw` 最高差 0.76 rad）。
    /// `raw` / `sticky` 仍可用作敏感性对照。
    pub fn canonical() -> Self {
        Self {
            limits: Limits::canonical(),
            rz: RzConvention::Smoothed,
            parity0_is_left: true,
            coord_quantum: 5e-7,
        }
    }

    /// 由量化步长推出的线性距离容差：两点的每个坐标各带 `quantum` 误差，
    /// 距离误差上界 = `2 * quantum * sqrt(2)`（`tests/validate.rs` 有数值对照）。
    pub fn linear_margin(&self) -> f64 {
        2.0 * self.coord_quantum * SQRT_2
    }
}

/// 一次校验的完整报告。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationReport {
    pub point_count: usize,
    pub parity0_is_left: bool,
    pub rz_convention: RzConvention,
    pub coord_quantum: f64,
    pub linear_margin: f64,
    pub limits: Limits,
    /// 显式跳过的检查（没有地图时的可站立检查等）——跳过不写成「通过」。
    pub skipped: Vec<String>,
    pub strict_violation_steps: usize,
    pub hard_violation_steps: usize,
    pub strict_counts: BTreeMap<Violation, usize>,
    pub hard_counts: BTreeMap<Violation, usize>,
    /// 末端落足点到 goal 的距离（软指标，不参与门禁）。
    pub goal_error_foot: Option<f64>,
    /// 末端**双足中心**到 goal 的距离（对齐 `Robot::reach_target` 的判据面）。
    pub goal_error_center: Option<f64>,
    /// 末端的重建朝向 `[偶数下标脚, 奇数下标脚]`（用来对照被测者的真实末态）。
    pub final_rz: [f64; 2],
    pub steps: Vec<StepCheck>,
}

impl ValidationReport {
    /// 硬约束是否全部满足（量化界内的边界值不算违规）。
    pub fn passed(&self) -> bool {
        self.hard_violation_steps == 0
    }
}

/// 足的覆盖格 —— 逐字重写 `Foot::cover`（`foot.rs:43-66`）。
///
/// 返回按 `(x, y)` 升序去重后的格集。
pub fn foot_cover(position: Dot, rz: f64, shape: &ShapeLimits) -> Vec<Idx> {
    let half_length = shape.length / 2.0;
    let half_width = shape.width / 2.0;
    let (cos_rz, sin_rz) = (rz.cos(), rz.sin());
    let mut set = std::collections::BTreeSet::new();

    let mut l = -half_length;
    while l <= half_length {
        let mut w = -half_width;
        while w <= half_width {
            let world_l = position.x() + l * cos_rz - w * sin_rz;
            let world_w = position.y() + l * sin_rz + w * cos_rz;
            set.insert((world_l.round() as i32, world_w.round() as i32));
            w += shape.scan_step;
        }
        l += shape.scan_step;
    }

    set.into_iter().map(|(x, y)| Idx::new(x, y)).collect()
}

/// 足的四角 —— 逐字重写 `Foot::corners`（`foot.rs:75-101`），顺序一致。
pub fn foot_corners(position: Dot, rz: f64, shape: &ShapeLimits) -> [Dot; 4] {
    let half_length = shape.length / 2.0;
    let half_width = shape.width / 2.0;
    let l_cos = half_length * rz.cos();
    let l_sin = half_length * rz.sin();
    let w_cos = half_width * rz.cos();
    let w_sin = half_width * rz.sin();

    [
        Dot::new(position.x() + l_cos - w_sin, position.y() + l_sin + w_cos),
        Dot::new(position.x() + l_cos + w_sin, position.y() + l_sin - w_cos),
        Dot::new(position.x() - l_cos + w_sin, position.y() - l_sin - w_cos),
        Dot::new(position.x() - l_cos - w_sin, position.y() - l_sin + w_cos),
    ]
}

/// `Robot::new` 的初始两足位置：`start ± (0, sep/2)`，`rz = 0`。
///
/// 返回 `(left, right)`；`parity0_is_left` 决定偶数下标那一只脚是哪一个（见 [`ValidateOptions`]）。
pub fn initial_feet(start: Dot, limits: &Limits) -> [(Dot, f64); 2] {
    let sep = INITIAL_SEPARATION_WEIGHT_MIN * limits.min_separation
        + INITIAL_SEPARATION_WEIGHT_MAX * limits.max_separation;
    [
        (Dot::new(start.x(), start.y() - sep / 2.0), 0.0),
        (Dot::new(start.x(), start.y() + sep / 2.0), 0.0),
    ]
}

/// 重建某一脚走第 `i` 步之后的 `rz`。
fn rebuilt_rz(
    convention: RzConvention,
    previous_rz: f64,
    old_pos: Dot,
    new_pos: Dot,
    active_distance: f64,
) -> f64 {
    match convention {
        RzConvention::Raw => {
            let stride = old_pos.distance(new_pos);
            if stride >= active_distance {
                old_pos.angle(new_pos)
            } else {
                previous_rz
            }
        }
        RzConvention::Sticky => previous_rz,
        RzConvention::Smoothed => {
            let stride = old_pos.distance(new_pos);
            if stride >= active_distance {
                let step_direction = old_pos.angle(new_pos);
                previous_rz + wrap_angle(step_direction - previous_rz) * HEADING_SMOOTHING
            } else {
                previous_rz
            }
        }
    }
}

/// 初始两足状态：`(每只脚的 (位置, rz), 按脚序的 rz)`，脚序由 `parity0_is_left` 决定。
fn initial_state(start: Dot, options: &ValidateOptions) -> ([(Dot, f64); 2], [f64; 2]) {
    let [(a_pos, a_rz), (b_pos, b_rz)] = initial_feet(start, &options.limits);
    if options.parity0_is_left {
        ([(a_pos, a_rz), (b_pos, b_rz)], [a_rz, b_rz])
    } else {
        ([(b_pos, b_rz), (a_pos, a_rz)], [b_rz, a_rz])
    }
}

/// 逐点重建落足的**姿态序列**：返回与 `path` 等长的 `(位置, rz)`。
///
/// 第 `i` 项 = 点位 `path[i]` 落定后该脚的朝向。这是朝向推进规则的**唯一实现处**：
/// [`validate_path`] 也只从这个序列取 `rz`，所以可视化画出来的朝向与校验器判的朝向
/// 必然同源（同 [`initial_feet`]、同 [`rebuilt_rz`]、同一「同脚上一点」下标规则）。
///
/// 非有限坐标的那一步不更新朝向（与校验器一致，它在那一步直接跳过）。
pub fn foot_poses(path: &[Dot], start: Dot, options: &ValidateOptions) -> Vec<(Dot, f64)> {
    let (initial, mut rz) = initial_state(start, options);
    let mut poses = Vec::with_capacity(path.len());
    for (index, current) in path.iter().enumerate() {
        let foot = index % 2;
        if current.x().is_finite() && current.y().is_finite() {
            let old_pos = if index >= 2 {
                path[index - 2]
            } else {
                initial[foot].0
            };
            rz[foot] = rebuilt_rz(
                options.rz,
                rz[foot],
                old_pos,
                *current,
                options.limits.active_distance,
            );
        }
        poses.push((*current, rz[foot]));
    }
    poses
}

/// 校验一条点位序列。
///
/// `terrain` 为 `None` 时跳过可站立 / 法向角检查，并在 `skipped` 里显式登记。
pub fn validate_path(
    path: &[Dot],
    start: Dot,
    goal: Option<Dot>,
    terrain: Option<&Terrain>,
    options: &ValidateOptions,
) -> ValidationReport {
    let limits = options.limits;
    let mut skipped = Vec::new();
    if terrain.is_none() {
        skipped.push(
            "standability: 未提供 --map，足覆盖是否全在图内与法向角检查**未执行**（不是通过）"
                .to_string(),
        );
    }

    let (initial, mut rz) = initial_state(start, options);

    let mut steps = Vec::with_capacity(path.len());
    let mut strict_counts: BTreeMap<Violation, usize> = BTreeMap::new();
    let mut hard_counts: BTreeMap<Violation, usize> = BTreeMap::new();
    let linear_margin = options.linear_margin();
    // 朝向推进只走 `foot_poses` 这一个实现（可视化也用同一份）。
    let poses = foot_poses(path, start, options);

    for (index, current) in path.iter().enumerate() {
        let foot = index % 2;
        let support_foot = 1 - foot;
        let old_pos = if index >= 2 {
            path[index - 2]
        } else {
            initial[foot].0
        };
        let support_pos = if index >= 1 {
            path[index - 1]
        } else {
            initial[support_foot].0
        };
        let support_rz = rz[support_foot];

        let mut strict = Vec::new();
        let mut hard = Vec::new();

        if !current.x().is_finite() || !current.y().is_finite() {
            strict.push(Violation::NonFinite);
            hard.push(Violation::NonFinite);
            *strict_counts.entry(Violation::NonFinite).or_insert(0) += 1;
            *hard_counts.entry(Violation::NonFinite).or_insert(0) += 1;
            steps.push(StepCheck {
                index,
                foot,
                stride: f64::NAN,
                stride_ok: false,
                turn_rad: f64::NAN,
                turn_ok: false,
                spacing: f64::NAN,
                spacing_ok: false,
                normal_angle_rad: None,
                standable: None,
                strict_violations: strict,
                hard_violations: hard,
            });
            continue;
        }

        let stride = old_pos.distance(*current);
        let stride_ok = stride < limits.max_stride;
        if !stride_ok {
            strict.push(Violation::Stride);
            if stride >= limits.max_stride + linear_margin {
                hard.push(Violation::Stride);
            }
        }

        let step_direction = old_pos.angle(*current);
        let turn_rad = wrap_angle(step_direction - support_rz);
        let turn_ok = turn_rad > -limits.max_turn && turn_rad < limits.max_turn;
        if !turn_ok {
            strict.push(Violation::Turn);
            if turn_rad <= -limits.max_turn - TURN_QUANTUM_MARGIN
                || turn_rad >= limits.max_turn + TURN_QUANTUM_MARGIN
            {
                hard.push(Violation::Turn);
            }
        }

        let new_rz = poses[index].1;
        let corners = foot_corners(*current, new_rz, &limits.shape);
        let center_line = Line::from_point_angle(support_pos, support_rz);
        let min_dist = corners
            .iter()
            .map(|corner| center_line.dist_to_point(*corner))
            .fold(f64::INFINITY, f64::min);
        let spacing = (min_dist - limits.shape.width / 2.0).max(0.0);

        let lower = limits.min_separation - limits.separation_eps;
        let upper = limits.max_separation + limits.separation_eps;
        let spacing_ok = spacing >= lower && spacing <= upper;
        if !spacing_ok {
            strict.push(Violation::Spacing);
            if spacing < lower - linear_margin || spacing > upper + linear_margin {
                hard.push(Violation::Spacing);
            }
        }

        let (normal_angle_rad, standable) = match terrain {
            None => (None, None),
            Some(terrain) => {
                let cover = foot_cover(*current, new_rz, &limits.shape);
                let in_bounds = cover.iter().all(|idx| terrain.grid.in_bounds(*idx));
                if !in_bounds {
                    strict.push(Violation::FootOutOfGrid);
                    hard.push(Violation::FootOutOfGrid);
                    (None, Some(false))
                } else {
                    let points: Vec<Dot> = cover.iter().map(|idx| idx.as_dot()).collect();
                    match terrain.plane_at(&points) {
                        Ok(plane) => {
                            let angle = plane.normal_angle();
                            let ok = angle < limits.max_normal_angle;
                            if !ok {
                                strict.push(Violation::NormalAngle);
                                hard.push(Violation::NormalAngle);
                            }
                            (Some(angle), Some(ok))
                        }
                        Err(_) => {
                            strict.push(Violation::DegeneratePlane);
                            hard.push(Violation::DegeneratePlane);
                            (None, Some(false))
                        }
                    }
                }
            }
        };

        for violation in &strict {
            *strict_counts.entry(*violation).or_insert(0) += 1;
        }
        for violation in &hard {
            *hard_counts.entry(*violation).or_insert(0) += 1;
        }

        rz[foot] = new_rz;
        steps.push(StepCheck {
            index,
            foot,
            stride,
            stride_ok,
            turn_rad,
            turn_ok,
            spacing,
            spacing_ok,
            normal_angle_rad,
            standable,
            strict_violations: strict,
            hard_violations: hard,
        });
    }

    let (goal_error_foot, goal_error_center) = match goal {
        None => (None, None),
        Some(goal) => {
            let last = path.last();
            let foot_error = last.map(|dot| dot.distance(goal));
            let center_error = match path.len() {
                0 => None,
                1 => foot_error,
                n => {
                    let center = Dot::new(
                        (path[n - 1].x() + path[n - 2].x()) / 2.0,
                        (path[n - 1].y() + path[n - 2].y()) / 2.0,
                    );
                    Some(center.distance(goal))
                }
            };
            (foot_error, center_error)
        }
    };

    ValidationReport {
        point_count: path.len(),
        parity0_is_left: options.parity0_is_left,
        rz_convention: options.rz,
        coord_quantum: options.coord_quantum,
        linear_margin,
        limits,
        skipped,
        strict_violation_steps: steps
            .iter()
            .filter(|step| !step.strict_violations.is_empty())
            .count(),
        hard_violation_steps: steps
            .iter()
            .filter(|step| !step.hard_violations.is_empty())
            .count(),
        strict_counts,
        hard_counts,
        goal_error_foot,
        goal_error_center,
        final_rz: rz,
        steps,
    }
}

/// 敏感性探针：换一种约定后结论会不会翻转。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Probe {
    /// 换了哪一项假设。
    pub change: String,
    pub hard_violation_steps: usize,
    pub strict_violation_steps: usize,
    /// 主报告「通过 / 不通过」这个**结论**是否被这次换假设翻转。
    pub verdict_flips: bool,
}

/// 两次敏感性检查：`rz` 重建约定、奇偶脚映射。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensitivityReport {
    pub rz_convention_flip: Probe,
    pub parity_flip: Probe,
}

fn probe(change: String, primary: &ValidationReport, alternative: &ValidationReport) -> Probe {
    Probe {
        change,
        hard_violation_steps: alternative.hard_violation_steps,
        strict_violation_steps: alternative.strict_violation_steps,
        verdict_flips: primary.passed() != alternative.passed(),
    }
}

/// 换一套假设再校验一遍，报告结论是否翻转。
///
/// 输入只有 `(x, y)`，所以「脚朝向」与「哪只脚先动」都是重建出来的假设；
/// 结论若依赖这两条假设，必须一起报告，不能只报一种。
pub fn sensitivity(
    path: &[Dot],
    start: Dot,
    goal: Option<Dot>,
    terrain: Option<&Terrain>,
    options: &ValidateOptions,
) -> SensitivityReport {
    let primary = validate_path(path, start, goal, terrain, options);

    let mut other_rz = *options;
    other_rz.rz = match options.rz {
        RzConvention::Raw => RzConvention::Sticky,
        RzConvention::Sticky => RzConvention::Smoothed,
        RzConvention::Smoothed => RzConvention::Raw,
    };
    let rz_alternative = validate_path(path, start, goal, terrain, &other_rz);

    let mut flipped_parity = *options;
    flipped_parity.parity0_is_left = !options.parity0_is_left;
    let parity_alternative = validate_path(path, start, goal, terrain, &flipped_parity);

    SensitivityReport {
        rz_convention_flip: probe(
            format!("rz 约定 {} -> {}", options.rz.name(), other_rz.rz.name()),
            &primary,
            &rz_alternative,
        ),
        parity_flip: probe(
            format!(
                "偶数下标 {} -> {}",
                if options.parity0_is_left {
                    "左脚"
                } else {
                    "右脚"
                },
                if flipped_parity.parity0_is_left {
                    "左脚"
                } else {
                    "右脚"
                }
            ),
            &primary,
            &parity_alternative,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use trapla_core::foot::Foot;

    #[test]
    fn covers_match_core_primitive() {
        let shape = ShapeLimits::from(FootShape::biped());
        for (x, y, rz) in [
            (0.0, 0.0, 0.0),
            (12.5, -3.25, 0.7),
            (-7.5, 9.0, -1.9),
            (33.3, 44.4, 2.9),
        ] {
            let position = Dot::new(x, y);
            let core_foot = Foot::new(position, rz, FootShape::from(shape));
            assert_eq!(foot_cover(position, rz, &shape), core_foot.cover());
            assert_eq!(foot_corners(position, rz, &shape), core_foot.corners());
        }
    }

    #[test]
    fn initial_feet_match_robot_new() {
        use trapla_core::robot::Robot;
        let limits = Limits::canonical();
        let [(left, left_rz), (right, right_rz)] = initial_feet(Dot::new(3.0, 4.0), &limits);
        let robot = Robot::new(limits.to_config(), Dot::new(3.0, 4.0));
        assert_eq!(robot.feet()[0].position, left);
        assert_eq!(robot.feet()[1].position, right);
        assert_eq!(robot.feet()[0].rz, left_rz);
        assert_eq!(robot.feet()[1].rz, right_rz);
    }

    #[test]
    fn clean_step_passes_and_oversized_step_fails() {
        // 一条自造的干净路径：每步都远小于 40、间距与转角都靠约束算过。
        let limits = Limits::canonical();
        let start = Dot::new(0.0, 0.0);
        let options = ValidateOptions {
            limits,
            rz: RzConvention::Raw,
            parity0_is_left: true,
            coord_quantum: 0.0,
        };
        // 用同一套几何反推一条合法步：交替脚，沿 +x 方向走，双足间距 5.2。
        let clean: Vec<Dot> = (0..8)
            .map(|i| {
                let x = 10.0 * (i as f64 + 1.0);
                if i % 2 == 0 {
                    Dot::new(x, -2.6)
                } else {
                    Dot::new(x, 2.6)
                }
            })
            .collect();
        let report = validate_path(&clean, start, None, None, &options);
        assert!(
            report.passed(),
            "干净路径不该有硬越界: {:?}",
            report.hard_counts
        );

        // 负向对照：把第 4 步的同脚位移推到 60（> 40）。
        let mut broken = clean.clone();
        broken[4] = Dot::new(clean[4].x() + 30.0, clean[4].y());
        let report = validate_path(&broken, start, None, None, &options);
        assert!(!report.passed());
        assert!(
            report.steps[4]
                .strict_violations
                .contains(&Violation::Stride),
            "第 4 步应报步长越界: {:?}",
            report.steps[4]
        );
        assert!(
            report.steps[4].hard_violations.contains(&Violation::Stride),
            "第 4 步应报硬步长越界: {:?}",
            report.steps[4]
        );
    }

    #[test]
    fn tiny_overshoot_is_within_quantum_not_hard() {
        let limits = Limits::canonical();
        let options = ValidateOptions {
            limits,
            rz: RzConvention::Raw,
            parity0_is_left: true,
            coord_quantum: 5e-7,
        };
        let start = Dot::new(0.0, 0.0);
        let path = vec![
            Dot::new(limits.max_stride + 4e-7, -2.6),
            Dot::new(0.0, 2.6),
            Dot::new(limits.max_stride + 4e-7, -2.6),
        ];
        let report = validate_path(&path, start, None, None, &options);
        assert_eq!(report.strict_violation_steps, 1);
        assert!(report.passed(), "4e-7 的越界在量化界内");
        assert!(options.linear_margin() > 4e-7);
    }

    #[test]
    fn spacing_violation_is_detected() {
        let limits = Limits::canonical();
        let options = ValidateOptions {
            limits,
            rz: RzConvention::Raw,
            parity0_is_left: true,
            coord_quantum: 0.0,
        };
        let start = Dot::new(0.0, 0.0);
        // 两只脚都落在同一条 y 上：间距被压到 0 < min。
        let path = vec![Dot::new(20.0, 0.0), Dot::new(40.0, 0.0)];
        let report = validate_path(&path, start, None, None, &options);
        assert!(!report.passed());
        assert!(
            report
                .steps
                .iter()
                .any(|step| step.strict_violations.contains(&Violation::Spacing)),
            "应报出间距越界: {:?}",
            report.strict_counts
        );
    }

    /// 交叉核对：本地间距公式 vs 参考谓词 `Robot::satisfy_spacing`。
    ///
    /// 复现逐步状态（位置用序列、朝向用**已验证**的提交语义），把它装进 `Robot::set_feet`，
    /// 再用参考谓词判同一步：两边布尔值必须逐个一致。这一步不是为了「信参考实现」，
    /// 而是为了确认**本地公式没有算错** —— 校验器的判定仍然走本地公式，不调参考谓词。
    ///
    /// 同时统计：参考谓词按「候选语义（`Robot::next`）」判通过、而**实际执行姿态**
    /// （`next_committed` 的平滑朝向）落在区间外的步数 —— 这是软/硬约束漂移的量。
    #[test]
    fn spacing_formula_agrees_with_reference_predicate() {
        use crate::planner::{PlanRequest, Planner, RobotPlanner};
        use trapla_core::foot::Foot;
        use trapla_core::robot::{Robot, WhichFoot};

        let mut checked = 0usize;
        let mut mismatches = 0usize;
        let mut drift_steps = 0usize;
        let mut standable_checked = 0usize;
        let mut standable_mismatches = 0usize;
        let mut standable_drift = 0usize;

        for id in ["flat", "step", "bumpy", "corridor", "obstacles"] {
            let scenario = crate::scenario::builtin(128)
                .into_iter()
                .find(|s| s.id == id)
                .expect("内置场景");
            let terrain = scenario.terrain();
            let config = RobotConfig::default();
            let request = PlanRequest {
                terrain: &terrain,
                start: scenario.start_dot(),
                goal: scenario.goal_dot(),
                config,
                seed: 1,
            };
            let result = RobotPlanner.plan(&request);
            if result.path.is_empty() {
                continue;
            }

            let limits = Limits::from(config);
            let shape = limits.shape;
            let [(a_pos, a_rz), (b_pos, b_rz)] = initial_feet(scenario.start_dot(), &limits);
            let mut state = if result.parity0_is_left {
                [(a_pos, a_rz), (b_pos, b_rz)]
            } else {
                [(b_pos, b_rz), (a_pos, a_rz)]
            };

            for (index, current) in result.path.iter().enumerate() {
                let moving = index % 2;
                let support = 1 - moving;
                let old = if index >= 2 {
                    result.path[index - 2]
                } else {
                    state[moving].0
                };

                let mut robot = Robot::new(config, scenario.start_dot());
                robot.set_feet(
                    Foot::new(state[0].0, state[0].1, shape.into()),
                    Foot::new(state[1].0, state[1].1, shape.into()),
                    if moving == 0 {
                        WhichFoot::Left
                    } else {
                        WhichFoot::Right
                    },
                );
                let reference = robot.satisfy_spacing(*current);

                let spacing_with = |new_rz: f64| {
                    let corners = foot_corners(*current, new_rz, &shape);
                    let line = Line::from_point_angle(state[support].0, state[support].1);
                    let min_dist = corners
                        .iter()
                        .map(|corner| line.dist_to_point(*corner))
                        .fold(f64::INFINITY, f64::min);
                    (min_dist - shape.width / 2.0).max(0.0)
                };
                let lower = limits.min_separation - limits.separation_eps;
                let upper = limits.max_separation + limits.separation_eps;
                let in_range = |spacing: f64| spacing >= lower && spacing <= upper;

                // 候选语义（参考谓词内部用 `Robot::next` 算新脚朝向）
                let candidate_rz = rebuilt_rz(
                    RzConvention::Raw,
                    state[moving].1,
                    old,
                    *current,
                    limits.active_distance,
                );
                // 提交语义（实际执行）
                let committed_rz = rebuilt_rz(
                    RzConvention::Smoothed,
                    state[moving].1,
                    old,
                    *current,
                    limits.active_distance,
                );

                checked += 1;
                if in_range(spacing_with(candidate_rz)) != reference {
                    mismatches += 1;
                }
                if reference && !in_range(spacing_with(committed_rz)) {
                    drift_steps += 1;
                }

                // 可站立：本地 = 覆盖格全在图内 + 顶点拟合平面法向角 < 阈值。
                // 对照 `Foot::standable`（它调用的是同一套原语的参考组合）。
                let local_standable = |rz: f64| {
                    let cover = foot_cover(*current, rz, &shape);
                    if !cover.iter().all(|idx| terrain.grid.in_bounds(*idx)) {
                        return false;
                    }
                    let points: Vec<Dot> = cover.iter().map(|idx| idx.as_dot()).collect();
                    terrain
                        .plane_at(&points)
                        .is_ok_and(|plane| plane.normal_angle() < limits.max_normal_angle)
                };
                let reference_standable =
                    Foot::new(*current, candidate_rz, shape.into()).standable(&terrain);
                standable_checked += 1;
                if local_standable(candidate_rz) != reference_standable {
                    standable_mismatches += 1;
                }
                if reference_standable && !local_standable(committed_rz) {
                    standable_drift += 1;
                }

                state[moving] = (*current, committed_rz);
            }
        }

        assert!(checked >= 50, "样本太少：{checked}");
        assert_eq!(
            mismatches, 0,
            "本地间距公式与参考谓词不一致 {mismatches}/{checked} 步"
        );
        assert_eq!(
            standable_mismatches, 0,
            "本地可站立判定与 Foot::standable 不一致 {standable_mismatches}/{standable_checked} 步"
        );
        println!(
            "公式交叉核对：间距 {checked} 步全一致；可站立 {standable_checked} 步全一致；\
             候选姿态通过而执行姿态越界：间距 {drift_steps} 步 / 可站立 {standable_drift} 步"
        );
    }

    #[test]
    fn rz_conventions_are_all_reachable() {
        let mut options = ValidateOptions::canonical();
        let path = vec![
            Dot::new(20.0, -2.6),
            Dot::new(20.0, 2.6),
            Dot::new(40.0, -2.6),
            Dot::new(40.0, 2.6),
        ];
        let mut point_counts = Vec::new();
        for convention in [
            RzConvention::Raw,
            RzConvention::Sticky,
            RzConvention::Smoothed,
        ] {
            options.rz = convention;
            let report = validate_path(&path, Dot::new(0.0, 0.0), None, None, &options);
            assert_eq!(report.rz_convention, convention);
            point_counts.push(report.steps.len());
        }
        // 三种约定下报告的是同一批步数（约定只影响几何量，不影响步数）。
        assert_eq!(point_counts, vec![4, 4, 4]);
    }

    /// `foot_poses` 与 `validate_path` 的朝向重建必须同源：末态逐位相等。
    #[test]
    fn foot_poses_agree_with_validation_final_rz() {
        let path = vec![
            Dot::new(20.0, -2.6),
            Dot::new(20.0, 2.6),
            Dot::new(40.0, -2.6),
            Dot::new(40.0, 2.6),
            Dot::new(60.0, -2.6),
            Dot::new(60.0, 2.6),
        ];
        let start = Dot::new(0.0, 0.0);
        for convention in [
            RzConvention::Raw,
            RzConvention::Sticky,
            RzConvention::Smoothed,
        ] {
            let options = ValidateOptions {
                rz: convention,
                ..ValidateOptions::canonical()
            };
            let report = validate_path(&path, start, None, None, &options);
            let poses = foot_poses(&path, start, &options);
            let mut last = [f64::NAN; 2];
            for (index, (_, rz)) in poses.iter().enumerate() {
                last[index % 2] = *rz;
            }
            assert_eq!(poses.len(), path.len());
            assert_eq!(
                last,
                report.final_rz,
                "{} 约定下 foot_poses 末态与校验器不一致",
                convention.name()
            );
        }
    }

    /// raw（步向直接当朝向）与 smoothed（提交语义）在同一条有转弯的路径上必须不同
    /// —— 这是 viz 那条「不许用 raw 画足」负向对照的语义依据。
    #[test]
    fn raw_and_smoothed_headings_differ_on_turning_path() {
        let path = vec![
            Dot::new(0.0, 0.0),
            Dot::new(0.0, 6.0),
            Dot::new(20.0, 0.0),
            Dot::new(20.0, 6.0),
            // 同一只脚上一步朝 +x，这一步朝 +y：raw 直接跳到 90°，
            // smoothed 只走一半（HEADING_SMOOTHING = 0.5）→ 差 ~45°。
            Dot::new(20.0, 40.0),
            Dot::new(20.0, 46.0),
            Dot::new(0.0, 80.0),
            Dot::new(0.0, 86.0),
        ];
        let raw = foot_poses(
            &path,
            Dot::new(0.0, 0.0),
            &ValidateOptions {
                rz: RzConvention::Raw,
                ..ValidateOptions::canonical()
            },
        );
        let smoothed = foot_poses(&path, Dot::new(0.0, 0.0), &ValidateOptions::canonical());
        let max_diff = raw
            .iter()
            .zip(&smoothed)
            .map(|(a, b)| wrap_angle(a.1 - b.1).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_diff > 0.5,
            "raw 与 smoothed 的最大朝向差只有 {max_diff} rad，负向对照失去意义"
        );
    }
}
