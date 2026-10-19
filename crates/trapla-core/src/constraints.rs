//! 硬约束的**显式、单一事实源**。
//!
//! 几何在这里只有一份实现；**「用哪种姿态判定」由调用方显式给出** —— 本模块从不自己推导
//! 候选姿态。这正是本次修复（候选判定 vs 落足提交的姿态漂移）留下的接缝：
//! 姿态推导在 `robot.rs` 里只有两处命名入口（候选 `Robot::next` / 提交 `Robot::next_committed`），
//! 判定本身一律走这里。
//!
//! 四条硬约束族（`doc/project.md` §3.1.1 / §3.2.4）：
//!
//! 1. 步长：同脚位移严格 `< max_stride`；
//! 2. 转向角：`wrap_angle(步向 − 支撑脚 rz)` 归一到 `(-π, π]` 后严格落在 `(-max_turn, max_turn)`；
//! 3. 双足间距：支撑脚纵向中心线到候选足四角的最小垂距 − 支撑脚半宽，夹紧 `>= 0` 后落在闭区间
//!    `[min − SEPARATION_EPS, max + SEPARATION_EPS]`；
//! 4. 足可站立：覆盖格全在图内 **且** 拟合平面法向角严格 `< max_normal_angle`；
//!    拆成 [`HardConstraint::FootOutOfGrid`] / [`HardConstraint::NormalAngle`] 两条判定，
//!    与独立校验器（`trapla-lab` 的 `Violation`）逐条对应。
//!
//! 阈值一律来自 [`RobotConfig`] 或 [`crate::config`] 的常量；本模块不写任何阈值字面量。
//! 仅有的数值字面量是 `0.0`（间距夹紧下界）与 `f64::NEG_INFINITY`（布尔型约束的违约裕度）。

use crate::config::{RobotConfig, SEPARATION_EPS};
use crate::foot::Foot;
use crate::geom::{Dot, Line};
use crate::terrain::Terrain;

/// 硬约束的类别。
///
/// 前三条与 `doc/project.md` §3.2.4 的三条一一对应；第 4 族「足可站立」由
/// [`HardConstraint::FootOutOfGrid`]（覆盖格在图内）与 [`HardConstraint::NormalAngle`]
/// （法向角）两条组成 —— 拆开是为了让越界类别与独立校验器逐条对齐，而不是合并成一个布尔。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum HardConstraint {
    /// 同脚位移严格 `< max_stride`。
    Stride,
    /// 转向角严格落在开区间 `(-max_turn, max_turn)`。
    Turn,
    /// 双足间距落在闭区间 `[min − eps, max + eps]`。
    Spacing,
    /// 足覆盖格有落在地图外的。
    FootOutOfGrid,
    /// 拟合平面法向角 `>= max_normal_angle`，或拟合退化。
    NormalAngle,
}

impl HardConstraint {
    /// 全部类别（顺序与 [`check_step`] / [`check_standable`] 的返回数组一致）。
    pub const ALL: [Self; 5] = [
        Self::Stride,
        Self::Turn,
        Self::Spacing,
        Self::FootOutOfGrid,
        Self::NormalAngle,
    ];

    /// 稳定名字（与 `trapla-lab` 的报告用词一致）。
    pub fn name(self) -> &'static str {
        match self {
            Self::Stride => "stride",
            Self::Turn => "turn",
            Self::Spacing => "spacing",
            Self::FootOutOfGrid => "foot_out_of_grid",
            Self::NormalAngle => "normal_angle",
        }
    }
}

/// 单条硬约束的判定结果。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Verdict {
    /// 判的是哪一条。
    pub constraint: HardConstraint,
    /// 是否满足（判据与冻结谓词逐位一致，见各检定函数的文档）。
    pub satisfied: bool,
    /// 离最近那条边界还剩多少余量：`> 0` = 满足，`<= 0` = 违约。
    ///
    /// 严格型判据（步长 / 转向角 / 法向角）的 `margin = 上限 − 实际值`；
    /// 闭区间型（间距）的 `margin = min(实际值 − 下界, 上界 − 实际值)`；
    /// 布尔型（覆盖格在图内）满足时恒为 [`f64::INFINITY`]。
    /// 法向角那一条在覆盖格出界或拟合退化时取 [`f64::NEG_INFINITY`]
    /// （数值不存在，不用一个假的有限裕度冒充）。
    pub margin: f64,
}

impl Verdict {
    fn new(constraint: HardConstraint, satisfied: bool, margin: f64) -> Self {
        Self {
            constraint,
            satisfied,
            margin,
        }
    }
}

/// 一组判定是否**全部**满足。
pub fn all_satisfied(verdicts: &[Verdict]) -> bool {
    verdicts.iter().all(|verdict| verdict.satisfied)
}

/// 第一条不满足的判定（按传入顺序）。
pub fn first_violation(verdicts: &[Verdict]) -> Option<Verdict> {
    verdicts.iter().copied().find(|verdict| !verdict.satisfied)
}

/// 步长：`|candidate − swing| < max_stride`（严格小于）。
///
/// 原 `Robot::satisfy_stride`（`robot.rs`）的几何；`candidate` 取的是候选**位置** ——
/// 步长与姿态无关，两种姿态口径下同值。
pub fn stride(candidate: Dot, swing: Dot, config: &RobotConfig) -> Verdict {
    let value = swing.distance(candidate);
    Verdict::new(
        HardConstraint::Stride,
        value < config.max_stride,
        config.max_stride - value,
    )
}

/// 转向角：`wrap_angle(步向 − support_rz)` 严格落在 `(-max_turn, max_turn)`。
///
/// 「步向」= `swing` 指向候选位置的方向。先归一到 `(-π, π]` 再判（有意分歧 D1，
/// 见 `doc/known_divergences.md`）；姿态口径同样不影响本判据。
pub fn turn(candidate: Dot, swing: Dot, support_rz: f64, config: &RobotConfig) -> Verdict {
    let angle = crate::metrics::wrap_angle(swing.angle(candidate) - support_rz);
    Verdict::new(
        HardConstraint::Turn,
        angle > -config.max_turn && angle < config.max_turn,
        config.max_turn - angle.abs(),
    )
}

/// 双足间距：候选足**按传入姿态**（位置 + `rz`）取四角，量到支撑脚纵向中心线。
///
/// 几何口径逐字复刻 `doc/project.md` §3.2.4 与冻结谓词 `satisfy_spacing`：
/// 最小垂距 − 支撑脚半宽，夹紧 `>= 0`，闭区间 `[min − eps, max + eps]`。
///
/// **姿态由调用方给出**：这里读的是 `candidate.rz`，所以同一位置用 `next` 姿态还是
/// `next_committed` 姿态会得到不同结论 —— 这正是本次修复的对象。
pub fn spacing(candidate: &Foot, support: &Foot, config: &RobotConfig) -> Verdict {
    let half_width = support.shape.half_width();
    let center_line = Line::from_point_angle(support.position, support.rz);

    let mut min_dist = f64::INFINITY;
    for pt in candidate.corners() {
        min_dist = min_dist.min(center_line.dist_to_point(pt));
    }

    let value = (min_dist - half_width).max(0.0);
    let lower = config.min_separation - SEPARATION_EPS;
    let upper = config.max_separation + SEPARATION_EPS;

    Verdict::new(
        HardConstraint::Spacing,
        value >= lower && value <= upper,
        (value - lower).min(upper - value),
    )
}

/// 覆盖格是否全在地图内（原 `Foot::standable` 的第一步）。
pub fn foot_in_grid(candidate: &Foot, terrain: &Terrain) -> Verdict {
    let rows = terrain.grid.rows() as i32;
    let cols = terrain.grid.cols() as i32;
    let inside = candidate
        .cover()
        .iter()
        .all(|idx| idx.x >= 0 && idx.x < rows && idx.y >= 0 && idx.y < cols);

    Verdict::new(
        HardConstraint::FootOutOfGrid,
        inside,
        if inside {
            f64::INFINITY
        } else {
            f64::NEG_INFINITY
        },
    )
}

/// 拟合平面法向角是否严格 `< max_normal_angle`（拟合退化按不满足计）。
pub fn normal_angle(candidate: &Foot, terrain: &Terrain, config: &RobotConfig) -> Verdict {
    let points: Vec<Dot> = candidate.cover().iter().map(|idx| idx.as_dot()).collect();
    match terrain.normal_angle(&points) {
        Ok(angle) => Verdict::new(
            HardConstraint::NormalAngle,
            angle < config.max_normal_angle,
            config.max_normal_angle - angle,
        ),
        Err(_) => Verdict::new(HardConstraint::NormalAngle, false, f64::NEG_INFINITY),
    }
}

/// 位置型三条（步长 / 转向角 / 间距）一次判完，顺序同 [`HardConstraint::ALL`] 的前三项。
///
/// `candidate` 的姿态由调用方给定 —— `Robot::position_check` 传 `next` 姿态，
/// `Robot::position_check_committed` 传 `next_committed` 姿态。
pub fn check_step(
    candidate: &Foot,
    swing: &Foot,
    support: &Foot,
    config: &RobotConfig,
) -> [Verdict; 3] {
    [
        stride(candidate.position, swing.position, config),
        turn(candidate.position, swing.position, support.rz, config),
        spacing(candidate, support, config),
    ]
}

/// 「足可站立」一族两条一次判完，顺序同 [`HardConstraint::ALL`] 的后两项。
///
/// 覆盖格出界时不再拟合平面（与 `Foot::standable` 的短路一致）：此时 `foot_in_grid` 违约、
/// `normal_angle` 登记为不满足且裕度 `-∞`。
pub fn check_standable(candidate: &Foot, terrain: &Terrain, config: &RobotConfig) -> [Verdict; 2] {
    let grid = foot_in_grid(candidate, terrain);
    let normal = if grid.satisfied {
        normal_angle(candidate, terrain, config)
    } else {
        Verdict::new(HardConstraint::NormalAngle, false, f64::NEG_INFINITY)
    };
    [grid, normal]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::FootShape;

    fn config() -> RobotConfig {
        RobotConfig::default()
    }

    fn foot(x: f64, y: f64, rz: f64) -> Foot {
        Foot::new(Dot::new(x, y), rz, FootShape::biped())
    }

    #[test]
    fn stride_is_strict() {
        let cfg = config();
        let swing = Dot::new(0.0, 0.0);
        assert!(!stride(Dot::new(cfg.max_stride, 0.0), swing, &cfg).satisfied);
        assert!(stride(Dot::new(cfg.max_stride - 1e-6, 0.0), swing, &cfg).satisfied);
        assert!(stride(Dot::new(0.0, 0.0), swing, &cfg).margin > 0.0);
    }

    #[test]
    fn turn_is_strict_and_wrapped() {
        let cfg = config();
        let swing = Dot::new(0.0, 0.0);
        let at_limit = Dot::new(10.0 * cfg.max_turn.cos(), 10.0 * cfg.max_turn.sin());
        assert!(!turn(at_limit, swing, 0.0, &cfg).satisfied);

        // 支撑脚朝向差一个整圈时不得判成伪大角（D1）
        let wrapped = turn(at_limit, swing, std::f64::consts::TAU, &cfg);
        assert!(!wrapped.satisfied);
        assert!((wrapped.margin - 0.0).abs() < 1e-12);
    }

    #[test]
    fn spacing_uses_the_pose_it_is_given() {
        let cfg = config();
        // 支撑脚在原点、朝向 0 ⇒ 中心线是 x 轴，侧距 = 候选足四角最小 |y| − 半宽
        let support = foot(0.0, 0.0, 0.0);
        // 姿态与中心线平行：四角 y = 3.7 / 6.7 ⇒ 间距 2.2，落在区间内
        assert!(spacing(&foot(0.0, 5.2, 0.0), &support, &cfg).satisfied);
        // 同一位置、姿态转到与中心线垂直：半长把四角撑到 y = 2.7 ⇒ 间距 1.2 ⇒ 违约
        let rotated = foot(0.0, 5.2, std::f64::consts::FRAC_PI_2);
        let verdict = spacing(&rotated, &support, &cfg);
        assert!(!verdict.satisfied, "同一位置换姿态必须换结论：{verdict:?}");
        assert!(verdict.margin < 0.0);
    }
}
