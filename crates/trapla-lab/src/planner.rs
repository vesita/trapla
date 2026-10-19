//! 执行层：把「候选算法」与跑分框架解耦的 [`Planner`] 抽象。
//!
//! 框架只认这个接口：给一张地形 + 起终点 + 配置 + seed，交回一条点位序列（或失败类别）。
//! 现有算法 [`RobotPlanner`] 是**参考实现**；[`NaiveStraightPlanner`] 是**故意坏**的对照实现
//! （只朝目标走、不查任何约束），用来证明框架能区分好坏 —— 见 [`PlannerRole`]。

use serde::{Deserialize, Serialize};
use trapla_core::config::RobotConfig;
use trapla_core::error::CoreError;
use trapla_core::geom::Dot;
use trapla_core::rng::Rng;
use trapla_core::robot::Robot;
use trapla_core::terrain::Terrain;

use crate::validate::{Limits, initial_feet};

/// 一次规划请求。
#[derive(Debug, Clone, Copy)]
pub struct PlanRequest<'a> {
    pub terrain: &'a Terrain,
    pub start: Dot,
    pub goal: Dot,
    pub config: RobotConfig,
    pub seed: u64,
}

/// 规划结局分类。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlanStatus {
    /// 交回了非空点位序列。
    Planned,
    /// 明确判定不可达（`CoreError::NoPath`）。
    NoPath,
    /// 输入被拒（起点/终点越界等）。
    InputError,
}

impl PlanStatus {
    pub fn name(self) -> &'static str {
        match self {
            Self::Planned => "planned",
            Self::NoPath => "no_path",
            Self::InputError => "input_error",
        }
    }
}

/// 奇偶脚映射的来源。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParityEvidence {
    /// 由 `Robot::feet()` 与序列末点比对得出（公开 API，精确）。
    Matched,
    /// 由规划器自己的约定声明（坏实现没有可核对的机器人状态）。
    Assumed,
}

/// 一次规划的结果。
#[derive(Debug, Clone)]
pub struct PlanResult {
    pub status: PlanStatus,
    pub path: Vec<Dot>,
    /// 点位序列里**偶数下标**那一只脚是不是左脚。
    pub parity0_is_left: bool,
    pub parity_evidence: ParityEvidence,
    /// 规划器内部末态（足位 + 朝向）：给「重建 rz 是否等于真实朝向」这类对照实验留证据。
    /// 只有能提供真实状态的规划器才填；坏实现留 `None`。
    pub final_feet: Option<[(Dot, f64); 2]>,
    /// 失败原因 / 备注（成功时为空串）。
    pub detail: String,
}

impl PlanResult {
    fn failed(status: PlanStatus, detail: impl Into<String>) -> Self {
        Self {
            status,
            path: Vec::new(),
            parity0_is_left: true,
            parity_evidence: ParityEvidence::Assumed,
            final_feet: None,
            detail: detail.into(),
        }
    }
}

/// 规划器的角色：门禁怎么对待它的产出。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlannerRole {
    /// 参考实现：门禁要求它满足期望类别、且硬约束零违规。
    Reference,
    /// 对照坏实现：门禁**不**要求它合规，而是要求它至少被判出一次硬违规
    /// （否则说明校验器是空转的）。
    Fixture,
}

/// 候选算法接口。
pub trait Planner {
    /// 稳定名字（出现在报告与基线里）。
    fn name(&self) -> &'static str;
    /// 一句话说明。
    fn description(&self) -> &'static str;
    fn role(&self) -> PlannerRole;
    fn plan(&self, request: &PlanRequest<'_>) -> PlanResult;
}

/// 起点 / 终点是否落在栅格内（与 CLI `plan` 同源的判据：`Dot::as_idx` + `Grid::in_bounds`）。
fn in_bounds(terrain: &Terrain, dot: Dot) -> bool {
    dot.x().is_finite() && dot.y().is_finite() && terrain.grid.in_bounds(dot.as_idx())
}

/// 参考实现：`Robot::find_path`。
pub struct RobotPlanner;

impl Planner for RobotPlanner {
    fn name(&self) -> &'static str {
        "robot"
    }

    fn description(&self) -> &'static str {
        "trapla-core 的 Robot::find_path（缩放图 A* + 足步生成）"
    }

    fn role(&self) -> PlannerRole {
        PlannerRole::Reference
    }

    fn plan(&self, request: &PlanRequest<'_>) -> PlanResult {
        if !in_bounds(request.terrain, request.start) {
            return PlanResult::failed(PlanStatus::InputError, "起点越出栅格边界");
        }
        if !in_bounds(request.terrain, request.goal) {
            return PlanResult::failed(PlanStatus::InputError, "目标点越出栅格边界");
        }

        let mut robot = Robot::new(request.config, request.start);
        let mut rng = Rng::from_seed(request.seed);
        match robot.find_path(request.terrain, request.goal, &mut rng) {
            Ok(path) => {
                let (parity0_is_left, parity_evidence) = infer_parity(&robot, &path);
                let final_feet = [
                    (robot.feet()[0].position, robot.feet()[0].rz),
                    (robot.feet()[1].position, robot.feet()[1].rz),
                ];
                PlanResult {
                    status: PlanStatus::Planned,
                    path,
                    parity0_is_left,
                    parity_evidence,
                    final_feet: Some(final_feet),
                    detail: String::new(),
                }
            }
            Err(CoreError::NoPath { .. }) => {
                PlanResult::failed(PlanStatus::NoPath, "缩放图 A* 或候选集判无路径")
            }
            Err(other) => PlanResult::failed(PlanStatus::InputError, other.to_string()),
        }
    }
}

/// 由「序列末点 = 最后移动的那只脚」这一事实反推奇偶脚映射。
///
/// `find_path` 每推入一个点都对应一次 `walk_update`，末点必等于 `robot.feet()` 中某一只脚的
/// 当前位置（`position` 是精确赋值，不是重算）。取不到就退回假定值并登记。
fn infer_parity(robot: &Robot, path: &[Dot]) -> (bool, ParityEvidence) {
    let Some(last) = path.last() else {
        return (true, ParityEvidence::Assumed);
    };
    let last_index = path.len() - 1;
    if *last == robot.feet()[0].position {
        (last_index.is_multiple_of(2), ParityEvidence::Matched)
    } else if *last == robot.feet()[1].position {
        (!last_index.is_multiple_of(2), ParityEvidence::Matched)
    } else {
        (true, ParityEvidence::Assumed)
    }
}

/// 对照坏实现：只朝目标走，不查步长 / 转角 / 间距 / 可站立。
///
/// 步长故意取 `1.5 × max_stride`，**必然**越界；也从不返回无路径或输入错。
pub struct NaiveStraightPlanner;

impl Planner for NaiveStraightPlanner {
    fn name(&self) -> &'static str {
        "naive_straight"
    }

    fn description(&self) -> &'static str {
        "只朝目标直线迈步、不查任何硬约束（对照用坏实现）"
    }

    fn role(&self) -> PlannerRole {
        PlannerRole::Fixture
    }

    fn plan(&self, request: &PlanRequest<'_>) -> PlanResult {
        const MAX_STEPS: usize = 256;
        let limits = Limits::from(request.config);
        let [(mut left, _), (mut right, _)] = initial_feet(request.start, &limits);
        let step = request.config.max_stride * 1.5;

        let mut path = Vec::new();
        for index in 0..MAX_STEPS {
            let foot = index % 2;
            let current = if foot == 0 { left } else { right };
            if current.distance(request.goal) < request.config.active_distance {
                break;
            }
            let direction = current.angle(request.goal);
            let next = Dot::new(
                current.x() + step * direction.cos(),
                current.y() + step * direction.sin(),
            );
            if foot == 0 {
                left = next;
            } else {
                right = next;
            }
            path.push(next);
        }

        PlanResult {
            status: PlanStatus::Planned,
            path,
            parity0_is_left: true,
            parity_evidence: ParityEvidence::Assumed,
            final_feet: Some([(left, 0.0), (right, 0.0)]),
            detail: format!("固定步长 {step}（1.5 × max_stride），不查约束"),
        }
    }
}

/// 内置规划器清单：`(参考实现, 坏实现)`。
pub fn builtin() -> (RobotPlanner, NaiveStraightPlanner) {
    (RobotPlanner, NaiveStraightPlanner)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario;
    use crate::validate::{RzConvention, ValidateOptions, validate_path};

    fn flat_request<'a>(terrain: &'a Terrain, config: RobotConfig) -> PlanRequest<'a> {
        PlanRequest {
            terrain,
            start: Dot::new(10.0, 10.0),
            goal: Dot::new(60.0, 60.0),
            config,
            seed: 42,
        }
    }

    #[test]
    fn robot_planner_maps_parity_from_robot_state() {
        let scenario = scenario::builtin(128)
            .into_iter()
            .find(|s| s.id == "flat")
            .expect("内置里有 flat");
        let terrain = scenario.terrain();
        let request = PlanRequest {
            terrain: &terrain,
            start: scenario.start_dot(),
            goal: scenario.goal_dot(),
            config: RobotConfig::default(),
            seed: scenario.seed,
        };
        let result = RobotPlanner.plan(&request);
        assert_eq!(result.status, PlanStatus::Planned, "{}", result.detail);
        assert!(!result.path.is_empty());
        assert_eq!(result.parity_evidence, ParityEvidence::Matched);
    }

    #[test]
    fn naive_planner_is_caught_by_the_validator() {
        let scenario = scenario::builtin(128)
            .into_iter()
            .find(|s| s.id == "flat")
            .expect("内置里有 flat");
        let terrain = scenario.terrain();
        let request = PlanRequest {
            terrain: &terrain,
            start: scenario.start_dot(),
            goal: scenario.goal_dot(),
            config: RobotConfig::default(),
            seed: scenario.seed,
        };
        let result = NaiveStraightPlanner.plan(&request);
        assert_eq!(result.status, PlanStatus::Planned);

        let options = ValidateOptions {
            limits: Limits::from(request.config),
            rz: RzConvention::Raw,
            parity0_is_left: result.parity0_is_left,
            coord_quantum: 0.0,
        };
        let report = validate_path(
            &result.path,
            request.start,
            Some(request.goal),
            Some(&terrain),
            &options,
        );
        assert!(
            report.hard_counts.contains_key(&crate::validate::Violation::Stride),
            "坏实现的固定步长必须被判出步长越界: {:?}",
            report.hard_counts
        );
    }

    #[test]
    fn out_of_bounds_is_input_error() {
        let scenario = scenario::builtin(128)
            .into_iter()
            .find(|s| s.id == "oob")
            .expect("内置里有 oob");
        let terrain = scenario.terrain();
        let request = flat_request(&terrain, RobotConfig::default());
        let request = PlanRequest {
            goal: scenario.goal_dot(),
            ..request
        };
        assert_eq!(RobotPlanner.plan(&request).status, PlanStatus::InputError);
    }
}
