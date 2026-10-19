//! 执行 + 汇总 + 门禁：场景 × seed 批量跑分、基线对比、回归判定。
//!
//! 一条 bench = 五个**严格按序**的阶段（[`crate::runstate`]）：场景 → 执行 → 校验 → 汇总 → 门禁。
//! 运行记录里的软指标（交叉数、横向间距）取自 `trapla-core::metrics`；硬约束结论一律来自
//! [`crate::validate`]（独立实现，不看 `metrics.stride`）。

use std::collections::BTreeMap;
use std::time::Instant;

use serde::{Deserialize, Serialize};
use trapla_core::config::RobotConfig;
use trapla_core::geom::Dot;
use trapla_core::metrics;
use trapla_core::terrain::Terrain;

use crate::planner::{ParityEvidence, PlanResult, PlanStatus, Planner, PlannerRole};
use crate::runstate::{Stage, StageEntry, StageError, StageMachine};
use crate::scenario::{self, Scenario};
use crate::sha256;
use crate::validate::{Limits, RzConvention, ValidateOptions, ValidationReport, Violation, validate_path};

/// bench 出错分类。
#[derive(Debug)]
pub enum BenchError {
    /// 阶段机拒绝（编程错误）。
    Stage(StageError),
    /// 输入错（未知场景 id 等）。
    Input(String),
    /// 输出落盘失败。
    Output(String),
}

impl std::fmt::Display for BenchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Stage(error) => write!(f, "{error}"),
            Self::Input(detail) | Self::Output(detail) => write!(f, "{detail}"),
        }
    }
}

impl std::error::Error for BenchError {}

impl From<StageError> for BenchError {
    fn from(error: StageError) -> Self {
        Self::Stage(error)
    }
}

/// 跑分配置。
#[derive(Debug, Clone)]
pub struct BenchConfig {
    pub rows: usize,
    pub cols: usize,
    pub seeds: Vec<u64>,
    pub scenario_ids: Option<Vec<String>>,
    pub base_config: RobotConfig,
    pub rz: RzConvention,
    pub coord_quantum: f64,
    /// 上一轮的 bench 报告（同格式 JSON）。
    pub baseline: Option<BenchReport>,
    /// 步数允许的相对增长（0.10 = +10%）。
    pub max_step_regression: f64,
    /// `proper_crossings` 允许的绝对增长。
    pub proper_slack: usize,
    /// 横向间距 min 允许的下降量。
    pub lateral_slack: f64,
    /// 路径落盘目录（每 run 一个 `<scenario>-<planner>-<seed>.csv`）。
    pub dump_paths_dir: Option<std::path::PathBuf>,
}

impl Default for BenchConfig {
    fn default() -> Self {
        Self {
            rows: 200,
            cols: 200,
            seeds: vec![1, 2],
            scenario_ids: None,
            base_config: RobotConfig::default(),
            rz: RzConvention::Smoothed,
            coord_quantum: 5e-7,
            baseline: None,
            max_step_regression: 0.10,
            proper_slack: 0,
            lateral_slack: 0.0,
            dump_paths_dir: None,
        }
    }
}

/// 一次执行的记录（执行层产出，校验层与报告层消费）。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunRecord {
    pub scenario_id: String,
    pub class: String,
    pub expected: String,
    pub planner: String,
    pub role: PlannerRole,
    pub seed: u64,
    pub status: PlanStatus,
    pub detail: String,
    pub parity0_is_left: bool,
    pub parity_evidence: ParityEvidence,
    pub points: usize,
    /// `trapla-core::metrics` 的步数（= 点数）。
    pub steps: usize,
    pub proper_crossings: usize,
    pub predicate_crossings: usize,
    /// 横向间距最小值（软指标）。
    pub lateral_min: Option<f64>,
    /// 同脚位移最大值（硬约束量，来自校验器）。
    pub stride_max_same_foot: Option<f64>,
    /// 转角绝对值最大值（弧度）。
    pub turn_max_abs_rad: Option<f64>,
    pub spacing_min: Option<f64>,
    pub spacing_max: Option<f64>,
    /// 超出输入量化界的硬越界步数。
    pub hard_violations: usize,
    /// 严格判据下的越界步数（含量化界内的边界值）。
    pub strict_violations: usize,
    pub violation_kinds: BTreeMap<String, usize>,
    pub standability_failures: usize,
    pub goal_error_center: Option<f64>,
    pub reached: bool,
    pub elapsed_ms: f64,
    pub path_sha256: String,
}

/// 场景 × 规划器的汇总。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Aggregate {
    pub scenario_id: String,
    pub planner: String,
    pub role: PlannerRole,
    pub runs: usize,
    pub steps_mean: f64,
    pub steps_max: usize,
    pub proper_crossings_sum: usize,
    pub lateral_min: Option<f64>,
    pub hard_violations: usize,
    pub strict_violations: usize,
    pub expected_met: bool,
}

/// 回归门禁结论。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Gate {
    pub passed: bool,
    pub failures: Vec<String>,
    /// 坏实现被校验器判出的硬越界总数（必须 > 0，否则校验器形同空转）。
    pub fixture_hard_violations: usize,
}

/// 一份完整的 bench 报告（也是 `--baseline` 的输入格式）。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchReport {
    pub rows: usize,
    pub cols: usize,
    pub seeds: Vec<u64>,
    pub planners: Vec<String>,
    pub stage_log: Vec<StageEntry>,
    pub runs: Vec<RunRecord>,
    pub aggregates: Vec<Aggregate>,
    pub gate: Gate,
}

/// 路径指纹：把每个点的两个 `f64` 位模式串起来取 sha256。
pub fn path_sha256(path: &[Dot]) -> String {
    let mut bytes = Vec::with_capacity(path.len() * 16);
    for dot in path {
        bytes.extend_from_slice(&dot.x().to_bits().to_le_bytes());
        bytes.extend_from_slice(&dot.y().to_bits().to_le_bytes());
    }
    sha256::hex_of(&bytes)
}

fn expected_met(expected: &str, status: PlanStatus, reached: bool) -> bool {
    match expected {
        "plannable" => status == PlanStatus::Planned && reached,
        "no_path" => status == PlanStatus::NoPath,
        "input_error" => status == PlanStatus::InputError,
        _ => false,
    }
}

/// 跑一轮 bench。
pub fn run(config: &BenchConfig, planners: &[&dyn Planner]) -> Result<BenchReport, BenchError> {
    let mut stages = StageMachine::new();

    // ① 场景层
    stages.begin(Stage::Scenario)?;
    let scenarios = scenario::select(config.rows, config.scenario_ids.as_deref())
        .map_err(BenchError::Input)?;
    let terrains: Vec<Terrain> = scenarios.iter().map(Scenario::terrain).collect();
    stages.end(Stage::Scenario)?;

    // ② 执行层
    stages.begin(Stage::Execute)?;
    let mut executions: Vec<(usize, usize, u64, PlanResult, f64)> = Vec::new();
    for (scenario_index, scenario) in scenarios.iter().enumerate() {
        for (planner_index, planner) in planners.iter().enumerate() {
            for seed in &config.seeds {
                let request = crate::planner::PlanRequest {
                    terrain: &terrains[scenario_index],
                    start: scenario.start_dot(),
                    goal: scenario.goal_dot(),
                    config: config.base_config,
                    seed: *seed,
                };
                let started = Instant::now();
                let result = planner.plan(&request);
                let elapsed_ms = started.elapsed().as_secs_f64() * 1000.0;
                if let Some(dir) = &config.dump_paths_dir {
                    std::fs::create_dir_all(dir).map_err(|error| {
                        BenchError::Output(format!("建目录 {} 失败: {error}", dir.display()))
                    })?;
                    if !result.path.is_empty() {
                        let path = dir.join(format!(
                            "{}-{}-{seed}.csv",
                            scenario.id,
                            planner.name()
                        ));
                        trapla_io::csv::write_xy(&path, &result.path).map_err(|error| {
                            BenchError::Output(format!("写 {} 失败: {error}", path.display()))
                        })?;
                    }
                }
                executions.push((scenario_index, planner_index, *seed, result, elapsed_ms));
            }
        }
    }
    stages.end(Stage::Execute)?;

    // ③ 校验层
    stages.begin(Stage::Validate)?;
    let limits = Limits::from(config.base_config);
    let active_distance = limits.active_distance;
    let mut records = Vec::with_capacity(executions.len());
    for (scenario_index, planner_index, seed, result, elapsed_ms) in &executions {
        let scenario = &scenarios[*scenario_index];
        let planner = planners[*planner_index];
        records.push(build_record(
            config,
            scenario,
            &terrains[*scenario_index],
            planner,
            *seed,
            result,
            *elapsed_ms,
            &limits,
            active_distance,
        ));
    }
    stages.end(Stage::Validate)?;

    // ④ 汇总
    stages.begin(Stage::Aggregate)?;
    let aggregates = aggregate(&records);
    stages.end(Stage::Aggregate)?;

    // ⑤ 门禁
    stages.begin(Stage::Gate)?;
    let mut gate = gate(&records, &aggregates, config);
    stages.end(Stage::Gate)?;
    gate.failures.shrink_to_fit();

    Ok(BenchReport {
        rows: config.rows,
        cols: config.cols,
        seeds: config.seeds.clone(),
        planners: planners.iter().map(|planner| planner.name().to_string()).collect(),
        stage_log: stages.log().to_vec(),
        runs: records,
        aggregates,
        gate,
    })
}

#[allow(clippy::too_many_arguments)]
fn build_record(
    config: &BenchConfig,
    scenario: &Scenario,
    terrain: &Terrain,
    planner: &dyn Planner,
    seed: u64,
    result: &PlanResult,
    elapsed_ms: f64,
    limits: &Limits,
    active_distance: f64,
) -> RunRecord {
    let mut record = RunRecord {
        scenario_id: scenario.id.clone(),
        class: scenario.class.name().to_string(),
        expected: scenario.expected.name().to_string(),
        planner: planner.name().to_string(),
        role: planner.role(),
        seed,
        status: result.status,
        detail: result.detail.clone(),
        parity0_is_left: result.parity0_is_left,
        parity_evidence: result.parity_evidence,
        points: result.path.len(),
        steps: result.path.len(),
        proper_crossings: 0,
        predicate_crossings: 0,
        lateral_min: None,
        stride_max_same_foot: None,
        turn_max_abs_rad: None,
        spacing_min: None,
        spacing_max: None,
        hard_violations: 0,
        strict_violations: 0,
        violation_kinds: BTreeMap::new(),
        standability_failures: 0,
        goal_error_center: None,
        reached: false,
        elapsed_ms,
        path_sha256: path_sha256(&result.path),
    };

    if result.path.is_empty() {
        return record;
    }

    let metrics = metrics::compute(&result.path, scenario.goal_dot());
    record.steps = metrics.steps;
    record.proper_crossings = metrics.proper_crossings;
    record.predicate_crossings = metrics.predicate_crossings;
    record.lateral_min = metrics
        .lateral_separation
        .iter()
        .copied()
        .reduce(f64::min);

    let report = validate(config, scenario, terrain, result, limits);
    record.goal_error_center = report.goal_error_center;
    record.reached = report
        .goal_error_center
        .is_some_and(|error| error < active_distance);
    record.hard_violations = report.hard_violation_steps;
    record.strict_violations = report.strict_violation_steps;
    record.standability_failures = report
        .hard_counts
        .get(&Violation::NormalAngle)
        .copied()
        .unwrap_or(0)
        + report
            .hard_counts
            .get(&Violation::FootOutOfGrid)
            .copied()
            .unwrap_or(0)
        + report
            .hard_counts
            .get(&Violation::DegeneratePlane)
            .copied()
            .unwrap_or(0);
    for (kind, count) in &report.hard_counts {
        record
            .violation_kinds
            .insert(kind.name().to_string(), *count);
    }
    record.stride_max_same_foot = report.steps.iter().map(|step| step.stride).reduce(f64::max);
    record.turn_max_abs_rad = report
        .steps
        .iter()
        .map(|step| step.turn_rad.abs())
        .reduce(f64::max);
    record.spacing_min = report.steps.iter().map(|step| step.spacing).reduce(f64::min);
    record.spacing_max = report.steps.iter().map(|step| step.spacing).reduce(f64::max);

    record
}

fn validate(
    config: &BenchConfig,
    scenario: &Scenario,
    terrain: &Terrain,
    result: &PlanResult,
    limits: &Limits,
) -> ValidationReport {
    let options = ValidateOptions {
        limits: *limits,
        rz: config.rz,
        parity0_is_left: result.parity0_is_left,
        coord_quantum: config.coord_quantum,
    };
    validate_path(
        &result.path,
        scenario.start_dot(),
        Some(scenario.goal_dot()),
        Some(terrain),
        &options,
    )
}

fn aggregate(records: &[RunRecord]) -> Vec<Aggregate> {
    let mut keys: Vec<(String, String)> = Vec::new();
    for record in records {
        let key = (record.scenario_id.clone(), record.planner.clone());
        if !keys.contains(&key) {
            keys.push(key);
        }
    }

    keys.into_iter()
        .map(|(scenario_id, planner)| {
            let group: Vec<&RunRecord> = records
                .iter()
                .filter(|record| record.scenario_id == scenario_id && record.planner == planner)
                .collect();
            let runs = group.len();
            let steps_mean = group.iter().map(|r| r.steps as f64).sum::<f64>() / runs.max(1) as f64;
            let steps_max = group.iter().map(|r| r.steps).max().unwrap_or(0);
            let proper_crossings_sum = group.iter().map(|r| r.proper_crossings).sum();
            let lateral_min = group
                .iter()
                .filter_map(|r| r.lateral_min)
                .reduce(f64::min);
            let hard_violations = group.iter().map(|r| r.hard_violations).sum();
            let strict_violations = group.iter().map(|r| r.strict_violations).sum();
            let expected_met = group.iter().all(|r| {
                expected_met(&r.expected, r.status, r.reached)
                    || r.role == PlannerRole::Fixture
            });
            Aggregate {
                scenario_id,
                planner,
                role: group.first().map_or(PlannerRole::Fixture, |r| r.role),
                runs,
                steps_mean,
                steps_max,
                proper_crossings_sum,
                lateral_min,
                hard_violations,
                strict_violations,
                expected_met,
            }
        })
        .collect()
}

fn gate(records: &[RunRecord], aggregates: &[Aggregate], config: &BenchConfig) -> Gate {
    let mut failures = Vec::new();

    // 参考实现：期望类别必须达成，硬约束必须零越界。
    for record in records
        .iter()
        .filter(|record| record.role == PlannerRole::Reference)
    {
        if !expected_met(&record.expected, record.status, record.reached) {
            failures.push(format!(
                "参考实现 {} 在场景 {}(seed {}) 未达成期望 {}（实际 {}，末点距目标 {:?}）",
                record.planner, record.scenario_id, record.seed, record.expected,
                record.status.name(), record.goal_error_center
            ));
        }
        if record.hard_violations > 0 {
            failures.push(format!(
                "参考实现 {} 在场景 {}(seed {}) 有 {} 步硬约束越界: {:?}",
                record.planner, record.scenario_id, record.seed, record.hard_violations,
                record.violation_kinds
            ));
        }
    }

    // 负向对照：坏实现必须至少被判出一次硬越界，否则校验器可能是空转的。
    let fixture_hard_violations: usize = records
        .iter()
        .filter(|record| record.role == PlannerRole::Fixture)
        .map(|record| record.hard_violations)
        .sum();
    if fixture_hard_violations == 0 {
        failures.push(
            "负向对照失效：对照坏实现在所有场景 × seed 上都没有被判出硬越界，说明校验器没在真的检查"
                .to_string(),
        );
    }

    if let Some(baseline) = &config.baseline {
        failures.extend(compare_baseline(aggregates, baseline, config));
    }

    Gate {
        passed: failures.is_empty(),
        failures,
        fixture_hard_violations,
    }
}

fn compare_baseline(
    aggregates: &[Aggregate],
    baseline: &BenchReport,
    config: &BenchConfig,
) -> Vec<String> {
    let mut failures = Vec::new();
    for aggregate in aggregates {
        let key = (aggregate.scenario_id.as_str(), aggregate.planner.as_str());
        let Some(base) = baseline.aggregates.iter().find(|candidate| {
            candidate.scenario_id == key.0 && candidate.planner == key.1
        }) else {
            failures.push(format!(
                "基线里没有 {}/{} 的汇总，无法判定回归（fail-closed）",
                key.0, key.1
            ));
            continue;
        };

        let allowed = base.steps_mean * (1.0 + config.max_step_regression) + 1e-9;
        if aggregate.steps_mean > allowed {
            failures.push(format!(
                "步数回归 {}/{}: {:.2} > 基线 {:.2} × (1+{:.2})",
                key.0, key.1, aggregate.steps_mean, base.steps_mean, config.max_step_regression
            ));
        }
        if aggregate.proper_crossings_sum > base.proper_crossings_sum + config.proper_slack {
            failures.push(format!(
                "proper_crossings 回归 {}/{}: {} > 基线 {}+{}",
                key.0, key.1, aggregate.proper_crossings_sum, base.proper_crossings_sum, config.proper_slack
            ));
        }
        if let (Some(current), Some(before)) = (aggregate.lateral_min, base.lateral_min)
            && current < before - config.lateral_slack
        {
            failures.push(format!(
                "横向间距最小值回归 {}/{}: {:.4} < 基线 {:.4}-{}",
                key.0, key.1, current, before, config.lateral_slack
            ));
        }
        if aggregate.hard_violations > base.hard_violations {
            failures.push(format!(
                "硬约束越界回归 {}/{}: {} > 基线 {}",
                key.0, key.1, aggregate.hard_violations, base.hard_violations
            ));
        }
    }
    failures
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner::{NaiveStraightPlanner, RobotPlanner};

    fn small_config() -> BenchConfig {
        BenchConfig {
            rows: 64,
            cols: 64,
            seeds: vec![1, 2],
            scenario_ids: Some(vec![
                "flat".to_string(),
                "step".to_string(),
                "corridor".to_string(),
                "fault".to_string(),
                "oob".to_string(),
            ]),
            ..BenchConfig::default()
        }
    }

    #[test]
    fn bench_is_deterministic_and_separates_good_from_bad() {
        let config = small_config();
        let robot = RobotPlanner;
        let naive = NaiveStraightPlanner;
        let first = run(&config, &[&robot, &naive]).expect("第一轮");
        let second = run(&config, &[&robot, &naive]).expect("第二轮");

        let hashes = |report: &BenchReport| -> Vec<String> {
            report
                .runs
                .iter()
                .map(|run| {
                    format!(
                        "{}/{}/{}={}",
                        run.scenario_id, run.planner, run.seed, run.path_sha256
                    )
                })
                .collect()
        };
        assert_eq!(hashes(&first), hashes(&second), "同 seed 两轮路径必须逐位一致");

        assert!(
            first.gate.fixture_hard_violations > 0,
            "坏实现必须被判出硬越界"
        );
        assert_eq!(first.stage_log.len(), Stage::ORDER.len());
    }
}
