//! 报告层：人读表 + 机器 JSON / CSV 的渲染。判定逻辑不在这里（这里只排版）。

use std::fmt::Write as _;

use crate::bench::{BenchReport, RunRecord};
use crate::planner::PlannerRole;
use crate::scenario::Scenario;
use crate::validate::ValidationReport;

fn optional(value: Option<f64>, decimals: usize) -> String {
    match value {
        Some(value) if value.is_finite() => format!("{value:.decimals$}"),
        Some(_) => "non-finite".to_string(),
        None => "-".to_string(),
    }
}

/// 校验报告的人读表（每一步一行）+ 汇总。
pub fn validation_table(report: &ValidationReport) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "点位 {} 个 | 偶数下标={} | rz 约定={} | 量化={:e} | 线性容差={:.3e}",
        report.point_count,
        if report.parity0_is_left { "左脚" } else { "右脚" },
        report.rz_convention.name(),
        report.coord_quantum,
        report.linear_margin
    );
    let _ = writeln!(
        out,
        "阈值: max_stride={} min_sep={} max_sep={} max_turn={}° max_normal={}° active_distance={}",
        report.limits.max_stride,
        report.limits.min_separation,
        report.limits.max_separation,
        report.limits.max_turn.to_degrees(),
        report.limits.max_normal_angle.to_degrees(),
        report.limits.active_distance
    );
    let _ = writeln!(
        out,
        "{:>5} {:>4} {:>11} {:>10} {:>10} {:>10}  flags",
        "step", "foot", "stride", "turn(deg)", "spacing", "normal(deg)"
    );

    for step in &report.steps {
        let mut flags = String::new();
        for violation in &step.hard_violations {
            let _ = write!(flags, "!{} ", violation.name());
        }
        for violation in &step.strict_violations {
            if !step.hard_violations.contains(violation) {
                let _ = write!(flags, "~{} ", violation.name());
            }
        }
        let _ = writeln!(
            out,
            "{:>5} {:>4} {:>11.6} {:>10.6} {:>10.6} {:>10}  {}",
            step.index,
            step.foot,
            step.stride,
            step.turn_rad.to_degrees(),
            step.spacing,
            optional(step.normal_angle_rad.map(f64::to_degrees), 6),
            flags.trim_end()
        );
    }

    let _ = writeln!(
        out,
        "汇总: 严格越界步数={} 超量化界步数={} | 严格 {:?} | 硬 {:?}",
        report.strict_violation_steps,
        report.hard_violation_steps,
        report.strict_counts,
        report.hard_counts
    );
    let _ = writeln!(
        out,
        "末端: 落足点距 goal={} 双足中心距 goal={}",
        optional(report.goal_error_foot, 6),
        optional(report.goal_error_center, 6)
    );
    for skipped in &report.skipped {
        let _ = writeln!(out, "跳过: {skipped}");
    }
    if report.passed() {
        let _ = writeln!(out, "结论: 通过（硬约束全部满足）");
    } else {
        let _ = writeln!(out, "结论: 不通过（存在硬约束越界）");
    }
    out
}

/// 场景清单表。
pub fn scenario_table(scenarios: &[Scenario]) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{:<12} {:<16} {:<12} {:>5} {:>6} {:<14} {:<14} grid_sha256",
        "id", "class", "expected", "size", "seed", "start", "goal"
    );
    for scenario in scenarios {
        let _ = writeln!(
            out,
            "{:<12} {:<16} {:<12} {:>5} {:>6} {:<14} {:<14} {}",
            scenario.id,
            scenario.class.name(),
            scenario.expected.name(),
            format!("{}x{}", scenario.rows, scenario.cols),
            scenario.seed,
            format!("({}, {})", scenario.start.0, scenario.start.1),
            format!("({}, {})", scenario.goal.0, scenario.goal.1),
            &scenario.grid_fingerprint()[..16]
        );
    }
    out
}

/// 一条运行记录的人读行。
fn run_row(run: &RunRecord) -> String {
    format!(
        "{:<12} {:<15} {:>4} {:>6} {:>6} {:>10} {:>10} {:>6} {:<7} {:>9.2}",
        run.scenario_id,
        run.planner,
        run.seed,
        run.steps,
        run.proper_crossings,
        optional(run.lateral_min, 4),
        optional(run.stride_max_same_foot, 4),
        run.hard_violations,
        run.status.name(),
        run.elapsed_ms
    )
}

/// bench 报告的汇总表 + 门禁结论。
pub fn bench_table(report: &BenchReport) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "尺寸 {}x{} | seeds {:?} | 规划器 {:?}",
        report.rows, report.cols, report.seeds, report.planners
    );
    let _ = writeln!(
        out,
        "{:<12} {:<15} {:>4} {:>6} {:>6} {:>10} {:>10} {:>6} {:<7} {:>9}",
        "scenario", "planner", "seed", "steps", "proper", "lateral_min", "stride_max", "hard", "status", "ms"
    );
    for run in &report.runs {
        let _ = writeln!(out, "{}", run_row(run));
    }

    let _ = writeln!(out, "\n汇总（按场景 × 规划器）");
    let _ = writeln!(
        out,
        "{:<12} {:<15} {:>5} {:>10} {:>7} {:>11} {:>6} {:>6}",
        "scenario", "planner", "runs", "steps_mean", "proper", "lateral_min", "hard", "期望达成"
    );
    for aggregate in &report.aggregates {
        let _ = writeln!(
            out,
            "{:<12} {:<15} {:>5} {:>10.2} {:>7} {:>11} {:>6} {:>6}",
            aggregate.scenario_id,
            aggregate.planner,
            aggregate.runs,
            aggregate.steps_mean,
            aggregate.proper_crossings_sum,
            optional(aggregate.lateral_min, 4),
            aggregate.hard_violations,
            if aggregate.role == PlannerRole::Fixture {
                "n/a"
            } else if aggregate.expected_met {
                "是"
            } else {
                "否"
            }
        );
    }

    let _ = writeln!(out, "\n阶段耗时: {:?}", report.stage_log);
    let _ = writeln!(
        out,
        "负向对照: 坏实现被判出的硬越界步数 = {}",
        report.gate.fixture_hard_violations
    );
    if report.gate.passed {
        let _ = writeln!(out, "门禁: 通过");
    } else {
        let _ = writeln!(out, "门禁: 不通过，{} 条失败", report.gate.failures.len());
        for failure in &report.gate.failures {
            let _ = writeln!(out, "  - {failure}");
        }
    }
    out
}

/// bench 明细 CSV（一条运行一行）。
pub fn bench_csv(report: &BenchReport) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "scenario,class,expected,planner,role,seed,status,steps,proper_crossings,lateral_min,\
         stride_max_same_foot,turn_max_deg,spacing_min,spacing_max,hard_violations,\
         standability_failures,goal_error_center,reached,elapsed_ms,path_sha256"
    );
    for run in &report.runs {
        let elapsed_text = format!("{:.3}", run.elapsed_ms);
        let _ = writeln!(
            out,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            run.scenario_id,
            run.class,
            run.expected,
            run.planner,
            match run.role {
                PlannerRole::Reference => "reference",
                PlannerRole::Fixture => "fixture",
            },
            run.seed,
            run.status.name(),
            run.steps,
            run.proper_crossings,
            optional(run.lateral_min, 6),
            optional(run.stride_max_same_foot, 6),
            optional(run.turn_max_abs_rad.map(f64::to_degrees), 6),
            optional(run.spacing_min, 6),
            optional(run.spacing_max, 6),
            run.hard_violations,
            run.standability_failures,
            optional(run.goal_error_center, 6),
            run.reached,
            elapsed_text,
            run.path_sha256,
        );
    }
    out
}
