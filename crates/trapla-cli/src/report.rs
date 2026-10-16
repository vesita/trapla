//! 结果摘要：`plan` 与 `analyze` 共用的字段表。
//!
//! 摘要一律走 **stdout**；进度与诊断只在各自模块里 `eprintln!`，不混进 stdout。

use trapla_core::metrics::PathMetrics;

/// 打印 `PathMetrics` 的关键字段（浮点固定 6 位小数，与 CSV 输出精度一致）。
pub(crate) fn print_summary(metrics: &PathMetrics) {
    println!("steps: {}", metrics.steps);
    println!("total_length: {:.6}", metrics.total_length);
    println!("goal_error: {:.6}", metrics.goal_error);
    println!("predicate_crossings: {}", metrics.predicate_crossings);
    println!("proper_crossings: {}", metrics.proper_crossings);
}
