//! `analyze` 子命令：读 `x,y` 路径 → 调 `trapla_core::metrics::compute` → 打印 / 写 JSON。
//!
//! 指标逻辑只有 [`trapla_core::metrics::compute`] 一处，本模块不重算任何一项。

use std::process::ExitCode;

use trapla_core::geom::Dot;
use trapla_core::metrics;
use trapla_io::csv::read_xy;

use crate::report;
use crate::{AnalyzeArgs, EXIT_INPUT, EXIT_INTERNAL, EXIT_OK};

pub fn run(args: &AnalyzeArgs) -> ExitCode {
    let points = match read_xy(&args.path) {
        Ok(points) => points,
        Err(error) => {
            eprintln!("错误: {error}");
            return ExitCode::from(EXIT_INPUT);
        }
    };
    eprintln!("已读入 {} 个点: {}", points.len(), args.path.display());

    let goal = Dot::new(args.goal.x, args.goal.y);
    let metrics = metrics::compute(&points, goal);

    if let Some(out) = &args.out {
        let text = match serde_json::to_string_pretty(&metrics) {
            Ok(text) => text,
            Err(error) => {
                eprintln!("错误: 指标序列化失败: {error}");
                return ExitCode::from(EXIT_INTERNAL);
            }
        };
        if let Err(error) = std::fs::write(out, format!("{text}\n")) {
            eprintln!("错误: 写出 {} 失败: {error}", out.display());
            return ExitCode::from(EXIT_INTERNAL);
        }
        eprintln!("指标已写入 {}", out.display());
    }

    report::print_summary(&metrics);
    ExitCode::from(EXIT_OK)
}
