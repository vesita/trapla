//! `plan` 子命令：读地图 → 规划 → 写出路径与指标。

use std::process::ExitCode;
use std::time::Instant;

use trapla_core::config::RobotConfig;
use trapla_core::error::CoreError;
use trapla_core::geom::Dot;
use trapla_core::metrics::{self, PathMetrics};
use trapla_core::rng::Rng;
use trapla_core::robot::Robot;
use trapla_core::terrain::Terrain;
use trapla_io::csv::{read_matrix, write_xy};

use crate::report;
use crate::{DEFAULT_SEED, EXIT_INPUT, EXIT_INTERNAL, EXIT_NO_PATH, EXIT_OK, PlanArgs};

/// 一次规划的结果：seed、路径、该次指标。
type Run = (u64, Vec<Dot>, PathMetrics);

pub fn run(args: &PlanArgs) -> ExitCode {
    let repeat = args.repeat.unwrap_or(1);
    if repeat == 0 {
        eprintln!("错误: --repeat 至少为 1");
        return ExitCode::from(EXIT_INPUT);
    }
    let base_seed = args.seed.unwrap_or(DEFAULT_SEED);

    // 显式路径：直接读参数字符串，不经过「exe 父目录 + 相对路径」的任何拼接。
    eprintln!("读取地图: {}", args.map.display());
    let grid = match read_matrix(&args.map) {
        Ok(grid) => grid,
        Err(error) => {
            eprintln!("错误: {error}");
            return ExitCode::from(EXIT_INPUT);
        }
    };
    eprintln!("地图已读入: {} 行 x {} 列", grid.rows(), grid.cols());

    // 坐标越界是**输入错**（退 1），不是「无路径」（退 2）。边界判定用 `Grid::in_bounds`
    // 与 `Dot::as_idx`（四舍五入成 `Idx`，x 是行、y 是列），与规划器取 `start_idx/goal_idx`
    // 的方式同源，不另立一套范围判定。
    let start = Dot::new(args.start.x, args.start.y);
    let goal = Dot::new(args.goal.x, args.goal.y);
    for (label, dot) in [("起点", start), ("目标点", goal)] {
        if let Some(error) = out_of_map_message(&grid, label, dot) {
            eprintln!("错误: {error}");
            return ExitCode::from(EXIT_INPUT);
        }
    }

    let terrain = Terrain::from_grid(grid);
    let config = RobotConfig::default();

    let started = Instant::now();
    let mut runs: Vec<Run> = Vec::with_capacity(repeat);
    for i in 0..repeat {
        let seed = base_seed.wrapping_add(i as u64);
        let mut rng = Rng::from_seed(seed);
        let mut robot = Robot::new(config, start);

        eprintln!("[{}/{}] 规划中 (seed {seed}) ...", i + 1, repeat);
        let path = match robot.find_path(&terrain, goal, &mut rng) {
            Ok(path) => path,
            Err(CoreError::NoPath { .. }) => {
                eprintln!("错误: 规划失败：无路径 (seed {seed})");
                return ExitCode::from(EXIT_NO_PATH);
            }
            Err(error) => {
                eprintln!("错误: 内部错误: {error}");
                return ExitCode::from(EXIT_INTERNAL);
            }
        };
        if path.is_empty() {
            eprintln!("错误: 规划失败：无路径 (seed {seed})");
            return ExitCode::from(EXIT_NO_PATH);
        }

        let metrics = metrics::compute(&path, goal);
        eprintln!(
            "[{}/{}] 完成: {} 个点 (seed {seed})",
            i + 1,
            repeat,
            path.len()
        );
        runs.push((seed, path, metrics));
    }
    let elapsed = started.elapsed();

    if let Err(error) = write_xy(&args.out, &runs[0].1) {
        eprintln!("错误: 写出 {} 失败: {error}", args.out.display());
        return ExitCode::from(EXIT_INTERNAL);
    }
    eprintln!("路径已写入 {} ({} 点)", args.out.display(), runs[0].1.len());

    if let Some(metrics_path) = &args.metrics {
        let payload = match metrics_json(&runs) {
            Ok(payload) => payload,
            Err(error) => {
                eprintln!("错误: 指标序列化失败: {error}");
                return ExitCode::from(EXIT_INTERNAL);
            }
        };
        let text = match serde_json::to_string_pretty(&payload) {
            Ok(text) => format!("{text}\n"),
            Err(error) => {
                eprintln!("错误: 指标序列化失败: {error}");
                return ExitCode::from(EXIT_INTERNAL);
            }
        };
        if let Err(error) = std::fs::write(metrics_path, text) {
            eprintln!("错误: 写出 {} 失败: {error}", metrics_path.display());
            return ExitCode::from(EXIT_INTERNAL);
        }
        eprintln!("指标已写入 {}", metrics_path.display());
    }

    println!("runs: {repeat}");
    println!("seed: {base_seed}");
    report::print_summary(&runs[0].2);
    eprintln!("耗时: {:.3} s（共 {repeat} 次规划）", elapsed.as_secs_f64());

    ExitCode::from(EXIT_OK)
}

/// 指标 JSON：`N == 1` 写单次对象；`N > 1` 写数组，每项在指标对象上追加 `seed` 字段。
fn metrics_json(runs: &[Run]) -> Result<serde_json::Value, serde_json::Error> {
    if runs.len() == 1 {
        return serde_json::to_value(&runs[0].2);
    }

    let mut items = Vec::with_capacity(runs.len());
    for (seed, _, metrics) in runs {
        let mut item = serde_json::to_value(metrics)?;
        if let serde_json::Value::Object(fields) = &mut item {
            fields.insert("seed".to_string(), serde_json::Value::from(*seed));
        }
        items.push(item);
    }
    Ok(serde_json::Value::Array(items))
}

/// 坐标落在图外（或不是有限数）时返回给用户看的说明；在图内返回 `None`。
fn out_of_map_message(grid: &trapla_core::grid::Grid, label: &str, dot: Dot) -> Option<String> {
    let (x, y) = (dot.x(), dot.y());
    if x.is_finite() && y.is_finite() && grid.in_bounds(dot.as_idx()) {
        return None;
    }
    Some(format!(
        "{label} ({x:.6},{y:.6}) 超出地图范围（{} 行 × {} 列）",
        grid.rows(),
        grid.cols()
    ))
}
