//! `trapla-viz` 入口：三种模式（交互 / 无头自检 / 截图）共用同一套输入加载。

mod cli;
mod data;
mod metrics;
mod orbit_camera;
mod scene;
mod viz;

use std::path::Path;
use std::process::ExitCode;

use clap::Parser;

use crate::cli::{Cli, Mode};
use crate::data::{Foot, FootCounts, FootPath, Terrain, VizError};
use crate::metrics::MetricsPanel;
use crate::scene::{auto_stride, build_terrain_mesh, sample_shape, MAX_VERTICES};

fn main() -> ExitCode {
    let cli = Cli::parse();
    match run(&cli) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("trapla-viz 错误: {e}");
            ExitCode::from(1)
        }
    }
}

fn run(cli: &Cli) -> Result<(), VizError> {
    let mode = cli.mode().map_err(VizError::Cli)?;
    let terrain = Terrain::read_csv(&cli.map)?;
    let path = FootPath::read_csv(&cli.path)?;
    let metrics = MetricsPanel::load(cli.metrics.as_deref());

    let stride = match cli.stride {
        None => auto_stride(terrain.rows(), terrain.cols()),
        Some(0) => {
            return Err(VizError::Cli(
                "`--stride` 必须 >= 1；不传表示自动选步长".to_string(),
            ));
        }
        Some(s) => s,
    };
    let (n_rows, n_cols) = sample_shape(terrain.rows(), terrain.cols(), stride);
    if n_rows * n_cols > MAX_VERTICES {
        eprintln!(
            "警告: --stride {stride} 产生 {} 个顶点，超过默认上限 {MAX_VERTICES}；显式步长优先，不自动改写",
            n_rows * n_cols
        );
    }

    let goal = cli.goal.map_or_else(
        || {
            path.points()
                .last()
                .copied()
                .expect("FootPath::read_csv 保证至少一个点")
        },
        |xy| (xy.x, xy.y),
    );

    let inputs = viz::SceneInputs {
        terrain,
        path,
        metrics,
        stride,
        goal,
    };

    match &mode {
        Mode::HeadlessSelftest => headless_selftest(cli, &inputs),
        Mode::Screenshot(png) => {
            // 先删旧文件：否则「文件已存在」会让截图成功判据变成假阳性。
            match std::fs::remove_file(png) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => {
                    return Err(VizError::Graphics(format!(
                        "无法删除旧截图 {}: {e}",
                        png.display()
                    )))
                }
            }
            viz::run(inputs, &mode, cli.frames).map_err(VizError::Graphics)?;
            verify_png(png)
        }
        Mode::Interactive => viz::run(inputs, &mode, cli.frames).map_err(VizError::Graphics),
    }
}

/// 无头自检：不建窗口，加载 + 构建场景 + 打印摘要。
fn headless_selftest(cli: &Cli, inputs: &viz::SceneInputs) -> Result<(), VizError> {
    let terrain = &inputs.terrain;
    let path = &inputs.path;
    let stride = inputs.stride;
    let (n_rows, n_cols) = sample_shape(terrain.rows(), terrain.cols(), stride);
    let mesh = build_terrain_mesh(terrain, stride);
    let counts = FootCounts::of_total(path.len());

    println!("=== trapla-viz 无头自检（未创建窗口） ===");
    println!(
        "地图: {}  行={} 列={} 原始格={}",
        cli.map.display(),
        terrain.rows(),
        terrain.cols(),
        terrain.rows() * terrain.cols()
    );
    println!(
        "降采样: stride={stride}  采样 {n_rows} 行 x {n_cols} 列  = {} 顶点（上限 {MAX_VERTICES}）",
        n_rows * n_cols
    );
    println!(
        "顶点算式: {}",
        mesh.vertex_formula(terrain.rows(), terrain.cols())
    );
    println!(
        "地形网格实测: Mesh::count_vertices()={}  三角形={}",
        mesh.mesh.count_vertices(),
        mesh.triangle_count()
    );
    println!("路径: {}  点数={}", cli.path.display(), path.len());
    println!(
        "  {}点数（偶数下标）={}  {}点数（奇数下标）={}",
        Foot::Left.label(),
        counts.left,
        Foot::Right.label(),
        counts.right
    );
    println!(
        "  对照: ceil(n/2)={} floor(n/2)={}  ->  {}",
        path.len().div_ceil(2),
        path.len() / 2,
        if counts.left == path.len().div_ceil(2) && counts.right == path.len() / 2 {
            "一致"
        } else {
            "不一致"
        }
    );
    println!(
        "目标点（地图坐标 x,y）: {:.4}, {:.4}",
        inputs.goal.0, inputs.goal.1
    );
    println!("指标: {}", inputs.metrics.summary());
    println!("=== 自检结束 ===");
    Ok(())
}

/// 截图产物核对：存在、非空、PNG 魔数。
fn verify_png(path: &Path) -> Result<(), VizError> {
    let bytes = std::fs::read(path)
        .map_err(|e| VizError::Graphics(format!("截图 {} 不存在或不可读: {e}", path.display())))?;
    if bytes.len() < 8 || &bytes[..8] != b"\x89PNG\r\n\x1a\n" {
        return Err(VizError::Graphics(format!(
            "{} 不是 PNG（前 8 字节不匹配）",
            path.display()
        )));
    }
    println!("截图已落盘: {} ({} 字节, PNG)", path.display(), bytes.len());
    Ok(())
}
