//! `trapla-viz` 入口：三种模式（交互 / 无头自检 / 截图）共用同一套场景加载。
//!
//! 场景来源有两种（见 [`crate::session`]）：显式 `--map`/`--path` 文件，或
//! `trapla-lab` 的内置场景（`--scenario` / 交互模式的沙盒菜单）。

mod cli;
mod data;
mod menu;
mod metrics;
mod orbit_camera;
mod phase;
mod scene;
mod session;
mod viz;

use std::path::Path;
use std::process::ExitCode;

use clap::Parser;

use crate::cli::{Cli, Mode};
use crate::data::{Foot, VizError};
use crate::scene::{
    Downsample, MAX_VERTICES, build_terrain_mesh, median_adjacent_normal_angle_deg,
    median_high_frequency_cm, sample_shape,
};
use crate::session::{SceneInputs, SceneSource};

/// 足朝向重建约定：默认提交语义（`smoothed`）；`raw` 只由负向对照开关打开。
fn foot_heading(raw: bool) -> trapla_lab::validate::RzConvention {
    if raw {
        trapla_lab::validate::RzConvention::Raw
    } else {
        trapla_lab::validate::RzConvention::Smoothed
    }
}

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
    let source = resolve_source(cli, &mode).map_err(VizError::Cli)?;
    // 视觉参数在所有模式里都先校验（fail-closed）；V1 的默认值不打印任何东西、输出不变。
    let exaggeration = cli.exaggeration().map_err(VizError::Cli)?;
    let foot_scale = cli.foot_scale().map_err(VizError::Cli)?;
    let foot_thickness = cli.foot_thickness().map_err(VizError::Cli)?;
    let base_depth = cli.base_depth().map_err(VizError::Cli)?;
    let foot_heading = foot_heading(cli.raw_foot_heading);
    let downsample = if cli.nearest_sampling {
        Downsample::Nearest
    } else {
        Downsample::WindowMean
    };

    match &mode {
        Mode::HeadlessSelftest => {
            let inputs = session::load(&source)?;
            headless_selftest(&inputs, exaggeration, downsample, cli.mesh_metrics);
            Ok(())
        }
        Mode::Screenshot(png) => {
            // 先删旧文件：否则「文件已存在」会让截图成功判据变成假阳性。
            match std::fs::remove_file(png) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => {
                    return Err(VizError::Graphics(format!(
                        "无法删除旧截图 {}: {e}",
                        png.display()
                    )));
                }
            }
            let config = viz::RunConfig {
                mode: mode.clone(),
                frames: cli.frames,
                source,
                // 截图模式默认跳过菜单；`--menu` 是对照开关（用于验证跳过确实生效）。
                start_in_menu: cli.menu,
                exaggeration,
                foot_scale,
                foot_thickness,
                base_depth,
                downsample,
                foot_heading,
            };
            viz::run(config).map_err(VizError::Graphics)?;
            verify_png(png)
        }
        Mode::Interactive => {
            let config = viz::RunConfig {
                mode: mode.clone(),
                frames: cli.frames,
                source,
                start_in_menu: cli.menu || !cli.has_explicit_inputs(),
                exaggeration,
                foot_scale,
                foot_thickness,
                base_depth,
                downsample,
                foot_heading,
            };
            viz::run(config).map_err(VizError::Graphics)
        }
    }
}

/// 由命令行参数推导场景来源。
///
/// - `--map` 与 `--path` 必须同时给；给了就与改动前行为完全一致；
/// - `--scenario <id>` 用内置场景，与文件输入互斥；
/// - 交互模式两者都不给 → 进沙盒菜单（菜单选场景）；无头/截图模式则要求显式来源。
fn resolve_source(cli: &Cli, mode: &Mode) -> Result<SceneSource, String> {
    let files = match (&cli.map, &cli.path) {
        (Some(map), Some(path)) => Some(SceneSource::Files {
            map: map.clone(),
            path: path.clone(),
            metrics: cli.metrics.clone(),
            goal: cli.goal.map(|xy| (xy.x, xy.y)),
            stride: cli.stride,
        }),
        (None, None) => None,
        (Some(_), None) | (None, Some(_)) => {
            return Err("`--map` 与 `--path` 必须同时给；只给一个无法确定场景输入".to_string());
        }
    };

    if let Some(scenario) = &cli.scenario {
        if files.is_some() {
            return Err(
                "`--scenario` 与 `--map`/`--path` 互斥：一次只能选一种场景来源".to_string(),
            );
        }
        if cli.metrics.is_some() || cli.goal.is_some() || cli.stride.is_some() {
            return Err(
                "`--metrics` / `--goal` / `--stride` 只对 `--map`/`--path` 输入有效".to_string(),
            );
        }
        return Ok(SceneSource::Scenario {
            id: scenario.clone(),
        });
    }

    if let Some(files) = files {
        return Ok(files);
    }

    match mode {
        Mode::Interactive => {
            if cli.metrics.is_some() || cli.goal.is_some() || cli.stride.is_some() {
                return Err(
                    "`--metrics` / `--goal` / `--stride` 只对 `--map`/`--path` 输入有效"
                        .to_string(),
                );
            }
            // 无显式输入：进菜单，默认选中第一个内置场景（菜单里可改）。
            let id = session::MenuOptions::builtin()
                .selected_id()
                .unwrap_or("flat")
                .to_string();
            Ok(SceneSource::Scenario { id })
        }
        _ => Err(
            "`--headless-selftest` / `--screenshot` 需要显式场景来源：`--map` 与 `--path` 同时给，或用 `--scenario <id>`"
                .to_string(),
        ),
    }
}

/// 无头自检：不建窗口，加载 + 构建场景 + 打印摘要。
///
/// `mesh_metrics` 打开时**额外**打印一行斑点指标（默认关闭，保证既有验收输出逐行不变）。
fn headless_selftest(
    inputs: &SceneInputs,
    exaggeration: f32,
    downsample: Downsample,
    mesh_metrics: bool,
) {
    let terrain = &inputs.terrain;
    let path = &inputs.path;
    let stride = inputs.stride;
    let (n_rows, n_cols) = sample_shape(terrain.rows(), terrain.cols(), stride);
    let mesh = build_terrain_mesh(terrain, stride, exaggeration, downsample);
    let counts = inputs.foot_counts();

    println!("=== trapla-viz 无头自检（未创建窗口） ===");
    if let Some(detail) = &inputs.scenario_detail {
        println!("{detail}");
    }
    println!(
        "地图: {}  行={} 列={} 原始格={}",
        inputs.map_label,
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
    if mesh_metrics {
        let mode = match downsample {
            Downsample::WindowMean => "窗口均值",
            Downsample::Nearest => "最近邻",
        };
        println!(
            "显示网格法线: 相邻顶点夹角中位数 = {:.4}°（降采样={mode}）",
            median_adjacent_normal_angle_deg(terrain, stride, downsample)
        );
        println!(
            "显示网格斑纹: 二阶差分中位数 = {:.4} cm（降采样={mode}）",
            median_high_frequency_cm(terrain, stride, downsample)
        );
    }
    println!("路径: {}  点数={}", inputs.path_label, path.len());
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
