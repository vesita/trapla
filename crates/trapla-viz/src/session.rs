//! 场景来源与加载：显式文件（`--map` / `--path`）或内置沙盒场景（`trapla-lab`）。
//!
//! 两条来源最终都变成同一个 [`SceneInputs`]，下游（无头自检 / 渲染 / 沙盒）不再区分；
//! 唯一的分支点在这里，`--map` / `--path` 的读取口径与改动前逐字一致。
//!
//! 沙盒场景走 `trapla-lab` 的 `scenario::write_map_csv` 落成与 `data/csv/map.csv` 同格式的
//! 临时文件，再用同一个 [`Terrain::read_csv`] 读回 —— 场景生成与文件输入共用一条读取路径，
//! 不会出现「菜单里的地形」与「命令行里的地形」两套口径。

use std::path::PathBuf;

use bevy::prelude::Resource;
use trapla_lab::scenario;

use crate::data::{FootCounts, FootPath, Terrain, VizError};
use crate::metrics::MetricsPanel;
use crate::scene::auto_stride;

/// 沙盒内置地图的边长（行 = 列 = 它）。
///
/// 200 是 `trapla-lab` 场景库的默认尺寸：200×200 = 40000 顶点，在
/// [`crate::scene::MAX_VERTICES`] 以内，不用降采样就能看到完整地形。
pub const SANDBOX_SIZE: usize = 200;

/// 场景来源。
#[derive(Resource, Debug, Clone, PartialEq)]
pub enum SceneSource {
    /// 显式文件输入：与引入沙盒之前的命令行行为完全一致。
    Files {
        /// 高度场 CSV。
        map: PathBuf,
        /// 落足点 CSV。
        path: PathBuf,
        /// 指标 JSON。
        metrics: Option<PathBuf>,
        /// 目标点（地图坐标）。
        goal: Option<(f64, f64)>,
        /// 降采样步长（`None` = 自动）。
        stride: Option<usize>,
    },
    /// `trapla-lab` 内置场景（尺寸固定 [`SANDBOX_SIZE`]）。
    Scenario {
        /// 场景 id（见 `scenario::builtin`）。
        id: String,
    },
}

/// 加载完成的场景输入（无头自检与渲染共用）。
#[derive(Debug)]
pub struct SceneInputs {
    /// 原始高度场。
    pub terrain: Terrain,
    /// 足中心点序列（沙盒场景没有规划路径，为空）。
    pub path: FootPath,
    /// 指标面板数据。
    pub metrics: MetricsPanel,
    /// 降采样步长。
    pub stride: usize,
    /// 目标点（地图坐标）。
    pub goal: (f64, f64),
    /// 起点（地图坐标）；文件模式为 `None`（起点由路径首点表示）。
    pub start: Option<(f64, f64)>,
    /// 自检摘要里打印的地图标签。
    pub map_label: String,
    /// 自检摘要里打印的路径标签。
    pub path_label: String,
    /// 沙盒场景的附加说明（类别 / 期望 / 栅格指纹）；文件模式为 `None`。
    pub scenario_detail: Option<String>,
}

impl SceneInputs {
    /// 左右脚点数（自检与渲染共用同一个口径）。
    pub fn foot_counts(&self) -> FootCounts {
        FootCounts::of_total(self.path.len())
    }
}

/// 按来源加载场景输入。
pub fn load(source: &SceneSource) -> Result<SceneInputs, VizError> {
    match source {
        SceneSource::Files {
            map,
            path,
            metrics,
            goal,
            stride,
        } => load_files(map, path, metrics.as_deref(), *goal, *stride),
        SceneSource::Scenario { id } => load_scenario(id),
    }
}

fn load_files(
    map: &std::path::Path,
    path: &std::path::Path,
    metrics: Option<&std::path::Path>,
    goal: Option<(f64, f64)>,
    requested_stride: Option<usize>,
) -> Result<SceneInputs, VizError> {
    let terrain = Terrain::read_csv(map)?;
    let footpath = FootPath::read_csv(path)?;
    let metrics = MetricsPanel::load(metrics);

    let stride = resolve_stride(terrain.rows(), terrain.cols(), requested_stride)?;
    warn_vertex_cap(&terrain, stride);

    let goal = goal.unwrap_or_else(|| {
        *footpath
            .points()
            .last()
            .expect("FootPath::read_csv 保证至少一个点")
    });

    Ok(SceneInputs {
        terrain,
        path: footpath,
        metrics,
        stride,
        goal,
        start: None,
        map_label: map.display().to_string(),
        path_label: path.display().to_string(),
        scenario_detail: None,
    })
}

fn load_scenario(id: &str) -> Result<SceneInputs, VizError> {
    let ids = [id.to_string()];
    let chosen = scenario::select(SANDBOX_SIZE, Some(&ids)).map_err(VizError::Cli)?;
    let scenario = chosen
        .first()
        .ok_or_else(|| VizError::Cli(format!("场景 `{id}` 选择结果为空（内置场景清单为空）")))?;

    let grid = scenario.grid();
    // 场景 → 与 map.csv 同格式的临时文件 → 同一个读取路径。
    // 目录带进程号：并发的两个 viz 进程互不覆盖；读完即删。
    let csv = std::env::temp_dir()
        .join(format!("trapla-viz-sandbox-{}", std::process::id()))
        .join(format!("{}.csv", scenario.id));
    scenario::write_map_csv(&csv, &grid).map_err(|source| VizError::Io {
        path: csv.clone(),
        source,
    })?;
    let terrain = Terrain::read_csv(&csv)?;
    // 临时文件删除失败不影响已读到的数据（目录按进程号隔离，下次启动不会读它）。
    if let Err(e) = std::fs::remove_file(&csv) {
        eprintln!("警告: 无法删除沙盒临时地图 {}: {e}", csv.display());
    }

    let stride = resolve_stride(terrain.rows(), terrain.cols(), None)?;
    warn_vertex_cap(&terrain, stride);

    Ok(SceneInputs {
        terrain,
        path: FootPath::empty(),
        metrics: MetricsPanel::load(None),
        stride,
        goal: scenario.goal,
        start: Some(scenario.start),
        map_label: format!(
            "场景 {} ({}x{}, seed={})",
            scenario.id, scenario.rows, scenario.cols, scenario.seed
        ),
        path_label: "内置场景（无 path.csv）".to_string(),
        scenario_detail: Some(format!(
            "场景: id={} class={} 期望={} {}x{} seed={} 栅格指纹={}",
            scenario.id,
            scenario.class.name(),
            scenario.expected.name(),
            scenario.rows,
            scenario.cols,
            scenario.seed,
            scenario.grid_fingerprint()
        )),
    })
}

/// 解析降采样步长：`None` = 自动，`Some(0)` 是非法输入。
pub fn resolve_stride(
    rows: usize,
    cols: usize,
    requested: Option<usize>,
) -> Result<usize, VizError> {
    match requested {
        None => Ok(auto_stride(rows, cols)),
        Some(0) => Err(VizError::Cli(
            "`--stride` 必须 >= 1；不传表示自动选步长".to_string(),
        )),
        Some(s) => Ok(s),
    }
}

/// 显式步长把顶点数顶到上限之外时告警（不改写步长，显式参数优先）。
fn warn_vertex_cap(terrain: &Terrain, stride: usize) {
    let (n_rows, n_cols) = crate::scene::sample_shape(terrain.rows(), terrain.cols(), stride);
    if n_rows * n_cols > crate::scene::MAX_VERTICES {
        eprintln!(
            "警告: --stride {stride} 产生 {} 个顶点，超过默认上限 {}；显式步长优先，不自动改写",
            n_rows * n_cols,
            crate::scene::MAX_VERTICES
        );
    }
}

/// 主菜单的选项面：**资源**承载，UI 代码只读它，不硬编码场景清单。
#[derive(Resource, Debug, Clone)]
pub struct MenuOptions {
    /// 内置场景尺寸。
    pub size: usize,
    /// 可选场景（顺序即内置清单顺序）。
    pub scenarios: Vec<MenuEntry>,
    /// 当前选中的下标。
    pub selected: usize,
}

/// 菜单里的一行场景。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MenuEntry {
    /// 场景 id（传给 [`SceneSource::Scenario`]）。
    pub id: String,
    /// 地形类别（`trapla-lab` 的 `ScenarioClass::name`）。
    pub class: String,
    /// 期望结果（`trapla-lab` 的 `ExpectedOutcome::name`）。
    pub expected: String,
}

impl MenuOptions {
    /// 从 `trapla-lab` 的内置场景清单构建（不生成栅格，只取元数据）。
    pub fn builtin() -> Self {
        let scenarios: Vec<MenuEntry> = scenario::builtin(SANDBOX_SIZE)
            .into_iter()
            .map(|s| MenuEntry {
                id: s.id,
                class: s.class.name().to_string(),
                expected: s.expected.name().to_string(),
            })
            .collect();
        Self {
            size: SANDBOX_SIZE,
            scenarios,
            selected: 0,
        }
    }

    /// 选中的场景 id；清单为空时 `None`。
    pub fn selected_id(&self) -> Option<&str> {
        self.scenarios.get(self.selected).map(|e| e.id.as_str())
    }
}

/// 菜单写入、`OnEnter(Playing)` 消费的选择（breeze 的 `MenuAction` 模式）。
#[derive(Resource, Debug, Default)]
pub struct MenuSelection(pub Option<String>);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn menu_options_follow_builtin_scenarios() {
        let options = MenuOptions::builtin();
        let builtin_ids: Vec<String> = scenario::builtin(SANDBOX_SIZE)
            .into_iter()
            .map(|s| s.id)
            .collect();
        let menu_ids: Vec<String> = options.scenarios.iter().map(|e| e.id.clone()).collect();
        assert_eq!(menu_ids, builtin_ids);
        assert_eq!(options.selected_id(), Some("flat"));
        assert!(options.scenarios.iter().any(|e| e.id == "corridor"));
    }

    #[test]
    fn scenario_load_roundtrips_through_map_csv() {
        let inputs = load(&SceneSource::Scenario {
            id: "step".to_string(),
        })
        .expect("内置 step 场景可加载");
        assert_eq!((inputs.terrain.rows(), inputs.terrain.cols()), (200, 200));
        // `step` 参数：x >= 100 起抬高 1.5，其余为 0（见 trapla-lab 的 builtin）。
        assert_eq!(inputs.terrain.value(50, 0), Some(0.0));
        assert_eq!(inputs.terrain.value(150, 0), Some(1.5));
        assert_eq!(inputs.start, Some((10.0, 100.0)));
        assert_eq!(inputs.goal, (150.0, 100.0));
        assert_eq!(inputs.path.len(), 0);
        assert_eq!(inputs.stride, 1);
        assert!(inputs.scenario_detail.is_some());
    }

    #[test]
    fn unknown_scenario_is_cli_error() {
        let err = load(&SceneSource::Scenario {
            id: "no-such-scenario".to_string(),
        })
        .expect_err("未知场景 id 必须报错");
        assert!(matches!(err, VizError::Cli(_)), "应归类为 Cli: {err}");
    }

    #[test]
    fn stride_zero_is_rejected() {
        assert!(matches!(
            resolve_stride(10, 10, Some(0)),
            Err(VizError::Cli(_))
        ));
        assert_eq!(resolve_stride(1000, 3000, None).unwrap(), 6);
    }
}
