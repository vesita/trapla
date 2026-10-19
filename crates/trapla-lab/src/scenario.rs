//! 场景层：**确定性**程序化地形工厂 + 场景 manifest。
//!
//! 场景 = 一个可复现的输入包（地形栅格 + start/goal + 期望类别）。生成器只依赖
//! `(id, 尺寸, seed)`，同一个场景在任何机器、任何次运行都产出同一张栅格 —— 指纹由
//! [`grid_fingerprint`] 给出（sha256），manifest 里落盘。
//!
//! 地形约定与 `trapla-core` 一致：格值即高度（格/c m），`+inf` 记障碍。

use std::fs;
use std::io::{self, Write as _};
use std::path::Path;

use serde::{Deserialize, Serialize};
use trapla_core::geom::{Dot, Idx};
use trapla_core::grid::{Grid, OBSTACLE};
use trapla_core::rng::Rng;

use crate::sha256;

/// 地形类别。六类地形 + 一个纯输入错误夹具。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioClass {
    /// 全平坦。
    Flat,
    /// 缓坡（线性坡度）。
    Slope,
    /// 台阶 / 断层（有限高度台阶；全高 +inf 断层是同类的极端取值）。
    Step,
    /// 颠簸（小振幅周期起伏）。
    Bumpy,
    /// 窄桥 / 走廊（两侧高墙之间的通道）。
    Corridor,
    /// 随机障碍场。
    ObstacleField,
    /// 输入边界夹具：地形平坦，但 start/goal 越界。
    InputBoundary,
}

impl ScenarioClass {
    pub fn name(self) -> &'static str {
        match self {
            Self::Flat => "flat",
            Self::Slope => "slope",
            Self::Step => "step",
            Self::Bumpy => "bumpy",
            Self::Corridor => "corridor",
            Self::ObstacleField => "obstacle_field",
            Self::InputBoundary => "input_boundary",
        }
    }
}

/// 场景的期望类别：跑分时用来判「框架有没有识别出该识别的东西」。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExpectedOutcome {
    /// 应该规划出一条到得了 goal 的路径。
    Plannable,
    /// 应该以「无路径」结束。
    NoPath,
    /// 应该在输入校验处被拒（坐标越界）。
    InputError,
}

impl ExpectedOutcome {
    pub fn name(self) -> &'static str {
        match self {
            Self::Plannable => "plannable",
            Self::NoPath => "no_path",
            Self::InputError => "input_error",
        }
    }
}

/// 生成参数。同一类别下不同取值就是不同的场景，不进代码分支。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ScenarioParams {
    Flat,
    Slope { grade: f64 },
    Step { at_x: usize, rise: f64 },
    Bumpy { amplitude: f64, period_x: f64, period_y: f64 },
    Corridor { band_y: (usize, usize), wall_height: f64 },
    ObstacleField { density: f64, clearance: usize },
    Walled { band_x: (usize, usize) },
    InputBoundary,
}

/// 一个场景的完整描述（即 manifest 的一行）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Scenario {
    pub id: String,
    pub class: ScenarioClass,
    pub expected: ExpectedOutcome,
    pub rows: usize,
    pub cols: usize,
    pub seed: u64,
    pub start: (f64, f64),
    pub goal: (f64, f64),
    pub params: ScenarioParams,
}

impl Scenario {
    pub fn start_dot(&self) -> Dot {
        Dot::new(self.start.0, self.start.1)
    }

    pub fn goal_dot(&self) -> Dot {
        Dot::new(self.goal.0, self.goal.1)
    }

    /// 与场景绑定的地形。
    pub fn terrain(&self) -> trapla_core::terrain::Terrain {
        trapla_core::terrain::Terrain::from_grid(self.grid())
    }

    /// 按参数生成栅格（行优先 `cells[x * cols + y]`）。
    pub fn grid(&self) -> Grid {
        let mut cells = Vec::with_capacity(self.rows * self.cols);
        let mut rng = Rng::from_seed(self.seed);
        let start_idx = self.start_dot().as_idx();
        let goal_idx = self.goal_dot().as_idx();

        for x in 0..self.rows {
            for y in 0..self.cols {
                cells.push(self.height_at(x, y, start_idx, goal_idx, &mut rng));
            }
        }

        Grid::new(cells, self.rows, self.cols).expect("生成器产出的元素数与 rows*cols 一致")
    }

    fn height_at(&self, x: usize, y: usize, start: Idx, goal: Idx, rng: &mut Rng) -> f64 {
        match &self.params {
            ScenarioParams::Flat | ScenarioParams::InputBoundary => 0.0,
            ScenarioParams::Slope { grade } => grade * (x + y) as f64,
            ScenarioParams::Step { at_x, rise } => {
                if x >= *at_x {
                    *rise
                } else {
                    0.0
                }
            }
            ScenarioParams::Bumpy {
                amplitude,
                period_x,
                period_y,
            } => {
                amplitude * ((x as f64 / period_x).sin() * (y as f64 / period_y).cos())
                    + amplitude * 0.5 * ((x as f64 / (period_x * 2.0)).cos())
            }
            ScenarioParams::Corridor { band_y, wall_height } => {
                if y >= band_y.0 && y < band_y.1 {
                    0.0
                } else {
                    *wall_height
                }
            }
            ScenarioParams::Walled { band_x } => {
                if x >= band_x.0 && x < band_x.1 {
                    OBSTACLE
                } else {
                    0.0
                }
            }
            ScenarioParams::ObstacleField { density, clearance } => {
                let near = |i: Idx| {
                    (i.x - x as i32).abs() <= *clearance as i32
                        && (i.y - y as i32).abs() <= *clearance as i32
                };
                if near(start) || near(goal) {
                    return 0.0;
                }
                // 逐格消耗随机序列：扫描顺序固定（x 外层、y 内层），因此格值与平台无关。
                let roll = rng.next_below(10_000) as f64 / 10_000.0;
                if roll < *density { OBSTACLE } else { 0.0 }
            }
        }
    }

    /// 本场景栅格的 sha256 指纹。
    pub fn grid_fingerprint(&self) -> String {
        grid_fingerprint(&self.grid())
    }
}

/// 栅格内容的 sha256 指纹：尺寸 + 每个格值的 `f64` 位模式（小端）。
///
/// 用位模式而不是十进制文本：`inf` / `-0.0` / `NaN` 这些没有十进制往返保证的值也能区分。
pub fn grid_fingerprint(grid: &Grid) -> String {
    let mut bytes = Vec::with_capacity(16 + grid.rows() * grid.cols() * 8);
    bytes.extend_from_slice(&(grid.rows() as u64).to_le_bytes());
    bytes.extend_from_slice(&(grid.cols() as u64).to_le_bytes());
    for x in 0..grid.rows() {
        for y in 0..grid.cols() {
            let value = grid.cell(Idx::new(x as i32, y as i32)).unwrap_or(f64::NAN);
            bytes.extend_from_slice(&value.to_bits().to_le_bytes());
        }
    }
    sha256::hex_of(&bytes)
}

/// 内置场景清单（尺寸可调；坐标按尺寸成比例，保证换尺寸后仍在缩放网格内可达）。
///
/// 六个地形类别各一个场景，外加两个期望类别夹具：`fault`（全高断层 ⇒ 预期无路径）与
/// `oob`（坐标越界 ⇒ 预期输入错）。
pub fn builtin(size: usize) -> Vec<Scenario> {
    let at = |fraction: f64| (size as f64 * fraction).round();
    let mid = size / 2;
    let wall_lo = mid.saturating_sub(size / 8);
    let wall_hi = mid + size / 8;
    let band_half = (size as f64 * 0.15) as usize;
    let band_lo = mid.saturating_sub(band_half);
    let band_hi = mid + band_half;

    let make = |id: &str,
                class: ScenarioClass,
                expected: ExpectedOutcome,
                start: (f64, f64),
                goal: (f64, f64),
                params: ScenarioParams| Scenario {
        id: id.to_string(),
        class,
        expected,
        rows: size,
        cols: size,
        seed: 42,
        start,
        goal,
        params,
    };

    vec![
        make(
            "flat",
            ScenarioClass::Flat,
            ExpectedOutcome::Plannable,
            (at(0.05), at(0.05)),
            (at(0.75), at(0.75)),
            ScenarioParams::Flat,
        ),
        make(
            "slope",
            ScenarioClass::Slope,
            ExpectedOutcome::Plannable,
            (at(0.05), at(0.05)),
            (at(0.75), at(0.75)),
            ScenarioParams::Slope { grade: 0.08 },
        ),
        make(
            "step",
            ScenarioClass::Step,
            ExpectedOutcome::Plannable,
            (at(0.05), at(0.5)),
            (at(0.75), at(0.5)),
            ScenarioParams::Step {
                at_x: mid,
                rise: 1.5,
            },
        ),
        make(
            "bumpy",
            ScenarioClass::Bumpy,
            ExpectedOutcome::Plannable,
            (at(0.05), at(0.05)),
            (at(0.75), at(0.75)),
            ScenarioParams::Bumpy {
                amplitude: 1.2,
                period_x: 6.0,
                period_y: 7.0,
            },
        ),
        make(
            "corridor",
            ScenarioClass::Corridor,
            ExpectedOutcome::Plannable,
            (at(0.05), at(0.5)),
            (at(0.75), at(0.5)),
            ScenarioParams::Corridor {
                band_y: (band_lo, band_hi),
                wall_height: 120.0,
            },
        ),
        make(
            "obstacles",
            ScenarioClass::ObstacleField,
            ExpectedOutcome::Plannable,
            (at(0.05), at(0.05)),
            (at(0.75), at(0.75)),
            ScenarioParams::ObstacleField {
                density: 0.05,
                clearance: 3,
            },
        ),
        make(
            "fault",
            ScenarioClass::Step,
            ExpectedOutcome::NoPath,
            (at(0.05), at(0.5)),
            (at(0.75), at(0.5)),
            ScenarioParams::Walled {
                band_x: (wall_lo, wall_hi),
            },
        ),
        make(
            "oob",
            ScenarioClass::InputBoundary,
            ExpectedOutcome::InputError,
            (at(0.05), at(0.05)),
            (size as f64 * 2.0, size as f64 * 2.0),
            ScenarioParams::InputBoundary,
        ),
    ]
}

/// 按 id 过滤内置场景；`None` 表示全部。
pub fn select(size: usize, ids: Option<&[String]>) -> Result<Vec<Scenario>, String> {
    let all = builtin(size);
    let Some(ids) = ids else {
        return Ok(all);
    };
    let mut chosen = Vec::with_capacity(ids.len());
    for id in ids {
        let hit = all
            .iter()
            .find(|scenario| &scenario.id == id)
            .ok_or_else(|| {
                let known: Vec<&str> = all.iter().map(|s| s.id.as_str()).collect();
                format!("未知场景 `{id}`；可用：{}", known.join(", "))
            })?;
        chosen.push(hit.clone());
    }
    Ok(chosen)
}

/// 写出地图 CSV（无表头；有限值用最短往返十进制，障碍写 `inf`）。
pub fn write_map_csv(path: &Path, grid: &Grid) -> io::Result<()> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent)?;
    }
    let file = fs::File::create(path)?;
    let mut writer = io::BufWriter::new(file);
    for x in 0..grid.rows() {
        for y in 0..grid.cols() {
            if y > 0 {
                write!(writer, ",")?;
            }
            let value = grid.cell(Idx::new(x as i32, y as i32)).unwrap_or(OBSTACLE);
            if value.is_infinite() {
                write!(writer, "inf")?;
            } else {
                write!(writer, "{value}")?;
            }
        }
        writeln!(writer)?;
    }
    writer.flush()
}

/// manifest 的 JSON 文本（数组，每项是场景 + 栅格指纹）。
pub fn manifest_json(scenarios: &[Scenario]) -> Result<String, serde_json::Error> {
    let items: Vec<serde_json::Value> = scenarios
        .iter()
        .map(|scenario| {
            let mut value = serde_json::to_value(scenario)?;
            if let serde_json::Value::Object(fields) = &mut value {
                fields.insert(
                    "grid_sha256".to_string(),
                    serde_json::Value::from(scenario.grid_fingerprint()),
                );
                fields.insert(
                    "class_name".to_string(),
                    serde_json::Value::from(scenario.class.name()),
                );
                fields.insert(
                    "expected_name".to_string(),
                    serde_json::Value::from(scenario.expected.name()),
                );
            }
            Ok(value)
        })
        .collect::<Result<_, serde_json::Error>>()?;
    Ok(format!(
        "{}\n",
        serde_json::to_string_pretty(&items)?
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builtin_covers_all_classes_and_expectations() {
        let scenarios = builtin(200);
        let classes: Vec<&str> = scenarios.iter().map(|s| s.class.name()).collect();
        for expected in [
            "flat",
            "slope",
            "step",
            "bumpy",
            "corridor",
            "obstacle_field",
        ] {
            assert!(classes.contains(&expected), "缺少类别 {expected}");
        }
        let outcomes: Vec<&str> = scenarios.iter().map(|s| s.expected.name()).collect();
        for expected in ["plannable", "no_path", "input_error"] {
            assert!(outcomes.contains(&expected), "缺少期望类别 {expected}");
        }
    }

    #[test]
    fn generation_is_deterministic() {
        for scenario in builtin(64) {
            let a = scenario.grid_fingerprint();
            let b = scenario.grid_fingerprint();
            assert_eq!(a, b, "场景 {} 两次生成不一致", scenario.id);
        }
    }

    #[test]
    fn obstacle_field_depends_only_on_seed() {
        let mut scenarios = builtin(64);
        let base = scenarios
            .iter()
            .find(|s| s.id == "obstacles")
            .expect("内置里有 obstacles")
            .clone();
        let mut other = base.clone();
        other.seed = 7;
        assert_ne!(base.grid_fingerprint(), other.grid_fingerprint());

        // 同 seed 的独立构造必须逐位一致。
        scenarios.push(base.clone());
        let again = scenarios
            .iter()
            .find(|s| s.id == "obstacles")
            .expect("内置里有 obstacles");
        assert_eq!(base.grid_fingerprint(), again.grid_fingerprint());
    }

    #[test]
    fn corridor_has_walls_and_open_band() {
        let scenario = builtin(200)
            .into_iter()
            .find(|s| s.id == "corridor")
            .expect("内置里有 corridor");
        let grid = scenario.grid();
        let ScenarioParams::Corridor { band_y, .. } = scenario.params else {
            panic!("corridor 的参数应是 Corridor");
        };
        for y in 0..scenario.cols {
            let inside = y >= band_y.0 && y < band_y.1;
            let value = grid.cell(Idx::new(100, y as i32)).expect("在图内");
            if inside {
                assert_eq!(value, 0.0);
            } else {
                assert!(value > 0.0, "y={y} 应是墙");
            }
        }
    }

    #[test]
    fn map_csv_round_trips_obstacles() {
        let scenario = builtin(64)
            .into_iter()
            .find(|s| s.id == "fault")
            .expect("内置里有 fault");
        let grid = scenario.grid();
        let dir = std::env::temp_dir().join(format!("trapla-lab-roundtrip-{}", std::process::id()));
        fs::create_dir_all(&dir).expect("建临时目录");
        let path = dir.join("fault.csv");
        write_map_csv(&path, &grid).expect("写地图");

        let read_back = trapla_io::csv::read_matrix(&path).expect("读回地图");
        assert_eq!(grid_fingerprint(&grid), grid_fingerprint(&read_back));
        fs::remove_dir_all(&dir).ok();
    }
}
