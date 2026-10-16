//! 输入读取与错误分类。
//!
//! 这里**不依赖** `trapla-io`（其 `read_matrix` / `read_xy` 目前是 `todo!()`），
//! 以保证可视化前端的自检不被其它单元的进度阻塞；CSV 口径与 `data/csv/` 实测一致。

use std::fmt;
use std::path::{Path, PathBuf};

/// 可视化前端的错误分类。
#[derive(Debug)]
pub enum VizError {
    /// 读文件失败（不存在 / 无权限等）。
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// CSV 某一行无法解析：`line` 从 1 开始计。
    Format {
        path: PathBuf,
        line: usize,
        detail: String,
    },
    /// 矩阵每行列数不一致。
    Shape {
        path: PathBuf,
        line: usize,
        expected: usize,
        got: usize,
    },
    /// 路径文件里没有任何有效点。
    EmptyPath { path: PathBuf },
    /// 命令行参数非法。
    Cli(String),
    /// 渲染 / 截图阶段失败。
    Graphics(String),
}

impl fmt::Display for VizError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(f, "读文件失败 {}: {source}", path.display())
            }
            Self::Format { path, line, detail } => {
                write!(f, "{} 第 {line} 行格式错误: {detail}", path.display())
            }
            Self::Shape {
                path,
                line,
                expected,
                got,
            } => write!(
                f,
                "{} 第 {line} 行列数不一致: 期望 {expected} 列，实际 {got} 列",
                path.display()
            ),
            Self::EmptyPath { path } => {
                write!(
                    f,
                    "{} 里没有有效点位（至少需要 1 行 `x,y`）",
                    path.display()
                )
            }
            Self::Cli(msg) => write!(f, "{msg}"),
            Self::Graphics(msg) => write!(f, "{msg}"),
        }
    }
}

impl std::error::Error for VizError {}

/// `Result` 别名。
pub type Result<T> = std::result::Result<T, VizError>;

fn read_to_string(path: &Path) -> Result<String> {
    std::fs::read_to_string(path).map_err(|source| VizError::Io {
        path: path.to_path_buf(),
        source,
    })
}

/// 行优先高度场（`x` 是行、`y` 是列，沿用 `data/csv/map.csv` 约定）。
#[derive(Debug, Clone)]
pub struct Terrain {
    cells: Vec<f64>,
    rows: usize,
    cols: usize,
}

impl Terrain {
    /// 读无表头纯数值矩阵。空行跳过；各行列数必须一致。
    pub fn read_csv(path: &Path) -> Result<Self> {
        let text = read_to_string(path)?;
        let mut cells = Vec::new();
        let mut rows = 0usize;
        let mut cols = 0usize;
        for (i, raw) in text.lines().enumerate() {
            let line_no = i + 1;
            let line = raw.trim_end_matches('\r');
            if line.trim().is_empty() {
                continue;
            }
            let mut n = 0usize;
            for field in line.split(',') {
                let v: f64 = field.trim().parse().map_err(|e| VizError::Format {
                    path: path.to_path_buf(),
                    line: line_no,
                    detail: format!("`{}` 不是数值: {e}", field.trim()),
                })?;
                cells.push(v);
                n += 1;
            }
            if cols == 0 {
                cols = n;
            } else if n != cols {
                return Err(VizError::Shape {
                    path: path.to_path_buf(),
                    line: line_no,
                    expected: cols,
                    got: n,
                });
            }
            rows += 1;
        }
        Ok(Self { cells, rows, cols })
    }

    /// 行数。
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// 列数。
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// 单元格值；越界返回 `None`。
    pub fn value(&self, row: usize, col: usize) -> Option<f64> {
        if row < self.rows && col < self.cols {
            self.cells.get(row * self.cols + col).copied()
        } else {
            None
        }
    }

    /// 最接近 `(row, col)` 的单元格值（坐标先四舍五入再夹到界内）。
    pub fn value_nearest(&self, row: f64, col: f64) -> f64 {
        if self.rows == 0 || self.cols == 0 {
            return 0.0;
        }
        let r = (row.round().max(0.0) as usize).min(self.rows - 1);
        let c = (col.round().max(0.0) as usize).min(self.cols - 1);
        self.value(r, c).unwrap_or(0.0)
    }
}

/// 顺序足中心点序列（原始地图坐标：`x` 是行、`y` 是列）。
#[derive(Debug, Clone)]
pub struct FootPath {
    points: Vec<(f64, f64)>,
}

impl FootPath {
    /// 读 `x,y` 两列；首行是 `x,y` 表头时自动跳过。
    pub fn read_csv(path: &Path) -> Result<Self> {
        let text = read_to_string(path)?;
        let mut points = Vec::new();
        for (i, raw) in text.lines().enumerate() {
            let line_no = i + 1;
            let line = raw.trim_end_matches('\r').trim();
            if line.is_empty() {
                continue;
            }
            let Some((a, b)) = line.split_once(',') else {
                return Err(VizError::Format {
                    path: path.to_path_buf(),
                    line: line_no,
                    detail: format!("期望 `x,y` 两列，收到 `{line}`"),
                });
            };
            let a = a.trim();
            let b = b.trim();
            if a.eq_ignore_ascii_case("x") && b.eq_ignore_ascii_case("y") {
                continue; // 表头
            }
            let parse = |s: &str, axis: &str| -> Result<f64> {
                s.parse::<f64>().map_err(|e| VizError::Format {
                    path: path.to_path_buf(),
                    line: line_no,
                    detail: format!("{axis} 列 `{s}` 不是数值: {e}"),
                })
            };
            points.push((parse(a, "x")?, parse(b, "y")?));
        }
        if points.is_empty() {
            return Err(VizError::EmptyPath {
                path: path.to_path_buf(),
            });
        }
        Ok(Self { points })
    }

    /// 全部点位。
    pub fn points(&self) -> &[(f64, f64)] {
        &self.points
    }

    /// 点位个数（= 步数）。
    pub fn len(&self) -> usize {
        self.points.len()
    }
}

/// 分足：偶数下标 = 左脚，奇数下标 = 右脚（`task-10` 裁决 4，实测严格交替）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Foot {
    /// 左脚（偶数下标）。
    Left,
    /// 右脚（奇数下标）。
    Right,
}

impl Foot {
    /// 按下标判脚。
    pub fn of_index(i: usize) -> Self {
        if i.is_multiple_of(2) {
            Self::Left
        } else {
            Self::Right
        }
    }

    /// 中文名。
    pub fn label(self) -> &'static str {
        match self {
            Self::Left => "左脚",
            Self::Right => "右脚",
        }
    }
}

/// 左右脚点数统计。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FootCounts {
    /// 左脚点数（偶数下标）。
    pub left: usize,
    /// 右脚点数（奇数下标）。
    pub right: usize,
}

impl FootCounts {
    /// 从总点数算：左脚 `ceil(n/2)`、右脚 `floor(n/2)`。
    pub fn of_total(n: usize) -> Self {
        Self {
            left: n.div_ceil(2),
            right: n / 2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 分足：偶数下标 = 左脚，奇数下标 = 右脚（task-10 裁决，实测严格交替）。
    #[test]
    fn foot_parity() {
        assert_eq!(Foot::of_index(0), Foot::Left);
        assert_eq!(Foot::of_index(1), Foot::Right);
        assert_eq!(Foot::of_index(50), Foot::Left);
        assert_eq!(Foot::of_index(51), Foot::Right);
    }

    /// 左右脚计数 = ceil(n/2) / floor(n/2)，奇偶都要对。
    #[test]
    fn foot_counts_are_ceil_and_floor() {
        for n in 0..=7 {
            let c = FootCounts::of_total(n);
            assert_eq!(c.left, n.div_ceil(2));
            assert_eq!(c.right, n / 2);
            assert_eq!(c.left + c.right, n);
        }
        let c = FootCounts::of_total(51);
        assert_eq!((c.left, c.right), (26, 25));
    }

    /// 路径读取跳过 `x,y` 表头与空行；全空报错而不是 panic。
    #[test]
    fn path_reads_with_header_and_blank_lines() {
        let p = std::env::temp_dir().join("trapla-viz-test-path.csv");
        std::fs::write(&p, "x,y\n1.5,2.5\n\n-3,-4\n").unwrap();
        let path = FootPath::read_csv(&p).unwrap();
        assert_eq!(path.len(), 2);
        assert_eq!(path.points()[1], (-3.0, -4.0));
        std::fs::write(&p, "x,y\n\n").unwrap();
        assert!(matches!(
            FootPath::read_csv(&p),
            Err(VizError::EmptyPath { .. })
        ));
        let _ = std::fs::remove_file(&p);
    }
}
