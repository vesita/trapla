//! 地图矩阵与点位序列的 CSV 读写。
//!
//! ## 契约（原 C++ 数据流的逐条复刻 + 三处修正）
//!
//! - 矩阵文件（`data/csv/map.csv`）：**无表头**、逗号分隔的纯数值；`map[x][y]` 里
//!   `x` 是行、`y` 是列，落进 [`Grid`] 就是 `cells[i * cols + j]`。
//!   空文件、空行、非数值、列数不齐都返回**带行号/列号/原文**的 [`IoError::Format`]，
//!   不 panic、不静默跳过（原 `reader.cpp:43` 的 `std::stod` 直接抛异常）。
//! - 点位文件（`data/csv/path_result.csv`）：表头原文 [`XY_HEADER`]（`x,y`，无空格）。
//! - 写出坐标用固定 [`XY_DECIMALS`] 位小数（`{:.6}`）：同一个 `Vec<Dot>` 两次写出的文件
//!   **逐字节相同**，与运行目录、exe 位置、区域设置都无关（原 `main.cpp:102-119` 直接用
//!   `operator<<` 的 6 位有效数字，既不可控也不可预测）。
//!
//! 精度取 6 位小数的理由：地图与步长都以「格」为单位、量级 10^3，规划出来的坐标是
//! `f64` 的任意值；6 位小数把量化误差压到 `5e-7` 格，比 1e-3 的间距容差
//! （`config::SEPARATION_EPS`）小 4 个数量级，读回后重算指标不受影响，同时保证
//! 输出文本短且稳定 —— 再加位数只是把 `f64` 的二进制噪声写进文件，对几何没有增益。

use std::fs;
use std::io::{BufWriter, Write as _};
use std::path::Path;

use trapla_core::geom::Dot;
use trapla_core::grid::Grid;

use crate::IoError;

/// 点位 CSV 的表头原文（`x,y`，不含空格）。
pub const XY_HEADER: &str = "x,y";

/// 写出坐标的固定小数位。
pub const XY_DECIMALS: usize = 6;

/// 读取无表头的纯数值矩阵（原 `map.csv` 格式）。
///
/// 行数 = 非空行数，列数 = 首行列数，其余各行必须与首行等长。返回的 [`Grid`] 用
/// 行优先 `cells[i * cols + j]`，即文件第 `i` 行第 `j` 列。
pub fn read_matrix(path: &Path) -> Result<Grid, IoError> {
    let text = fs::read_to_string(path)?;

    let mut cells: Vec<f64> = Vec::new();
    let mut rows = 0usize;
    let mut cols: Option<usize> = None;

    for (idx, line) in text.lines().enumerate() {
        let line_no = idx + 1;
        if line.trim().is_empty() {
            return Err(IoError::Format {
                line: line_no,
                detail: "空行：矩阵的每一行都必须有数值".to_string(),
            });
        }

        let fields: Vec<&str> = line.split(',').collect();
        let expected = *cols.get_or_insert(fields.len());
        if fields.len() != expected {
            return Err(IoError::Format {
                line: line_no,
                detail: format!("列数不符：首行 {expected} 列，本行 {} 列", fields.len()),
            });
        }

        for (col, raw) in fields.iter().enumerate() {
            cells.push(parse_cell(raw, line_no, col + 1)?);
        }
        rows += 1;
    }

    if rows == 0 {
        return Err(IoError::Format {
            line: 1,
            detail: "空文件：期望至少 1 行数值".to_string(),
        });
    }

    let cols = cols.unwrap_or(0);
    Ok(Grid::new(cells, rows, cols)?)
}

/// 写出点位序列，表头 [`XY_HEADER`]，坐标为固定 [`XY_DECIMALS`] 位小数。
///
/// 父目录不存在时先创建（原 `writer.cpp:55` 的 `createDirectories`）；文件用截断方式重写。
pub fn write_xy(path: &Path, pts: &[Dot]) -> Result<(), IoError> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent)?;
    }

    let file = fs::File::create(path)?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "{XY_HEADER}")?;
    for dot in pts {
        writeln!(
            writer,
            "{:.*},{:.*}",
            XY_DECIMALS,
            dot.x(),
            XY_DECIMALS,
            dot.y()
        )?;
    }
    writer.flush()?;
    Ok(())
}

/// 读取 `x,y` 两列；首行恰为 [`XY_HEADER`] 时跳过。
pub fn read_xy(path: &Path) -> Result<Vec<Dot>, IoError> {
    let text = fs::read_to_string(path)?;

    let mut points = Vec::new();
    for (idx, line) in text.lines().enumerate() {
        let line_no = idx + 1;
        if line.trim().is_empty() {
            return Err(IoError::Format {
                line: line_no,
                detail: "空行：每一行都必须是一个 x,y 点".to_string(),
            });
        }
        if idx == 0 && line.trim() == XY_HEADER {
            continue;
        }

        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() != 2 {
            return Err(IoError::Format {
                line: line_no,
                detail: format!("列数不符：期望 2 列（x,y），实际 {} 列", fields.len()),
            });
        }
        let x = parse_cell(fields[0], line_no, 1)?;
        let y = parse_cell(fields[1], line_no, 2)?;
        points.push(Dot::new(x, y));
    }

    if points.is_empty() {
        return Err(IoError::Format {
            line: 1,
            detail: "空文件：期望至少 1 个 x,y 点".to_string(),
        });
    }
    Ok(points)
}

/// 解析一个单元格；失败时给出**行号、列号与原文**。
fn parse_cell(raw: &str, line: usize, col: usize) -> Result<f64, IoError> {
    let raw = raw.trim();
    if raw.is_empty() {
        return Err(IoError::Format {
            line,
            detail: format!("第 {col} 列为空"),
        });
    }
    raw.parse::<f64>().map_err(|_| IoError::Format {
        line,
        detail: format!("第 {col} 列不是数值：`{raw}`"),
    })
}
